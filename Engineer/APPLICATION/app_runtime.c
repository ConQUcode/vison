/**
 * @file app_runtime.c
 * @brief 根据 app_config.h 选择运行链，并隔离机械臂、底盘和舵机台架任务。
 */

#include "app_runtime.h"

#include <math.h>
#include <string.h>

#include "bsp_dwt.h"
#include "buzzer.h"
#include "daemon.h"
#include "protocol_runtime.h"
#include "usb.h"

#include "DJI_motor.h"
#include "dmmotor.h"

#if APP_CHASSIS_ENABLED
#include "chassis.h"
#include "ins_task.h"
#if APP_CHASSIS_ONE_METER_ENABLED
#include "chassis_config.h"
#endif
#endif
#if APP_ARM_CORE_ENABLED
#include "app_arm_command_id.h"
#include "arm.h"
#include "arm_kinematics.h"
#include "arm_tool.h"
#include "app_arm_flow.h"
#include "app_fruit_task.h"
#if APP_HOST_CONTROL_ENABLED
#include "upper_controller_bridge.h"
#endif
#endif
#if APP_HUANER_FEEDBACK_ENABLED
#include "huaner_servo.h"
#endif
#if APP_MG995_ENABLED
#include "mg995_servo.h"
#endif

App_Arm_Teach_Debug_s g_app_arm_teach_debug;
App_Arm_Bd_Observation_Debug_s g_app_arm_bd_observation_debug;

#if APP_CHASSIS_ENABLED
/* INS_Init 返回的姿态快照只由 INS 写、底盘读。 */
static attitude_t *app_chassis_imu;
#endif

#if APP_HUANER_FEEDBACK_ENABLED
static uint8_t app_huaner_next_id;
static uint32_t app_huaner_next_tick;
#endif

#if APP_CHASSIS_ONE_METER_ENABLED
typedef enum {
    APP_CHASSIS_TEST_STRAIGHT_1 = 0,
    APP_CHASSIS_TEST_TURN_RIGHT,
    APP_CHASSIS_TEST_STRAIGHT_2,
    APP_CHASSIS_TEST_WAIT,
    APP_CHASSIS_TEST_FAILED
} App_Chassis_Test_State_e;

static App_Chassis_Test_State_e app_chassis_test_state;
static uint32_t app_chassis_test_command_id;
static uint32_t app_chassis_test_wait_tick;

static uint8_t AppChassisTestSubmit(Chassis_Command_Type_e type,
                                    float distance_mm,
                                    float angle_deg)
{
    Chassis_Command_s command;

    memset(&command, 0, sizeof(command));
    command.command_id = ++app_chassis_test_command_id;
    command.type = type;
    command.distance_mm = distance_mm;
    command.angle_deg = angle_deg;
    command.tolerance_mm = CHASSIS_DISTANCE_TOLERANCE_M * 1000.0f;
    command.heading_mode = CHASSIS_HEADING_HOLD_START;
    return ChassisSubmitCommand(&command) == CHASSIS_COMMAND_ACCEPTED;
}

/** 通过公共命令接口复现原1m、右转90deg、1m循环。 */
static void AppChassisOneMeterTestTask(uint32_t now_ms)
{
    Chassis_Status_s status;

    if (ChassisGetStatus(&status) == 0u ||
        status.state == CHASSIS_STATE_FAULT) {
        app_chassis_test_state = APP_CHASSIS_TEST_FAILED;
        return;
    }
    if (app_chassis_test_state == APP_CHASSIS_TEST_FAILED ||
        status.state == CHASSIS_STATE_RUNNING ||
        status.state == CHASSIS_STATE_STOPPING ||
        status.state == CHASSIS_STATE_WAIT_READY) {
        return;
    }
    switch (app_chassis_test_state) {
    case APP_CHASSIS_TEST_STRAIGHT_1:
        if (AppChassisTestSubmit(CHASSIS_COMMAND_RELATIVE_STRAIGHT,
                CHASSIS_TEST_DISTANCE_M * 1000.0f, 0.0f)) {
            app_chassis_test_state = APP_CHASSIS_TEST_TURN_RIGHT;
        }
        break;
    case APP_CHASSIS_TEST_TURN_RIGHT:
        if (AppChassisTestSubmit(CHASSIS_COMMAND_RELATIVE_TURN,
                0.0f, CHASSIS_TURN_ANGLE_DEG)) {
            app_chassis_test_state = APP_CHASSIS_TEST_STRAIGHT_2;
        }
        break;
    case APP_CHASSIS_TEST_STRAIGHT_2:
        if (AppChassisTestSubmit(CHASSIS_COMMAND_RELATIVE_STRAIGHT,
                CHASSIS_TEST_DISTANCE_M * 1000.0f, 0.0f)) {
            app_chassis_test_wait_tick = 0u;
            app_chassis_test_state = APP_CHASSIS_TEST_WAIT;
        }
        break;
    case APP_CHASSIS_TEST_WAIT:
        if (app_chassis_test_wait_tick == 0u) {
            app_chassis_test_wait_tick = now_ms;
            break;
        }
        if ((uint32_t)(now_ms - app_chassis_test_wait_tick) >= 3000u) {
            app_chassis_test_state = APP_CHASSIS_TEST_STRAIGHT_1;
        }
        break;
    default:
        break;
    }
}
#endif

#if APP_ARM_BD_OBSERVATION_TEST_ENABLED
static void AppArmBdObservationUpdateWatch(
    const Arm_Host_Status_s *host)
{
    float dx;
    float dy;
    float dz;

    if (host == NULL) {
        return;
    }
    g_app_arm_bd_observation_debug.host_ready = host->ready;
    g_app_arm_bd_observation_debug.host_busy = host->busy;
    g_app_arm_bd_observation_debug.command_state =
        (uint32_t)(host->active_command_id ==
            g_app_arm_bd_observation_debug.command_id ?
            host->active_command_state : host->last_command_state);
    g_app_arm_bd_observation_debug.fault_code = host->fault_code;
    g_app_arm_bd_observation_debug.path_preflight_passed =
        g_arm_motion_debug.path_preflight_passed;
    g_app_arm_bd_observation_debug.trajectory_progress =
        host->trajectory_progress;
    for (uint8_t axis = 0u; axis < 3u; ++axis) {
        g_app_arm_bd_observation_debug.actual_q_deg[axis] =
            host->q_feedback_deg[axis];
    }
    g_app_arm_bd_observation_debug.actual_center_mm[0] =
        host->tool_tip_mm.x_mm;
    g_app_arm_bd_observation_debug.actual_center_mm[1] =
        host->tool_tip_mm.y_mm;
    g_app_arm_bd_observation_debug.actual_center_mm[2] =
        host->tool_tip_mm.z_mm;
    g_app_arm_bd_observation_debug.actual_tool_pitch_deg =
        host->tool_pitch_feedback_deg;
    dx = host->tool_tip_mm.x_mm -
        g_app_arm_bd_observation_debug.target_center_mm[0];
    dy = host->tool_tip_mm.y_mm -
        g_app_arm_bd_observation_debug.target_center_mm[1];
    dz = host->tool_tip_mm.z_mm -
        g_app_arm_bd_observation_debug.target_center_mm[2];
    g_app_arm_bd_observation_debug.center_error_mm =
        sqrtf(dx * dx + dy * dy + dz * dz);
    g_app_arm_bd_observation_debug.pitch_error_deg =
        host->tool_pitch_feedback_deg -
        g_app_arm_bd_observation_debug.target_tool_pitch_deg;
}

/** HOME完成后底座与主臂同步收拢转向，再沿侧面进入BD观察位并保持。 */
static void AppArmBdObservationTask(uint32_t now_ms)
{
    Arm_Host_Status_s host;
    Arm_Joint_Command_s joint_command;
    Arm_Tool_Center_Command_s center_command;
    Arm_Command_Result_e result;

    if (ArmGetHostStatus(&host) == 0u) {
        return;
    }
    AppArmBdObservationUpdateWatch(&host);
    if (host.state == ARM_HOST_STATE_FAULT ||
        host.state == ARM_HOST_STATE_ESTOP) {
        g_app_arm_bd_observation_debug.state =
            (uint8_t)APP_ARM_BD_OBSERVATION_FAILED;
        g_app_arm_bd_observation_debug.state_tick_ms = now_ms;
        return;
    }

    switch ((App_Arm_Bd_Observation_State_e)
            g_app_arm_bd_observation_debug.state) {
    case APP_ARM_BD_OBSERVATION_WAIT_READY:
        if (host.ready == 0u || host.busy != 0u) {
            break;
        }
        if (g_app_arm_bd_observation_debug.base_command_id == 0u) {
            g_app_arm_bd_observation_debug.base_command_id =
                AppArmCommandIdNext();
        }
        memset(&joint_command, 0, sizeof(joint_command));
        joint_command.command_id =
            g_app_arm_bd_observation_debug.base_command_id;
        joint_command.move_type = ARM_MOVE_LINEAR;
        memcpy(joint_command.q_deg, host.q_feedback_deg,
               sizeof(joint_command.q_deg));
        joint_command.q_deg[ARM_JOINT_BASE_YAW] =
            APP_ARM_BD_OBSERVATION_BASE_Q1_DEG;
        joint_command.q_deg[ARM_JOINT_SHOULDER] =
            APP_ARM_BD_OBSERVATION_STAGING_Q2_DEG;
        joint_command.q_deg[ARM_JOINT_ELBOW] =
            APP_ARM_BD_OBSERVATION_STAGING_Q3_DEG;
        joint_command.tool_relative_pitch_valid = 1u;
        joint_command.tool_relative_pitch_deg =
            APP_ARM_BD_OBSERVATION_TOOL_PITCH_DEG -
            ArmToolSmallLinkPitchFromJoint(joint_command.q_deg);
        g_app_arm_bd_observation_debug.command_id =
            joint_command.command_id;
        memcpy(g_app_arm_bd_observation_debug.base_target_q_deg,
               joint_command.q_deg, sizeof(joint_command.q_deg));
        g_app_arm_bd_observation_debug.base_tool_relative_pitch_deg =
            joint_command.tool_relative_pitch_deg;
        result = ArmSubmitJointCommand(&joint_command);
        g_app_arm_bd_observation_debug.submit_result = (uint32_t)result;
        if (result == ARM_COMMAND_OK) {
            g_app_arm_bd_observation_debug.state =
                (uint8_t)APP_ARM_BD_OBSERVATION_BASE_SUBMITTED;
            g_app_arm_bd_observation_debug.state_tick_ms = now_ms;
        } else if (result != ARM_COMMAND_BUSY &&
                   result != ARM_COMMAND_NOT_READY) {
            g_app_arm_bd_observation_debug.state =
                (uint8_t)APP_ARM_BD_OBSERVATION_FAILED;
            g_app_arm_bd_observation_debug.state_tick_ms = now_ms;
        }
        break;

    case APP_ARM_BD_OBSERVATION_BASE_SUBMITTED:
        if (host.last_command_id !=
            g_app_arm_bd_observation_debug.base_command_id) {
            break;
        }
        g_app_arm_bd_observation_debug.submit_result =
            (uint32_t)host.last_command_result;
        g_app_arm_bd_observation_debug.command_state =
            (uint32_t)host.last_command_state;
        if (host.last_command_state == ARM_COMMAND_STATE_COMPLETED &&
            host.last_command_result == ARM_COMMAND_OK) {
            g_app_arm_bd_observation_debug.state =
                (uint8_t)APP_ARM_BD_OBSERVATION_SUBMIT_TARGET;
            g_app_arm_bd_observation_debug.state_tick_ms = now_ms;
        } else if (host.last_command_state == ARM_COMMAND_STATE_REJECTED ||
                   host.last_command_state == ARM_COMMAND_STATE_CANCELLED ||
                   host.last_command_state == ARM_COMMAND_STATE_FAULTED) {
            g_app_arm_bd_observation_debug.state =
                (uint8_t)APP_ARM_BD_OBSERVATION_FAILED;
            g_app_arm_bd_observation_debug.state_tick_ms = now_ms;
        }
        break;

    case APP_ARM_BD_OBSERVATION_SUBMIT_TARGET:
        if (host.ready == 0u || host.busy != 0u) {
            break;
        }
        if (g_app_arm_bd_observation_debug.target_command_id == 0u) {
            g_app_arm_bd_observation_debug.target_command_id =
                AppArmCommandIdNext();
        }
        memset(&center_command, 0, sizeof(center_command));
        center_command.command_id =
            g_app_arm_bd_observation_debug.target_command_id;
        center_command.move_type = ARM_MOVE_LINEAR;
        center_command.target_center_mm.x_mm =
            APP_ARM_BD_OBSERVATION_X_MM;
        center_command.target_center_mm.y_mm =
            APP_ARM_BD_OBSERVATION_Y_MM;
        center_command.target_center_mm.z_mm =
            APP_ARM_BD_OBSERVATION_Z_MM;
        center_command.max_speed_mm_s =
            APP_ARM_BD_OBSERVATION_SPEED_MM_S;
        center_command.tool_pitch_valid = 1u;
        center_command.tool_pitch_deg =
            APP_ARM_BD_OBSERVATION_TOOL_PITCH_DEG;
        g_app_arm_bd_observation_debug.command_id =
            center_command.command_id;
        result = ArmSubmitToolCenterCommand(&center_command);
        g_app_arm_bd_observation_debug.submit_result = (uint32_t)result;
        if (result == ARM_COMMAND_OK) {
            g_app_arm_bd_observation_debug.state =
                (uint8_t)APP_ARM_BD_OBSERVATION_TARGET_SUBMITTED;
            g_app_arm_bd_observation_debug.state_tick_ms = now_ms;
        } else if (result != ARM_COMMAND_BUSY &&
                   result != ARM_COMMAND_NOT_READY) {
            g_app_arm_bd_observation_debug.state =
                (uint8_t)APP_ARM_BD_OBSERVATION_FAILED;
            g_app_arm_bd_observation_debug.state_tick_ms = now_ms;
        }
        break;

    case APP_ARM_BD_OBSERVATION_TARGET_SUBMITTED:
        if (host.last_command_id !=
            g_app_arm_bd_observation_debug.target_command_id) {
            break;
        }
        g_app_arm_bd_observation_debug.submit_result =
            (uint32_t)host.last_command_result;
        g_app_arm_bd_observation_debug.command_state =
            (uint32_t)host.last_command_state;
        if (host.last_command_state == ARM_COMMAND_STATE_COMPLETED &&
            host.last_command_result == ARM_COMMAND_OK) {
            g_app_arm_bd_observation_debug.state =
                (uint8_t)APP_ARM_BD_OBSERVATION_HOLDING;
            g_app_arm_bd_observation_debug.state_tick_ms = now_ms;
        } else if (host.last_command_state == ARM_COMMAND_STATE_REJECTED ||
                   host.last_command_state == ARM_COMMAND_STATE_CANCELLED ||
                   host.last_command_state == ARM_COMMAND_STATE_FAULTED) {
            g_app_arm_bd_observation_debug.state =
                (uint8_t)APP_ARM_BD_OBSERVATION_FAILED;
            g_app_arm_bd_observation_debug.state_tick_ms = now_ms;
        }
        break;

    case APP_ARM_BD_OBSERVATION_HOLDING:
    case APP_ARM_BD_OBSERVATION_FAILED:
    default:
        break;
    }
}
#endif

#if APP_ARM_POSTURE_TEST_ENABLED
/** AC区台架测试在公共单侧任务完成后提交另一侧，持续左右交替。 */
static void AppArmPostureTestTask(uint32_t now_ms)
{
    App_Arm_Side_Pick_Place_Status_e status =
        AppArmSidePickPlacePoll(now_ms);

    if (status == APP_ARM_SIDE_PICK_PLACE_DONE) {
        App_Fruit_Side_e completed_side =
            (App_Fruit_Side_e)g_app_arm_posture_test_debug.active_side;
        App_Fruit_Side_e next_side;

        if (completed_side == APP_FRUIT_SIDE_LEFT) {
            next_side = APP_FRUIT_SIDE_RIGHT;
        } else if (completed_side == APP_FRUIT_SIDE_RIGHT) {
            next_side = APP_FRUIT_SIDE_LEFT;
        } else {
            return;
        }
        (void)AppArmSidePickPlaceStart(next_side, now_ms);
    }
}
#endif
#if APP_ARM_TEACH_POINT_ENABLED
/** 汇总被动反馈和FK结果；只读状态，不提交任何电机或舵机动作。 */
static void AppArmTeachPointUpdate(void)
{
    const Arm_State_s *arm = ArmGetState();
    const Arm_Tool_State_s *tool = ArmToolGetState();
    uint8_t all_ready = 1u;
    uint8_t all_disabled = 1u;

    if (arm == NULL || tool == NULL) {
        return;
    }
    for (uint8_t axis = 0u; axis < 3u; ++axis) {
        g_app_arm_teach_debug.dm_online[axis] = arm->motor_online[axis];
        g_app_arm_teach_debug.dm_enabled[axis] = arm->motor_enabled[axis];
        g_app_arm_teach_debug.dm_joint_deg[axis] =
            arm->q_feedback_deg[axis];
        g_app_arm_teach_debug.dm_feedback_age_ms[axis] =
            g_arm_dm_debug.axis[axis].feedback_age_ms;
        if (arm->motor_online[axis] == 0u) {
            all_ready = 0u;
        }
        if (arm->motor_enabled[axis] != 0u) {
            all_disabled = 0u;
        }
    }
    for (uint8_t servo = 0u; servo < 2u; ++servo) {
        g_app_arm_teach_debug.servo_online[servo] =
            tool->servo_online[servo] != 0u &&
            tool->servo_feedback_valid[servo] != 0u;
        g_app_arm_teach_debug.servo_position[servo] =
            tool->servo_feedback_pos[servo];
        g_app_arm_teach_debug.servo_angle_deg[servo] =
            g_app_arm_teach_debug.servo_online[servo] != 0u ?
                (float)tool->servo_feedback_pos[servo] * 0.24f : NAN;
        if (g_app_arm_teach_debug.servo_online[servo] == 0u) {
            all_ready = 0u;
        }
    }
    g_app_arm_teach_debug.servo_unload_requested =
        tool->servo_unload_requested;
    g_app_arm_teach_debug.servo_unload_done = tool->servo_unload_done;
    g_app_arm_teach_debug.servo_unload_count = tool->servo_unload_count;
    g_app_arm_teach_debug.servo_unload_fail_count =
        tool->servo_unload_fail_count;
    if (tool->servo_unload_done == 0u) {
        all_ready = 0u;
    }
    g_app_arm_teach_debug.tool_pitch_deg = tool->tool_pitch_feedback_deg;
    g_app_arm_teach_debug.wrist_center_valid =
        arm->kinematics_valid != 0u &&
        arm->motor_online[0] != 0u && arm->motor_online[1] != 0u &&
        arm->motor_online[2] != 0u;
    g_app_arm_teach_debug.wrist_center_mm[0] = arm->wrist_center.x_mm;
    g_app_arm_teach_debug.wrist_center_mm[1] = arm->wrist_center.y_mm;
    g_app_arm_teach_debug.wrist_center_mm[2] = arm->wrist_center.z_mm;
    g_app_arm_teach_debug.tool_center_valid =
        g_app_arm_teach_debug.wrist_center_valid != 0u &&
        g_app_arm_teach_debug.servo_online[0] != 0u &&
        isfinite(tool->tool_pitch_feedback_deg);
    if (g_app_arm_teach_debug.tool_center_valid != 0u) {
        g_app_arm_teach_debug.tool_center_mm[0] = arm->tool_tip.x_mm;
        g_app_arm_teach_debug.tool_center_mm[1] = arm->tool_tip.y_mm;
        g_app_arm_teach_debug.tool_center_mm[2] = arm->tool_tip.z_mm;
    } else {
        g_app_arm_teach_debug.tool_center_mm[0] = NAN;
        g_app_arm_teach_debug.tool_center_mm[1] = NAN;
        g_app_arm_teach_debug.tool_center_mm[2] = NAN;
    }
    g_app_arm_teach_debug.feedback_ready = all_ready;
    g_app_arm_teach_debug.dm_all_disabled = all_disabled;
    g_app_arm_teach_debug.update_count++;
}
#endif

void AppInit(void)
{
    /* DWT 是 INS 和各控制模块的统一高精度时间基准，只初始化一次。 */
    DWT_Init(168u);

#if APP_ARM_CORE_ENABLED
    AppArmCommandIdInit();
#endif

#if APP_CHASSIS_ONE_METER_ENABLED
    USB_Init();
    ProtocolRuntimeInit();
    BuzzerInit();
    app_chassis_imu = INS_Init();
    (void)ChassisInit(app_chassis_imu);
    app_chassis_test_state = APP_CHASSIS_TEST_STRAIGHT_1;
    app_chassis_test_command_id = 0xC1000000u;
    app_chassis_test_wait_tick = 0u;
#elif APP_ARM_ENABLED
    USB_Init();
    ProtocolRuntimeInit();
    BuzzerInit();
    app_chassis_imu = INS_Init();
    (void)ChassisInit(app_chassis_imu);
    ArmInit();
#if APP_ARM_TOOL_CENTER_TEST_ENABLE
    AppArmFlowInit();
    AppFruitTaskInit();
#endif
#elif APP_HOST_CONTROL_ENABLED
    USB_Init();
    ProtocolRuntimeInit();
    UpperControllerBridgeInit();
    BuzzerInit();
    app_chassis_imu = INS_Init();
    (void)ChassisInit(app_chassis_imu);
    (void)Mg995ServoInit();
    AppArmFlowInit();
    AppArmSidePickPlaceInit();
    ArmInit();
#elif APP_ARM_POSTURE_TEST_ENABLED
    AppArmFlowInit();
    AppArmSidePickPlaceInit();
    (void)AppArmSidePickPlaceStart(APP_FRUIT_SIDE_LEFT, HAL_GetTick());
    ArmInit();
#elif APP_ARM_BD_OBSERVATION_TEST_ENABLED
    memset(&g_app_arm_bd_observation_debug, 0,
           sizeof(g_app_arm_bd_observation_debug));
    g_app_arm_bd_observation_debug.state =
        (uint8_t)APP_ARM_BD_OBSERVATION_WAIT_READY;
    g_app_arm_bd_observation_debug.target_center_mm[0] =
        APP_ARM_BD_OBSERVATION_X_MM;
    g_app_arm_bd_observation_debug.target_center_mm[1] =
        APP_ARM_BD_OBSERVATION_Y_MM;
    g_app_arm_bd_observation_debug.target_center_mm[2] =
        APP_ARM_BD_OBSERVATION_Z_MM;
    g_app_arm_bd_observation_debug.target_tool_pitch_deg =
        APP_ARM_BD_OBSERVATION_TOOL_PITCH_DEG;
    g_app_arm_bd_observation_debug.target_speed_mm_s =
        APP_ARM_BD_OBSERVATION_SPEED_MM_S;
    ArmInit();
#elif APP_HUANER_FEEDBACK_ENABLED
    if (HuanerServoInit() != 0u) {
        app_huaner_next_id = 1u;
        app_huaner_next_tick = 0u;
    }
#elif APP_ARM_TEACH_POINT_ENABLED
    memset(&g_app_arm_teach_debug, 0, sizeof(g_app_arm_teach_debug));
    ArmInit();
#elif APP_MG995_TEST_ENABLED
    (void)Mg995ServoInit();
#endif
}

void AppImuTask(uint32_t now_ms)
{
#if APP_CHASSIS_ENABLED
    INS_Task();
    ChassisNotifyImuUpdate(now_ms);
#else
    (void)now_ms;
#endif
}

void AppChassisTask(uint32_t now_ms)
{
#if APP_CHASSIS_ENABLED
    ChassisTask(now_ms);
#if APP_CHASSIS_ONE_METER_ENABLED
    AppChassisOneMeterTestTask(now_ms);
#endif
#else
    (void)now_ms;
#endif
}

void AppArmTask(uint32_t now_ms)
{
#if APP_ARM_CORE_ENABLED
    ArmTask();
    /* ArmTask内可能执行较长路径预检，后续应用状态机和电机发送使用新时间。 */
    now_ms = HAL_GetTick();
#if APP_ARM_ENABLED
#if APP_ARM_TOOL_CENTER_TEST_ENABLE
    AppFruitTask(now_ms);
#endif
#elif APP_ARM_POSTURE_TEST_ENABLED
    AppArmPostureTestTask(now_ms);
#elif APP_ARM_BD_OBSERVATION_TEST_ENABLED
    AppArmBdObservationTask(now_ms);
#elif APP_ARM_TEACH_POINT_ENABLED
    (void)now_ms;
    AppArmTeachPointUpdate();
#elif APP_HOST_CONTROL_ENABLED
    if (AppArmSidePickPlaceGetStatus() ==
        APP_ARM_SIDE_PICK_PLACE_RUNNING) {
        (void)AppArmSidePickPlacePoll(now_ms);
    } else {
        (void)AppArmFlowPoll(now_ms);
    }
#endif
#else
    (void)now_ms;
#endif
}

void AppMotorControlTask(uint32_t now_ms)
{
#if APP_MG995_TEST_ENABLED
    /* MG995专项模式不注册或周期控制任何达妙/DJI电机。 */
    (void)now_ms;
#else
    /* 两个驱动在零实例时均为空操作；集中调用可保持唯一任务所有权。 */
#if !APP_ARM_TEACH_POINT_ENABLED
    DMMotorControl(now_ms);
#else
    /*
     * 打点模式禁止达妙周期控制帧，三轴保持无力；达妙不主动上报反馈，
     * 改用低频失能查询帧维持角度反馈和在线判定。
     */
    ArmTeachPointFeedbackPoll(now_ms);
#endif
    DJIMotorControl();
#endif
}

void AppUsbTask(uint32_t now_ms)
{
#if APP_USB_ENABLED
    static uint32_t last_daemon_tick;

    USB_ProcessTask();
    USB_TxTask();
    ProtocolRuntimeTask(now_ms);
#if APP_HOST_CONTROL_ENABLED
    UpperControllerBridgeTask(now_ms);
#endif
    BuzzerTask(now_ms);
    if ((uint32_t)(now_ms - last_daemon_tick) >= 10u) {
        last_daemon_tick = now_ms;
        DaemonTask();
    }
#elif APP_HUANER_FEEDBACK_ENABLED
    HuanerServoTask(now_ms);
    if ((int32_t)(now_ms - app_huaner_next_tick) >= 0 &&
        HuanerServoRequestPosition(app_huaner_next_id) ==
            HUANER_SERVO_RESULT_OK) {
        app_huaner_next_id = app_huaner_next_id == 1u ? 2u : 1u;
        app_huaner_next_tick = now_ms + 25u;
    }
#else
    (void)now_ms;
#endif
}
