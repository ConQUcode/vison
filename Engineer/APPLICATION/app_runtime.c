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

#if APP_CHASSIS_ONE_METER_ENABLED || APP_ARM_ENABLED
#include "chassis.h"
#include "ins_task.h"
#endif
#if APP_ARM_CORE_ENABLED
#include "arm.h"
#include "arm_kinematics.h"
#include "arm_tool.h"
#if APP_ARM_ENABLED
#include "fruit_usb_bridge.h"
#endif
#endif
#if APP_HUANER_FEEDBACK_ENABLED
#include "huaner_servo.h"
#endif

App_Arm_Teach_Debug_s g_app_arm_teach_debug;

#if APP_CHASSIS_ONE_METER_ENABLED || APP_ARM_ENABLED
/* INS_Init 返回的姿态快照只由 INS 写、底盘读。 */
static attitude_t *app_chassis_imu;
#endif

#if APP_HUANER_FEEDBACK_ENABLED
static uint8_t app_huaner_next_id;
static uint32_t app_huaner_next_tick;
#endif

#if APP_ARM_ENABLED && APP_ARM_TOOL_CENTER_TEST_ENABLE
/*
 * 抓放循环调度器。两条控制链的实现都在 app_arm_flow.c：
 * - 坐标抓取PickFlow：工具中心IK直线轨迹到参数化目标点后闭合夹爪。
 * - 固定角度放置PlaceFlow：写死关节角的转移、后方释放和恢复序列。
 * 本调度器只负责串联两个子流程；后续"底盘+双打点"任务层同样只需
 * 在这里按顺序启动子流程，不必修改子流程内部。
 */
typedef enum {
    APP_ARM_SCHED_WAIT_READY = 0, /* 等HOME完成、主机ready。 */
    APP_ARM_SCHED_PICK,           /* 坐标抓取子流程运行中。 */
    APP_ARM_SCHED_PLACE,          /* 固定角度放置子流程运行中。 */
    APP_ARM_SCHED_DONE,           /* 三次抓放完成后保持停止。 */
    APP_ARM_SCHED_FAILED          /* 任一子流程失败后原位保持。 */
} App_Arm_Sched_State_e;

static App_Arm_Sched_State_e app_arm_sched_state;
static uint8_t app_arm_pick_point_index;
static uint8_t app_arm_completed_pick_count;

/* 左右两个教导点交替抓取，首次从点1开始。 */
static const App_Arm_Pick_Target_s app_arm_pick_points[2] = {
    {
        { APP_ARM_PICK_POINT_1_Q1_DEG,
          APP_ARM_PICK_POINT_1_Q2_DEG,
          APP_ARM_PICK_POINT_1_Q3_DEG },
        APP_ARM_PICK_TOOL_RELATIVE_PITCH_DEG,
        APP_ARM_PICK_POINT_1_X_MM,
        APP_ARM_PICK_POINT_1_Y_MM,
        APP_ARM_PICK_POINT_1_Z_MM,
    },
    {
        { APP_ARM_PICK_POINT_2_Q1_DEG,
          APP_ARM_PICK_POINT_2_Q2_DEG,
          APP_ARM_PICK_POINT_2_Q3_DEG },
        APP_ARM_PICK_TOOL_RELATIVE_PITCH_DEG,
        APP_ARM_PICK_POINT_2_X_MM,
        APP_ARM_PICK_POINT_2_Y_MM,
        APP_ARM_PICK_POINT_2_Z_MM,
    },
};

/** READY后循环"点1抓放 -> 点2抓放"；失败即锁存停止。 */
static void AppArmPickPlaceTestTask(uint32_t now_ms)
{
    App_Arm_Flow_Status_e status = AppArmFlowPoll(now_ms);
    Arm_Host_Status_s host;

    if (status == APP_ARM_FLOW_FAILED || ChassisFaulted() != 0u) {
        app_arm_sched_state = APP_ARM_SCHED_FAILED;
    }
    switch (app_arm_sched_state) {
    case APP_ARM_SCHED_WAIT_READY:
        if (ChassisOneShotDone() == 0u ||
            ArmGetHostStatus(&host) == 0u || host.ready == 0u) {
            break;
        }
        if (AppArmFlowStartPick(
                &app_arm_pick_points[app_arm_pick_point_index], now_ms) != 0u) {
            app_arm_sched_state = APP_ARM_SCHED_PICK;
        }
        break;

    case APP_ARM_SCHED_PICK:
        if (status == APP_ARM_FLOW_DONE &&
            AppArmFlowStartPlace(now_ms) != 0u) {
            app_arm_sched_state = APP_ARM_SCHED_PLACE;
        }
        break;

    case APP_ARM_SCHED_PLACE: {
        uint8_t next_pick_point_index =
            (uint8_t)(app_arm_pick_point_index ^ 1u);
        if (status == APP_ARM_FLOW_DONE) {
            if ((uint8_t)(app_arm_completed_pick_count + 1u) >=
                    APP_ARM_TEST_PICK_COUNT) {
                app_arm_completed_pick_count++;
                g_app_arm_pick_place_test_debug.cycle_count++;
                app_arm_sched_state = APP_ARM_SCHED_DONE;
            } else if (ChassisStartOneShotStraight(
                    APP_ARM_BETWEEN_PICK_CHASSIS_DISTANCE_M,
                    APP_ARM_PRE_PICK_CHASSIS_TOLERANCE_M) != 0u) {
                app_arm_completed_pick_count++;
                g_app_arm_pick_place_test_debug.cycle_count++;
                app_arm_pick_point_index = next_pick_point_index;
                app_arm_sched_state = APP_ARM_SCHED_WAIT_READY;
            }
        }
        break;
    }

    case APP_ARM_SCHED_DONE:
    case APP_ARM_SCHED_FAILED:
    default:
        break;
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

#if APP_CHASSIS_ONE_METER_ENABLED
    USB_Init();
    ProtocolRuntimeInit();
    BuzzerInit();
    app_chassis_imu = INS_Init();
    (void)ChassisInit(app_chassis_imu);
#elif APP_ARM_ENABLED
    USB_Init();
    ProtocolRuntimeInit();
    FruitUsbBridgeInit();
    BuzzerInit();
    app_chassis_imu = INS_Init();
    (void)ChassisInitOneShotStraight(
        app_chassis_imu, APP_ARM_PRE_PICK_CHASSIS_DISTANCE_M,
        APP_ARM_PRE_PICK_CHASSIS_TOLERANCE_M);
    ArmInit();
#if APP_ARM_TOOL_CENTER_TEST_ENABLE
    AppArmFlowInit();
    app_arm_sched_state = APP_ARM_SCHED_WAIT_READY;
    app_arm_pick_point_index = 0u;
    app_arm_completed_pick_count = 0u;
#endif
#elif APP_HUANER_FEEDBACK_ENABLED
    if (HuanerServoInit() != 0u) {
        app_huaner_next_id = 1u;
        app_huaner_next_tick = 0u;
    }
#elif APP_ARM_TEACH_POINT_ENABLED
    memset(&g_app_arm_teach_debug, 0, sizeof(g_app_arm_teach_debug));
    ArmInit();
#endif
}

void AppImuTask(uint32_t now_ms)
{
#if APP_CHASSIS_ONE_METER_ENABLED || APP_ARM_ENABLED
    INS_Task();
    ChassisNotifyImuUpdate(now_ms);
#else
    (void)now_ms;
#endif
}

void AppChassisTask(uint32_t now_ms)
{
#if APP_CHASSIS_ONE_METER_ENABLED || APP_ARM_ENABLED
    ChassisTask(now_ms);
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
    AppArmPickPlaceTestTask(now_ms);
#endif
#elif APP_ARM_TEACH_POINT_ENABLED
    (void)now_ms;
    AppArmTeachPointUpdate();
#endif
#else
    (void)now_ms;
#endif
}

void AppMotorControlTask(uint32_t now_ms)
{
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
}

void AppUsbTask(uint32_t now_ms)
{
#if APP_ARM_ENABLED || APP_CHASSIS_ONE_METER_ENABLED
    static uint32_t last_daemon_tick;

    USB_ProcessTask();
    USB_TxTask();
    ProtocolRuntimeTask(now_ms);
#if APP_ARM_ENABLED
    FruitUsbBridgeTask(now_ms);
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
