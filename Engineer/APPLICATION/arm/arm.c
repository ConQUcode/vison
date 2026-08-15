/**
 * @file arm.c
 * @brief 三台达妙机械臂的启动、使能、HOME、命令邮箱和故障保护状态机。
 */

#include "arm.h"

#include "arm_config.h"
#include "arm_internal.h"
#include "arm_kinematics.h"
#include "arm_tool.h"
#include "arm_trajectory.h"
#include "dmmotor.h"
#include "can.h"
#include "stm32f4xx_hal.h"

#include <math.h>
#include <string.h>

#define ARM_PI          3.14159265358979323846f
#define ARM_DEG_TO_RAD  (ARM_PI / 180.0f)
#define ARM_RAD_TO_DEG  (180.0f / ARM_PI)
#define ARM_AXIS_COUNT  3u
#define ARM_AXIS_NONE   0xffu
#define ARM_AUTO_INIT_ARM_AXIS_COUNT 2u

typedef struct {
    DM_MotorInstance *motor;
    float motor_zero_trim_rad;
    float logical_zero_deg;
    float direction;
    float soft_min_deg;
    float soft_max_deg;
    float escape_min_deg;
    float escape_max_deg;
    float target_deg;
    float feedback_deg;
} Arm_Joint_Motor_s;

typedef struct {
    uint32_t boot_tick;
    uint32_t state_tick;
    uint32_t stable_tick;
    uint32_t direction_tick;
    uint32_t auto_init_tick;
    uint32_t auto_init_arm_stable_tick[ARM_AUTO_INIT_ARM_AXIS_COUNT];
    uint32_t auto_init_arm_direction_tick[ARM_AUTO_INIT_ARM_AXIS_COUNT];
    float direction_start_deg;
    float active_target_deg;
    float active_speed_deg_s;
    float auto_init_arm_direction_start_deg[ARM_AUTO_INIT_ARM_AXIS_COUNT];
    float auto_init_hold_base_deg;
    uint8_t enter_mode_index;
    uint8_t disable_sent;
    uint8_t resetting;
    uint8_t auto_init_initialized;
    uint8_t elbow_coupling_active;
} Arm_Runtime_s;

typedef struct {
    Arm_Command_s command;
    volatile uint8_t pending;
    volatile uint8_t cancel_requested;
    volatile uint8_t stop_realtime_requested;
    volatile uint8_t estop_requested;
    volatile uint8_t fault_reset_requested;
    uint32_t cancel_command_id;
    uint32_t stop_realtime_command_id;
    uint32_t estop_command_id;
    uint32_t fault_reset_command_id;
    uint32_t active_fault_reset_request;
    uint32_t latest_received_command_id;
} Arm_Command_Mailbox_s;

Arm_State_s g_arm_state;
Arm_DM_Debug_s g_arm_dm_debug;
Arm_Home_Joint_Debug_s g_arm_home_joint_debug;
Arm_Kinematics_Debug_s g_arm_kinematics_debug;
Arm_Control_Debug_s g_arm_control_debug;
Arm_Teach_Point_s g_arm_teach_point;
Arm_Host_Status_s g_arm_host_status;
Arm_Boot_Debug_s g_arm_boot_debug;

static Arm_Joint_Motor_s arm_joint[ARM_AXIS_COUNT];
static Arm_Runtime_s arm_runtime;
static Arm_Command_Mailbox_s arm_command_mailbox;

static uint8_t ArmCommandPose(const float pose_q_deg[3],
                              float speed_deg_s,
                              uint8_t allow_escape);
static uint8_t ArmSetJointCommandForPose(uint8_t axis,
                                         float target_deg,
                                         const float pose_q_deg[3],
                                         float speed_deg_s,
                                         uint8_t allow_escape);
static Arm_Command_Result_e ArmExecuteJointCommand(
    const Arm_Joint_Command_s *command);
static Arm_Command_Result_e ArmExecuteCartesianCommand(
    const Arm_Cartesian_Command_s *command);
static Arm_Command_Result_e ArmExecuteRealtimeTarget(
    const Arm_Realtime_Cartesian_Target_s *target);
static Arm_Command_Result_e ArmExecuteToolCommand(
    const Arm_Command_Tool_s *command);
static void ArmProcessCommandMailbox(uint32_t now_ms);
static void ArmUpdateHostStatus(void);

static uint8_t ArmResolveHomePose(const float seed_q_deg[3],
                                  float target_q_deg[3],
                                  Arm_Position_s *target_wrist,
                                  Arm_IK_Status_e *ik_status)
{
    const Arm_Position_s target_endpoint = {
        ARM_USB_HOME_X_MM,
        ARM_USB_HOME_Y_MM,
        ARM_USB_HOME_Z_MM
    };
    Arm_IK_Result_s ik_result;

    if (seed_q_deg == NULL || target_q_deg == NULL ||
        target_wrist == NULL || ik_status == NULL) {
        return 0u;
    }
    /*
     * HOME和上位机Cartesian命令统一以ID1俯仰舵机轴心为受控点。
     * 这里必须先通过IK、FK误差、软件限位和自动运动区域检查，任何检查
     * 失败都不会向三台达妙下发目标。
     */
    memset(&ik_result, 0, sizeof(ik_result));
    *target_wrist = target_endpoint;
    *ik_status = ArmInverseKinematics3DOF(
        &target_endpoint, seed_q_deg, &ik_result);
    if (*ik_status != ARM_IK_OK ||
        ik_result.position_error_mm > ARM_LINEAR_FK_ERROR_MAX_MM ||
        !ArmJointPoseWithinSoftLimits(ik_result.q_deg) ||
        !ArmAutoPoseIsSafe(ik_result.q_deg)) {
        return 0u;
    }
    memcpy(target_q_deg, ik_result.q_deg, sizeof(ik_result.q_deg));
    return 1u;
}

static uint8_t ArmToolReadyForMotion(void)
{
#if ARM_TOOL_ENABLE != 0u
    const Arm_Tool_State_s *tool = ArmToolGetState();

    /*
     * 这里表示末端工具模型已经完成上电初始化，可以参与主臂解算。
     * USART6舵机的最近发送结果只作为末端状态观察量，不阻塞三达妙轨迹。
     */
    return tool->init_state == ARM_TOOL_INIT_DONE &&
           tool->servo_feedback_valid[0] != 0u &&
           tool->servo_feedback_valid[1] != 0u &&
           ArmToolGripperFaulted() == 0u;
#else
    return 1u;
#endif
}

static float ArmAbs(float value)
{
    return value < 0.0f ? -value : value;
}

static uint8_t ArmAxisIsValid(Arm_Joint_e axis)
{
    return axis <= ARM_JOINT_ELBOW;
}

static float ArmMotorRadToJointDegBase(Arm_Joint_e axis, float motor_rad)
{
    const Arm_Joint_Motor_s *joint;

    if (!ArmAxisIsValid(axis) || !isfinite(motor_rad)) {
        return NAN;
    }
    joint = &arm_joint[(uint8_t)axis];
    return joint->logical_zero_deg + joint->direction *
        (motor_rad - joint->motor_zero_trim_rad) * ARM_RAD_TO_DEG;
}

static float ArmJointDegToMotorRadBase(Arm_Joint_e axis, float joint_deg)
{
    const Arm_Joint_Motor_s *joint;

    if (!ArmAxisIsValid(axis) || !isfinite(joint_deg)) {
        return NAN;
    }
    joint = &arm_joint[(uint8_t)axis];
    if (ArmAbs(joint->direction) < 0.5f) {
        return NAN;
    }
    return joint->motor_zero_trim_rad +
        (joint_deg - joint->logical_zero_deg) * ARM_DEG_TO_RAD /
            joint->direction;
}

static float ArmElbowCouplingCompDeg(float shoulder_deg)
{
#if ARM_ELBOW_SHOULDER_COUPLING_ENABLE
    if (arm_runtime.elbow_coupling_active == 0u) {
        return 0.0f;
    }
    if (!isfinite(shoulder_deg)) {
        return NAN;
    }
    return ARM_ELBOW_SHOULDER_COUPLING *
        (shoulder_deg - ARM_ELBOW_COUPLING_REFERENCE_DEG);
#else
    (void)shoulder_deg;
    return 0.0f;
#endif
}

static float ArmMotorRadToJointDegWithShoulder(Arm_Joint_e axis,
                                               float motor_rad,
                                               float shoulder_deg)
{
    float joint_deg = ArmMotorRadToJointDegBase(axis, motor_rad);

    if (axis == ARM_JOINT_ELBOW) {
        float coupling_deg = ArmElbowCouplingCompDeg(shoulder_deg);
        if (!isfinite(joint_deg) || !isfinite(coupling_deg)) {
            return NAN;
        }
        joint_deg += coupling_deg;
    }
    return joint_deg;
}

static float ArmJointDegToMotorRadWithShoulder(Arm_Joint_e axis,
                                               float joint_deg,
                                               float shoulder_deg)
{
    if (axis == ARM_JOINT_ELBOW) {
        float coupling_deg = ArmElbowCouplingCompDeg(shoulder_deg);
        if (!isfinite(joint_deg) || !isfinite(coupling_deg)) {
            return NAN;
        }
        joint_deg -= coupling_deg;
    }
    return ArmJointDegToMotorRadBase(axis, joint_deg);
}

static float ArmJointDegToMotorRadForPose(Arm_Joint_e axis,
                                          float joint_deg,
                                          const float pose_q_deg[3])
{
    float shoulder_deg;

    if (axis != ARM_JOINT_ELBOW) {
        return ArmJointDegToMotorRadBase(axis, joint_deg);
    }
    if (pose_q_deg != NULL &&
        isfinite(pose_q_deg[ARM_JOINT_SHOULDER])) {
        shoulder_deg = pose_q_deg[ARM_JOINT_SHOULDER];
    } else if (isfinite(arm_joint[ARM_JOINT_SHOULDER].feedback_deg)) {
        shoulder_deg = arm_joint[ARM_JOINT_SHOULDER].feedback_deg;
    } else if (isfinite(arm_joint[ARM_JOINT_SHOULDER].target_deg)) {
        shoulder_deg = arm_joint[ARM_JOINT_SHOULDER].target_deg;
    } else {
        shoulder_deg = ARM_ELBOW_COUPLING_REFERENCE_DEG;
    }
    return ArmJointDegToMotorRadWithShoulder(axis, joint_deg, shoulder_deg);
}

float ArmMotorRadToJointDeg(Arm_Joint_e axis, float motor_rad)
{
    float shoulder_deg = arm_joint[ARM_JOINT_SHOULDER].feedback_deg;

    if (axis == ARM_JOINT_SHOULDER) {
        return ArmMotorRadToJointDegBase(axis, motor_rad);
    }
    if (!isfinite(shoulder_deg)) {
        shoulder_deg = ARM_ELBOW_COUPLING_REFERENCE_DEG;
    }
    return ArmMotorRadToJointDegWithShoulder(axis, motor_rad, shoulder_deg);
}

float ArmJointDegToMotorRad(Arm_Joint_e axis, float joint_deg)
{
    if (axis == ARM_JOINT_SHOULDER) {
        return ArmJointDegToMotorRadBase(axis, joint_deg);
    }
    return ArmJointDegToMotorRadForPose(axis, joint_deg, NULL);
}

float ArmMotorVelocityToJointDegS(Arm_Joint_e axis, float motor_rad_s)
{
    if (!ArmAxisIsValid(axis) || !isfinite(motor_rad_s)) {
        return NAN;
    }
    if (axis == ARM_JOINT_ELBOW) {
#if ARM_ELBOW_SHOULDER_COUPLING_ENABLE
        if (arm_runtime.elbow_coupling_active != 0u) {
            float shoulder_deg_s = ArmMotorVelocityToJointDegS(
                ARM_JOINT_SHOULDER,
                arm_joint[ARM_JOINT_SHOULDER].motor != NULL ?
                    arm_joint[ARM_JOINT_SHOULDER].motor->
                        measure.velocity_rad_s : 0.0f);
            if (!isfinite(shoulder_deg_s)) {
                return NAN;
            }
            return arm_joint[(uint8_t)axis].direction * motor_rad_s *
                ARM_RAD_TO_DEG +
                ARM_ELBOW_SHOULDER_COUPLING * shoulder_deg_s;
        }
#endif
    }
    return arm_joint[(uint8_t)axis].direction * motor_rad_s * ARM_RAD_TO_DEG;
}

static Arm_Limit_Result_e ArmCheckLimit(uint8_t axis, float q_deg)
{
    const Arm_Joint_Motor_s *joint = &arm_joint[axis];
    float tolerance = ARM_LIMIT_TOLERANCE_DEG;

    if (!isfinite(q_deg)) {
        return ARM_LIMIT_INVALID;
    }
    if (q_deg >= joint->soft_min_deg - tolerance &&
        q_deg <= joint->soft_max_deg + tolerance) {
        return ARM_LIMIT_INSIDE_SOFT;
    }
    if (q_deg >= joint->escape_min_deg - tolerance &&
        q_deg <= joint->escape_max_deg + tolerance) {
        return ARM_LIMIT_ESCAPE_ALLOWED;
    }
    return ARM_LIMIT_OUTSIDE_HARD;
}

static uint8_t ArmAllFeedbackValid(uint32_t now_ms)
{
    uint8_t axis;

    for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
        DM_MotorInstance *motor = arm_joint[axis].motor;
        if (motor == NULL || !DMMotorIsOnline(motor, now_ms) ||
            !isfinite(motor->measure.position_rad) ||
            !isfinite(motor->measure.velocity_rad_s)) {
            return 0u;
        }
    }
    return 1u;
}

static uint8_t ArmAllTargetsSynced(void)
{
    uint8_t axis;

    for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
        if (arm_joint[axis].motor == NULL ||
            arm_joint[axis].motor->target_synced == 0u) {
            return 0u;
        }
    }
    return 1u;
}

static uint8_t ArmAllModesConfirmed(void)
{
    uint8_t axis;

    for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
        if (!DMMotorModeConfirmed(arm_joint[axis].motor)) {
            return 0u;
        }
    }
    return 1u;
}

static uint8_t ArmAllMotorsEnabled(void)
{
    uint8_t axis;

    for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
        if (arm_joint[axis].motor == NULL ||
            arm_joint[axis].motor->control_enabled == 0u) {
            return 0u;
        }
    }
    return 1u;
}

static uint8_t ArmTemperatureAtOrAbove(float threshold_c)
{
    uint8_t axis;

    for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
        DM_MotorInstance *motor = arm_joint[axis].motor;
        if (motor != NULL &&
            (motor->measure.mos_temperature_c >= threshold_c ||
             motor->measure.rotor_temperature_c >= threshold_c)) {
            return 1u;
        }
    }
    return 0u;
}

static uint8_t ArmAnyMotorStateFault(void)
{
    uint8_t axis;

    for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
        if (DMMotorHasActiveStateFault(arm_joint[axis].motor)) {
            return 1u;
        }
    }
    return 0u;
}

static uint8_t ArmAnyTxFault(void)
{
    uint8_t axis;

    for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
        DM_MotorInstance *motor = arm_joint[axis].motor;
        if (motor != NULL &&
            (motor->tx_fault_latched != 0u ||
             motor->consecutive_tx_fail >= ARM_DM_TX_FAIL_LIMIT)) {
            return 1u;
        }
    }
    return 0u;
}

static uint8_t ArmAnyPreviouslySeenMotorOffline(uint32_t now_ms)
{
    uint8_t axis;

    for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
        DM_MotorInstance *motor = arm_joint[axis].motor;
        if (motor != NULL && motor->measure.feedback_valid != 0u &&
            !DMMotorIsOnline(motor, now_ms)) {
            return 1u;
        }
    }
    return 0u;
}

static void ArmDisableAll(void)
{
    uint8_t axis;

    for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
        DM_MotorInstance *motor = arm_joint[axis].motor;
        if (motor != NULL) {
            DMMotorDisable(motor);
        }
    }
}

static void ArmLatchFault(Arm_Fault_e fault)
{
    if (fault == ARM_FAULT_NONE) {
        fault = ARM_FAULT_CONFIG;
    }
    if (g_arm_state.fault_latched == ARM_FAULT_NONE ||
        fault == ARM_FAULT_EMERGENCY_STOP) {
        g_arm_state.fault_latched = fault;
    }
    g_arm_state.mode = fault == ARM_FAULT_EMERGENCY_STOP ?
        ARM_MODE_ESTOP : ARM_MODE_FAULT;
    g_arm_state.start_state = fault == ARM_FAULT_EMERGENCY_STOP ?
        ARM_START_ESTOP : ARM_START_FAULT;
    ArmAbortMotion(ARM_MOTION_FAULT_ABORT);
    arm_runtime.elbow_coupling_active = 0u;
    ArmToolStopServo1Tracking();
    ArmToolClearPendingCommands();
    ArmDisableAll();
    arm_runtime.disable_sent = 1u;
}

static void ArmSetStartState(Arm_Start_State_e state, uint32_t now_ms)
{
    Arm_Start_State_e previous_state = g_arm_state.start_state;

    g_arm_state.start_state = state;
    arm_runtime.state_tick = now_ms;
    arm_runtime.stable_tick = 0u;
    arm_runtime.direction_tick = 0u;
    arm_runtime.direction_start_deg = 0.0f;
    if (state == ARM_START_READY && previous_state != ARM_START_READY) {
        arm_runtime.auto_init_initialized = 0u;
    }
}

static uint8_t ArmSyncAllCurrentTargets(void)
{
    uint8_t axis;
    float pose_q_deg[3];

    if (arm_runtime.elbow_coupling_active == 0u) {
        for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
            DM_MotorInstance *motor = arm_joint[axis].motor;
            if (motor == NULL || !DMMotorHoldCurrentPosition(motor)) {
                return 0u;
            }
            arm_joint[axis].target_deg = arm_joint[axis].feedback_deg;
        }
        return 1u;
    }
    pose_q_deg[0] = arm_joint[0].feedback_deg;
    pose_q_deg[1] = arm_joint[1].feedback_deg;
    pose_q_deg[2] = arm_joint[2].feedback_deg;
    return ArmCommandPose(pose_q_deg, ARM_ESCAPE_SPEED_DEG_S, 1u);
}

static uint8_t ArmEnterAllModes(void)
{
    uint8_t axis;
    uint8_t all_sent = 1u;

    for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
        DM_MotorInstance *motor = arm_joint[axis].motor;
        if (motor == NULL || !DMMotorEnterMode(motor)) {
            all_sent = 0u;
            continue;
        }
        g_arm_dm_debug.enter_mode_sent[axis] = 1u;
    }
    return all_sent;
}

static void ArmProcessEnableOnly(uint32_t now_ms)
{
    uint8_t axis;

    switch (g_arm_state.start_state) {
        case ARM_START_REGISTERED:
            if ((uint32_t)(now_ms - arm_runtime.boot_tick) <
                ARM_DM_POWER_ON_DELAY_MS) {
                break;
            }
            ArmSetStartState(ARM_START_WAIT_PASSIVE_FEEDBACK, now_ms);
            break;

        case ARM_START_WAIT_PASSIVE_FEEDBACK:
            /*
             * 达妙失能红灯状态可能不主动反馈。只被动监听300 ms，随后无论
             * 是否已有反馈，都必须先发送三轴Enter Motor Mode。
             */
            if ((uint32_t)(now_ms - arm_runtime.state_tick) <
                ARM_PASSIVE_FEEDBACK_WAIT_MS) {
                break;
            }
            ArmSetStartState(ARM_START_ENTER_ALL_MODES, now_ms);
            break;

        case ARM_START_ENTER_ALL_MODES:
            for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
                if (DMMotorEnterMode(arm_joint[axis].motor)) {
                    g_arm_dm_debug.enter_mode_sent[axis] = 1u;
                }
            }
            ArmSetStartState(ARM_START_WAIT_ENABLE_CONFIRM, now_ms);
            break;

        case ARM_START_WAIT_ENABLE_CONFIRM:
            /*
             * 未收到反馈时持续重发正确Motor ID的使能帧，不超时、不失能。
             */
            if ((uint32_t)(now_ms - arm_runtime.state_tick) >=
                ARM_DM_ENABLE_REFRESH_MS) {
                arm_runtime.state_tick = now_ms;
                for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
                    if (DMMotorEnterMode(arm_joint[axis].motor)) {
                        g_arm_dm_debug.enter_mode_sent[axis] = 1u;
                    }
                }
            }
            if (ArmAllFeedbackValid(now_ms)) {
                if (!ArmSyncAllCurrentTargets()) {
                    break;
                }
                for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
                    DMMotorEnterModeAndHoldOpenLoop(arm_joint[axis].motor);
                }
#if ARM_BOOT_MODE == ARM_BOOT_MODE_DM_SINGLE_AXIS_TEST
                g_arm_state.mode = ARM_MODE_DM_SINGLE_AXIS_TEST;
#else
                g_arm_state.mode = ARM_MODE_DM_ENABLE_ONLY;
#endif
                g_arm_state.active_axis = ARM_AXIS_NONE;
                ArmSetStartState(ARM_START_READY, now_ms);
            }
            break;

        case ARM_START_READY:
            /*
             * 纯使能模式只保持位置；联调模式在此基础上由arm轨迹层接管目标。
             */
            if ((uint32_t)(now_ms - arm_runtime.state_tick) >=
                ARM_DM_ENABLE_REFRESH_MS) {
                arm_runtime.state_tick = now_ms;
                for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
                    DMMotorEnterModeAndHoldOpenLoop(arm_joint[axis].motor);
                }
            }
            break;

        default:
            ArmSetStartState(ARM_START_REGISTERED, now_ms);
            break;
    }
}

static uint8_t ArmSetJointCommandForPose(uint8_t axis,
                                         float target_deg,
                                         const float pose_q_deg[3],
                                         float speed_deg_s,
                                         uint8_t allow_escape)
{
    Arm_Joint_Motor_s *joint;
    Arm_Limit_Result_e target_limit;
    float motor_rad;
    float velocity_rad_s;

    if (axis >= ARM_AXIS_COUNT || !isfinite(target_deg) ||
        !isfinite(speed_deg_s) || speed_deg_s <= 0.0f) {
        return 0u;
    }
    joint = &arm_joint[axis];
    target_limit = ArmCheckLimit(axis, target_deg);
    if (target_limit == ARM_LIMIT_INVALID ||
        target_limit == ARM_LIMIT_OUTSIDE_HARD ||
        (!allow_escape && target_limit != ARM_LIMIT_INSIDE_SOFT)) {
        return 0u;
    }
    motor_rad = ArmJointDegToMotorRadForPose((Arm_Joint_e)axis, target_deg,
                                             pose_q_deg);
    velocity_rad_s = ArmAbs(speed_deg_s * ARM_DEG_TO_RAD /
        joint->direction);
    if (!isfinite(motor_rad) || !isfinite(velocity_rad_s) ||
        !DMMotorSetPositionSpeed(joint->motor, motor_rad,
                                velocity_rad_s)) {
        return 0u;
    }
    joint->target_deg = target_deg;
    g_arm_state.q_target_deg[axis] = target_deg;
    return 1u;
}

static uint8_t ArmCommandPose(const float pose_q_deg[3],
                              float speed_deg_s,
                              uint8_t allow_escape)
{
    uint8_t axis;

    if (pose_q_deg == NULL || !isfinite(speed_deg_s) ||
        speed_deg_s <= 0.0f) {
        return 0u;
    }
    for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
        Arm_Limit_Result_e target_limit;

        if (!isfinite(pose_q_deg[axis])) {
            return 0u;
        }
        target_limit = ArmCheckLimit(axis, pose_q_deg[axis]);
        if (target_limit == ARM_LIMIT_INVALID ||
            target_limit == ARM_LIMIT_OUTSIDE_HARD ||
            (!allow_escape && target_limit != ARM_LIMIT_INSIDE_SOFT)) {
            return 0u;
        }
    }
    for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
        if (!ArmSetJointCommandForPose(axis, pose_q_deg[axis], pose_q_deg,
                                       speed_deg_s, allow_escape)) {
            return 0u;
        }
    }
    return 1u;
}

static uint8_t ArmCommandSingleAxis(uint8_t active_axis,
                                    float target_deg,
                                    float speed_deg_s,
                                    uint8_t allow_escape)
{
    uint8_t axis;
    float pose_q_deg[3];

    if (arm_runtime.elbow_coupling_active == 0u) {
        for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
            if (axis == active_axis) {
                continue;
            }
            if (!DMMotorSetPositionSpeed(arm_joint[axis].motor,
                    ArmJointDegToMotorRadBase((Arm_Joint_e)axis,
                        arm_joint[axis].feedback_deg),
                    ARM_ESCAPE_SPEED_DEG_S * ARM_DEG_TO_RAD)) {
                return 0u;
            }
            arm_joint[axis].target_deg = arm_joint[axis].feedback_deg;
            g_arm_state.q_target_deg[axis] = arm_joint[axis].feedback_deg;
        }
        return ArmSetJointCommandForPose(active_axis, target_deg, NULL,
                                         speed_deg_s, allow_escape);
    }
    for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
        pose_q_deg[axis] = arm_joint[axis].feedback_deg;
    }
    pose_q_deg[active_axis] = target_deg;
    return ArmCommandPose(pose_q_deg, speed_deg_s, allow_escape);
}

static uint8_t ArmAxisArrived(uint8_t axis, uint32_t now_ms)
{
    float error_deg;
    float speed_deg_s;

    error_deg = arm_runtime.active_target_deg - arm_joint[axis].feedback_deg;
    speed_deg_s = ArmMotorVelocityToJointDegS(
        (Arm_Joint_e)axis, arm_joint[axis].motor->measure.velocity_rad_s);
    if (ArmAbs(error_deg) <= ARM_ARRIVAL_ERROR_DEG &&
        ArmAbs(speed_deg_s) <= ARM_ARRIVAL_SPEED_DEG_S) {
        if (arm_runtime.stable_tick == 0u) {
            arm_runtime.stable_tick = now_ms;
        }
        return (uint32_t)(now_ms - arm_runtime.stable_tick) >=
            ARM_ARRIVAL_STABLE_MS;
    }
    arm_runtime.stable_tick = 0u;
    return 0u;
}

static uint8_t ArmAxisMovingWrongWay(uint8_t axis, uint32_t now_ms)
{
    float expected_direction = arm_runtime.active_target_deg >=
        arm_runtime.direction_start_deg ? 1.0f : -1.0f;
    float moved_deg = arm_joint[axis].feedback_deg -
        arm_runtime.direction_start_deg;

    if (moved_deg * expected_direction >= -ARM_WRONG_DIRECTION_DELTA_DEG) {
        arm_runtime.direction_tick = 0u;
        arm_runtime.direction_start_deg = arm_joint[axis].feedback_deg;
        return 0u;
    }
    if (arm_runtime.direction_tick == 0u) {
        arm_runtime.direction_tick = now_ms;
        return 0u;
    }
    return (uint32_t)(now_ms - arm_runtime.direction_tick) >=
        ARM_WRONG_DIRECTION_TIME_MS;
}

static uint8_t ArmStartAxisMotion(uint8_t axis,
                                  float target_deg,
                                  float speed_deg_s,
                                  uint32_t now_ms,
                                  uint8_t allow_escape)
{
    arm_runtime.active_target_deg = target_deg;
    arm_runtime.active_speed_deg_s = speed_deg_s;
    arm_runtime.direction_start_deg = arm_joint[axis].feedback_deg;
    arm_runtime.direction_tick = 0u;
    arm_runtime.stable_tick = 0u;
    g_arm_state.active_axis = axis;
    arm_runtime.state_tick = now_ms;
    return ArmCommandSingleAxis(axis, target_deg, speed_deg_s,
                                allow_escape);
}

static uint8_t ArmRunAxisMotion(uint8_t axis,
                                uint32_t now_ms,
                                uint32_t timeout_ms,
                                uint8_t allow_escape)
{
    float current_deg = arm_joint[axis].feedback_deg;
    float remaining = arm_runtime.active_target_deg - current_deg;

    if (!ArmAllFeedbackValid(now_ms)) {
        ArmLatchFault(ARM_FAULT_FEEDBACK_TIMEOUT);
        return 0u;
    }
    if (ArmAnyMotorStateFault()) {
        ArmLatchFault(ARM_FAULT_MOTOR_STATE);
        return 0u;
    }
    if (!isfinite(current_deg) ||
        ArmCheckLimit(axis, current_deg) == ARM_LIMIT_OUTSIDE_HARD ||
        (allow_escape && remaining != 0.0f &&
         ((current_deg < arm_joint[axis].soft_min_deg && remaining < 0.0f) ||
          (current_deg > arm_joint[axis].soft_max_deg && remaining > 0.0f)))) {
        ArmLatchFault(ARM_FAULT_HARD_BOUNDARY);
        return 0u;
    }
    if (ArmAxisMovingWrongWay(axis, now_ms)) {
        ArmLatchFault(ARM_FAULT_ESCAPE_DIRECTION);
        return 0u;
    }
    if ((uint32_t)(now_ms - arm_runtime.state_tick) >= timeout_ms) {
        ArmLatchFault(allow_escape ? ARM_FAULT_ESCAPE_TIMEOUT :
                                      ARM_FAULT_RETURN_TIMEOUT);
        return 0u;
    }
    if (!ArmCommandSingleAxis(axis, arm_runtime.active_target_deg,
                              arm_runtime.active_speed_deg_s,
                              allow_escape)) {
        ArmLatchFault(ARM_FAULT_CAN_TX);
        return 0u;
    }
    return ArmAxisArrived(axis, now_ms);
}

static uint8_t ArmAutoInitArmAxis(uint8_t pair_index)
{
    return pair_index == 0u ? ARM_JOINT_SHOULDER : ARM_JOINT_ELBOW;
}

static uint8_t ArmAutoInitArmAxisArrived(uint8_t pair_index,
                                         float target_deg,
                                         uint32_t now_ms)
{
    uint8_t axis = ArmAutoInitArmAxis(pair_index);
    float error_deg = target_deg - arm_joint[axis].feedback_deg;
    float speed_deg_s = ArmMotorVelocityToJointDegS(
        (Arm_Joint_e)axis, arm_joint[axis].motor->measure.velocity_rad_s);

    if (ArmAbs(error_deg) <= ARM_ARRIVAL_ERROR_DEG &&
        ArmAbs(speed_deg_s) <= ARM_ARRIVAL_SPEED_DEG_S) {
        if (arm_runtime.auto_init_arm_stable_tick[pair_index] == 0u) {
            arm_runtime.auto_init_arm_stable_tick[pair_index] = now_ms;
        }
        return (uint32_t)(now_ms -
            arm_runtime.auto_init_arm_stable_tick[pair_index]) >=
            ARM_ARRIVAL_STABLE_MS;
    }
    arm_runtime.auto_init_arm_stable_tick[pair_index] = 0u;
    return 0u;
}

static uint8_t ArmAutoInitArmAxisMovingWrongWay(uint8_t pair_index,
                                                float target_deg,
                                                uint32_t now_ms)
{
    uint8_t axis = ArmAutoInitArmAxis(pair_index);
    float expected_direction = target_deg >=
        arm_runtime.auto_init_arm_direction_start_deg[pair_index] ?
        1.0f : -1.0f;
    float moved_deg = arm_joint[axis].feedback_deg -
        arm_runtime.auto_init_arm_direction_start_deg[pair_index];

    if (moved_deg * expected_direction >= -ARM_WRONG_DIRECTION_DELTA_DEG) {
        arm_runtime.auto_init_arm_direction_tick[pair_index] = 0u;
        arm_runtime.auto_init_arm_direction_start_deg[pair_index] =
            arm_joint[axis].feedback_deg;
        return 0u;
    }
    if (arm_runtime.auto_init_arm_direction_tick[pair_index] == 0u) {
        arm_runtime.auto_init_arm_direction_tick[pair_index] = now_ms;
        return 0u;
    }
    return (uint32_t)(now_ms -
        arm_runtime.auto_init_arm_direction_tick[pair_index]) >=
        ARM_WRONG_DIRECTION_TIME_MS;
}

static uint8_t ArmStartAutoInitArmMotion(const float target_q_deg[3],
                                         float speed_deg_s)
{
    float pose_q_deg[3];
    uint8_t pair_index;

    if (target_q_deg == NULL || !isfinite(speed_deg_s) ||
        speed_deg_s <= 0.0f) {
        return 0u;
    }
    arm_runtime.auto_init_hold_base_deg =
        arm_joint[ARM_JOINT_BASE_YAW].feedback_deg;
    pose_q_deg[ARM_JOINT_BASE_YAW] = arm_runtime.auto_init_hold_base_deg;
    pose_q_deg[ARM_JOINT_SHOULDER] = target_q_deg[ARM_JOINT_SHOULDER];
    pose_q_deg[ARM_JOINT_ELBOW] = target_q_deg[ARM_JOINT_ELBOW];
    for (pair_index = 0u;
         pair_index < ARM_AUTO_INIT_ARM_AXIS_COUNT;
         ++pair_index) {
        uint8_t axis = ArmAutoInitArmAxis(pair_index);

        arm_runtime.auto_init_arm_stable_tick[pair_index] = 0u;
        arm_runtime.auto_init_arm_direction_tick[pair_index] = 0u;
        arm_runtime.auto_init_arm_direction_start_deg[pair_index] =
            arm_joint[axis].feedback_deg;
    }
    g_arm_state.active_axis = ARM_AXIS_NONE;
    return ArmCommandPose(pose_q_deg, speed_deg_s, 1u);
}

static uint8_t ArmRunAutoInitArmMotion(const float target_q_deg[3],
                                       float speed_deg_s,
                                       uint32_t now_ms,
                                       uint32_t timeout_ms)
{
    float pose_q_deg[3];
    uint8_t pair_index;
    uint8_t all_arrived = 1u;

    if (target_q_deg == NULL || !ArmAllFeedbackValid(now_ms)) {
        ArmLatchFault(ARM_FAULT_FEEDBACK_TIMEOUT);
        return 0u;
    }
    if (ArmAnyMotorStateFault()) {
        ArmLatchFault(ARM_FAULT_MOTOR_STATE);
        return 0u;
    }
    for (pair_index = 0u;
         pair_index < ARM_AUTO_INIT_ARM_AXIS_COUNT;
         ++pair_index) {
        uint8_t axis = ArmAutoInitArmAxis(pair_index);
        float current_deg = arm_joint[axis].feedback_deg;
        float remaining = target_q_deg[axis] - current_deg;

        if (!isfinite(current_deg) ||
            ArmCheckLimit(axis, current_deg) == ARM_LIMIT_OUTSIDE_HARD ||
            (remaining != 0.0f &&
             ((current_deg < arm_joint[axis].soft_min_deg &&
               remaining < 0.0f) ||
              (current_deg > arm_joint[axis].soft_max_deg &&
               remaining > 0.0f)))) {
            ArmLatchFault(ARM_FAULT_HARD_BOUNDARY);
            return 0u;
        }
        if (ArmAutoInitArmAxisMovingWrongWay(
                pair_index, target_q_deg[axis], now_ms)) {
            ArmLatchFault(ARM_FAULT_ESCAPE_DIRECTION);
            return 0u;
        }
    }
    if ((uint32_t)(now_ms - arm_runtime.auto_init_tick) >= timeout_ms) {
        ArmLatchFault(ARM_FAULT_ESCAPE_TIMEOUT);
        return 0u;
    }
    pose_q_deg[ARM_JOINT_BASE_YAW] = arm_runtime.auto_init_hold_base_deg;
    pose_q_deg[ARM_JOINT_SHOULDER] = target_q_deg[ARM_JOINT_SHOULDER];
    pose_q_deg[ARM_JOINT_ELBOW] = target_q_deg[ARM_JOINT_ELBOW];
    if (!ArmCommandPose(pose_q_deg, speed_deg_s, 1u)) {
        ArmLatchFault(ARM_FAULT_CAN_TX);
        return 0u;
    }
    for (pair_index = 0u;
         pair_index < ARM_AUTO_INIT_ARM_AXIS_COUNT;
         ++pair_index) {
        uint8_t axis = ArmAutoInitArmAxis(pair_index);

        if (!ArmAutoInitArmAxisArrived(
                pair_index, target_q_deg[axis], now_ms)) {
            all_arrived = 0u;
        }
    }
    return all_arrived;
}

static uint8_t ArmFindEscapeAxis(uint8_t *axis_out, float *target_out)
{
    static const uint8_t escape_order[3] = {
        ARM_JOINT_ELBOW, ARM_JOINT_SHOULDER, ARM_JOINT_BASE_YAW
    };
    uint8_t index;

    for (index = 0u; index < 3u; ++index) {
        uint8_t axis = escape_order[index];
        float q_deg = arm_joint[axis].feedback_deg;
        Arm_Limit_Result_e limit = ArmCheckLimit(axis, q_deg);
        if (limit == ARM_LIMIT_OUTSIDE_HARD || limit == ARM_LIMIT_INVALID) {
            return 0u;
        }
        if (limit == ARM_LIMIT_ESCAPE_ALLOWED) {
            *axis_out = axis;
            *target_out = q_deg < arm_joint[axis].soft_min_deg ?
                arm_joint[axis].soft_min_deg :
                arm_joint[axis].soft_max_deg;
            return 1u;
        }
    }
    *axis_out = ARM_AXIS_NONE;
    return 1u;
}

static Arm_Start_State_e ArmEscapeStateForAxis(uint8_t axis)
{
    if (axis == ARM_JOINT_ELBOW) {
        return ARM_START_ESCAPE_ELBOW;
    }
    if (axis == ARM_JOINT_SHOULDER) {
        return ARM_START_ESCAPE_SHOULDER;
    }
    return ARM_START_ESCAPE_BASE;
}

static void ArmBeginReturnSequence(uint32_t now_ms)
{
    ArmSetStartState(ARM_START_RETURN_ELBOW, now_ms);
    if (!ArmStartAxisMotion(ARM_JOINT_ELBOW, ARM_SAFE_Q3_DEG,
                            ARM_RETURN_SPEED_DEG_S, now_ms, 0u)) {
        ArmLatchFault(ARM_FAULT_CAN_TX);
    }
}

static void ArmProcessFaultReset(uint32_t now_ms)
{
    uint32_t request = g_arm_state.fault_reset_applied;
    uint32_t state_request = g_arm_state.fault_reset_request;
    uint32_t debug_request = g_arm_dm_debug.fault_reset_request;
    uint8_t axis;
    Arm_Fault_Reset_Result_e result = ARM_FAULT_RESET_OK;

    if (debug_request != g_arm_state.fault_reset_applied) {
        request = debug_request;
    }
    if (state_request != g_arm_state.fault_reset_applied) {
        request = state_request;
    }
    if (request == g_arm_state.fault_reset_applied) {
        return;
    }
    g_arm_state.fault_reset_request = request;
    g_arm_dm_debug.fault_reset_request = request;
    if (g_arm_state.fault_latched == ARM_FAULT_NONE) {
        result = ARM_FAULT_RESET_NOT_FAULTED;
    } else if (!ArmAllFeedbackValid(now_ms)) {
        result = ARM_FAULT_RESET_FEEDBACK_INVALID;
    } else if (ArmTemperatureAtOrAbove(ARM_TEMPERATURE_HOLD_C)) {
        result = ARM_FAULT_RESET_TEMPERATURE_HIGH;
    } else if (ArmAnyMotorStateFault()) {
        result = ARM_FAULT_RESET_MOTOR_STATE_ACTIVE;
    } else {
        for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
            Arm_Limit_Result_e limit = ArmCheckLimit(
                axis, arm_joint[axis].feedback_deg);
            if (limit == ARM_LIMIT_INVALID ||
                limit == ARM_LIMIT_OUTSIDE_HARD) {
                result = ARM_FAULT_RESET_OUTSIDE_HARD_BOUNDARY;
                break;
            }
        }
    }
    g_arm_state.fault_reset_result = result;
    g_arm_state.fault_reset_applied = request;
    g_arm_dm_debug.fault_reset_applied = request;
    g_arm_dm_debug.fault_reset_result = result;
    if (result != ARM_FAULT_RESET_OK) {
        return;
    }

    ArmDisableAll();
    arm_runtime.elbow_coupling_active = 0u;
    for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
        if (!DMMotorClearFault(arm_joint[axis].motor)) {
            g_arm_state.fault_reset_result =
                ARM_FAULT_RESET_CAN_UNAVAILABLE;
            g_arm_dm_debug.fault_reset_result =
                ARM_FAULT_RESET_CAN_UNAVAILABLE;
            return;
        }
        DMMotorResetSoftwareFault(arm_joint[axis].motor);
    }
    g_arm_state.fault_latched = ARM_FAULT_NONE;
    arm_runtime.resetting = 1u;
    arm_runtime.disable_sent = 0u;
    ArmSetStartState(ARM_START_WAIT_PASSIVE_FEEDBACK, now_ms);
    g_arm_state.mode = ARM_MODE_SAFE;
}

static void ArmUpdateFeedback(uint32_t now_ms)
{
    uint8_t axis;

    g_arm_dm_debug.elbow_coupling_active =
        arm_runtime.elbow_coupling_active;
    for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
        Arm_Joint_Motor_s *joint = &arm_joint[axis];
        DM_MotorInstance *motor = joint->motor;
        Arm_DM_Axis_Debug_s *debug = &g_arm_dm_debug.axis[axis];
        float velocity_deg_s = 0.0f;
        Arm_Limit_Result_e limit = ARM_LIMIT_INVALID;

        if (motor != NULL) {
            joint->feedback_deg = ArmMotorRadToJointDeg(
                (Arm_Joint_e)axis, motor->measure.position_rad);
            velocity_deg_s = ArmMotorVelocityToJointDegS(
                (Arm_Joint_e)axis, motor->measure.velocity_rad_s);
            limit = ArmCheckLimit(axis, joint->feedback_deg);
            g_arm_state.motor_online[axis] = DMMotorIsOnline(motor, now_ms);
            g_arm_state.motor_enabled[axis] = motor->control_enabled;
            g_arm_state.target_synced[axis] = motor->target_synced;
            g_arm_state.soft_limit_ok[axis] =
                limit == ARM_LIMIT_INSIDE_SOFT;
            g_arm_state.hard_boundary_ok[axis] =
                limit == ARM_LIMIT_INSIDE_SOFT ||
                limit == ARM_LIMIT_ESCAPE_ALLOWED;
            g_arm_state.motor_state[axis] = motor->measure.state;
            g_arm_state.motor_position_rad[axis] =
                motor->measure.position_rad;
            g_arm_state.motor_velocity_rad_s[axis] =
                motor->measure.velocity_rad_s;
            g_arm_state.motor_speed_dps[axis] = velocity_deg_s;
            g_arm_state.motor_torque_nm[axis] = motor->measure.torque_nm;
            g_arm_state.motor_current[axis] = motor->measure.torque_nm;
            g_arm_state.motor_command_rad[axis] = motor->position_ref_rad;
            g_arm_state.motor_velocity_limit_rad_s[axis] =
                motor->velocity_limit_rad_s;
            g_arm_state.mos_temperature_c[axis] =
                motor->measure.mos_temperature_c;
            g_arm_state.rotor_temperature_c[axis] =
                motor->measure.rotor_temperature_c;

            debug->motor_id = motor->motor_id;
            debug->master_id = motor->master_id;
            debug->command_id = motor->command_id;
            debug->raw_position_rad = motor->measure.position_rad;
            debug->logical_joint_deg = joint->feedback_deg;
            debug->logical_target_deg = joint->target_deg;
            debug->command_position_rad = motor->position_ref_rad;
            debug->velocity_limit_rad_s = motor->velocity_limit_rad_s;
            debug->velocity_limit_deg_s =
                motor->velocity_limit_rad_s * ARM_RAD_TO_DEG;
            debug->velocity_rad_s = motor->measure.velocity_rad_s;
            debug->velocity_deg_s = velocity_deg_s;
            debug->torque_nm = motor->measure.torque_nm;
            debug->mos_temperature_c = motor->measure.mos_temperature_c;
            debug->rotor_temperature_c =
                motor->measure.rotor_temperature_c;
            debug->state = motor->measure.state;
            debug->feedback_valid = motor->measure.feedback_valid;
            debug->online = g_arm_state.motor_online[axis];
            debug->target_synced = motor->target_synced;
            debug->enabled = motor->control_enabled;
            debug->mode_request_pending = motor->mode_request_pending;
            debug->mode_confirmed = DMMotorModeConfirmed(motor);
            debug->limit_result = limit;
            debug->hard_boundary_ok = g_arm_state.hard_boundary_ok[axis];
            debug->rx_count = motor->measure.rx_count;
            {
                int32_t feedback_age = (int32_t)(now_ms -
                    motor->measure.last_feedback_tick);
                debug->feedback_age_ms = feedback_age < 0 ? 0u :
                    (uint32_t)feedback_age;
            }
            debug->tx_count = motor->tx_count;
            debug->tx_fail_count = motor->tx_fail_count;
            debug->mode_command_count = motor->mode_command_count;
            debug->last_mode_tx_id = motor->last_mode_tx_id;
        }
        g_arm_state.q_feedback_deg[axis] = joint->feedback_deg;
        g_arm_state.q_target_deg[axis] = joint->target_deg;
    }
    /* 转成台架上直观的物理角：HOME时应分别接近-90deg和60deg。 */
    g_arm_home_joint_debug.shoulder_current_deg =
        -g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER];
    g_arm_home_joint_debug.shoulder_target_deg =
        -g_arm_state.q_target_deg[ARM_JOINT_SHOULDER];
    g_arm_home_joint_debug.elbow_included_current_deg =
        -g_arm_state.q_feedback_deg[ARM_JOINT_ELBOW];
    g_arm_home_joint_debug.elbow_included_target_deg =
        -g_arm_state.q_target_deg[ARM_JOINT_ELBOW];
    g_arm_state.all_targets_synced = ArmAllTargetsSynced();
    ArmForwardKinematics3DOF(g_arm_state.q_feedback_deg[0],
                             g_arm_state.q_feedback_deg[1],
                             g_arm_state.q_feedback_deg[2],
                             &g_arm_state.wrist_center);
    g_arm_state.small_link_pitch_deg = g_arm_state.q_feedback_deg[1] +
        (-180.0f - g_arm_state.q_feedback_deg[2]);
    g_arm_state.end_pitch_deg = g_arm_state.small_link_pitch_deg;
    g_arm_state.state_elapsed_ms = now_ms - arm_runtime.state_tick;

    g_arm_dm_debug.mode = g_arm_state.mode;
    g_arm_dm_debug.start_state = g_arm_state.start_state;
    g_arm_dm_debug.fault = g_arm_state.fault_latched;
    g_arm_dm_debug.power_on_delay_elapsed_ms =
        (uint32_t)(now_ms - arm_runtime.boot_tick);
    if (g_arm_dm_debug.power_on_delay_elapsed_ms >
        ARM_DM_POWER_ON_DELAY_MS) {
        g_arm_dm_debug.power_on_delay_elapsed_ms =
            ARM_DM_POWER_ON_DELAY_MS;
    }
    g_arm_dm_debug.power_on_delay_active =
        g_arm_state.start_state == ARM_START_REGISTERED &&
        g_arm_dm_debug.power_on_delay_elapsed_ms <
            ARM_DM_POWER_ON_DELAY_MS;
    g_arm_dm_debug.power_on_delay_done =
        g_arm_dm_debug.power_on_delay_elapsed_ms >=
            ARM_DM_POWER_ON_DELAY_MS;
    g_arm_dm_debug.fault_reset_request = g_arm_state.fault_reset_request;
    g_arm_dm_debug.fault_reset_applied = g_arm_state.fault_reset_applied;
    g_arm_dm_debug.fault_reset_result = g_arm_state.fault_reset_result;
}

static void ArmUpdateTeachAndKinematicsDebug(void)
{
    const Arm_Tool_State_s *tool = ArmToolGetState();
    uint8_t axis;
    uint32_t update_count = g_arm_teach_point.update_count + 1u;

    memset(&g_arm_teach_point, 0, sizeof(g_arm_teach_point));
    g_arm_teach_point.ready = ArmAllFeedbackValid(HAL_GetTick());
    g_arm_teach_point.point_type = ARM_CONTROL_POINT_WRIST_CENTER;
    g_arm_teach_point.kinematics_valid = g_arm_state.kinematics_valid;
    for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
        g_arm_teach_point.motor_online[axis] =
            g_arm_state.motor_online[axis];
        g_arm_teach_point.motor_enabled[axis] =
            g_arm_state.motor_enabled[axis];
        g_arm_teach_point.q_deg[axis] = g_arm_state.q_feedback_deg[axis];
        g_arm_teach_point.motor_position_rad[axis] =
            g_arm_state.motor_position_rad[axis];
    }
    g_arm_teach_point.wrist_center_mm = g_arm_state.wrist_center;
    g_arm_teach_point.point_type = ARM_CONTROL_POINT_TOOL_CENTER;
    g_arm_teach_point.tool_tip_mm = g_arm_state.tool_tip;
    g_arm_teach_point.small_link_pitch_deg =
        g_arm_state.small_link_pitch_deg;
    g_arm_teach_point.tool_ready = tool->tool_ready;
    memcpy(g_arm_teach_point.tool_servo_target_pos,
           tool->servo_target_pos,
           sizeof(g_arm_teach_point.tool_servo_target_pos));
    g_arm_teach_point.tool_error_code = tool->error_code;
    g_arm_teach_point.update_count = update_count;

    memset(&g_arm_kinematics_debug, 0, sizeof(g_arm_kinematics_debug));
    g_arm_kinematics_debug.kinematics_valid = g_arm_state.kinematics_valid;
    memcpy(g_arm_kinematics_debug.motor_online, g_arm_state.motor_online,
           sizeof(g_arm_kinematics_debug.motor_online));
    memcpy(g_arm_kinematics_debug.motor_enabled, g_arm_state.motor_enabled,
           sizeof(g_arm_kinematics_debug.motor_enabled));
    memcpy(g_arm_kinematics_debug.q_feedback_deg,
           g_arm_state.q_feedback_deg,
           sizeof(g_arm_kinematics_debug.q_feedback_deg));
    memcpy(g_arm_kinematics_debug.q_target_deg, g_arm_state.q_target_deg,
           sizeof(g_arm_kinematics_debug.q_target_deg));
    g_arm_kinematics_debug.wrist_center_mm = g_arm_state.wrist_center;
    g_arm_kinematics_debug.tool_center_mm = g_arm_state.tool_tip;
    g_arm_kinematics_debug.tool_axis_to_center_mm =
        ARM_TOOL_PITCH_AXIS_TO_CENTER_MM;
    g_arm_kinematics_debug.tool_pitch_feedback_deg =
        tool->tool_pitch_feedback_deg;
    g_arm_kinematics_debug.horizontal_radius_mm = sqrtf(
        g_arm_state.wrist_center.x_mm * g_arm_state.wrist_center.x_mm +
        g_arm_state.wrist_center.y_mm * g_arm_state.wrist_center.y_mm);
    g_arm_kinematics_debug.planar_reach_from_shoulder_mm =
        g_arm_kinematics_debug.horizontal_radius_mm;
    g_arm_kinematics_debug.wrist_height_from_shoulder_mm =
        g_arm_state.wrist_center.z_mm - ARM_BASE_HEIGHT_MM;
    g_arm_kinematics_debug.small_link_pitch_deg =
        g_arm_state.small_link_pitch_deg;
    g_arm_kinematics_debug.base_height_mm = ARM_BASE_HEIGHT_MM;
    g_arm_kinematics_debug.link_1_mm = ARM_LINK_1_MM;
    g_arm_kinematics_debug.link_2_mm = ARM_LINK_2_MM;
    g_arm_kinematics_debug.shoulder_offset_forward_mm =
        ARM_SHOULDER_OFFSET_FORWARD_MM;
    g_arm_kinematics_debug.shoulder_offset_left_mm =
        ARM_SHOULDER_OFFSET_LEFT_MM;
    g_arm_kinematics_debug.soft_limit_deg[0][0] = ARM_Q1_SOFT_MIN_DEG;
    g_arm_kinematics_debug.soft_limit_deg[0][1] = ARM_Q1_SOFT_MAX_DEG;
    g_arm_kinematics_debug.soft_limit_deg[1][0] = ARM_Q2_SOFT_MIN_DEG;
    g_arm_kinematics_debug.soft_limit_deg[1][1] = ARM_Q2_SOFT_MAX_DEG;
    g_arm_kinematics_debug.soft_limit_deg[2][0] = ARM_Q3_SOFT_MIN_DEG;
    g_arm_kinematics_debug.soft_limit_deg[2][1] = ARM_Q3_SOFT_MAX_DEG;
    g_arm_kinematics_debug.reference_q_deg[0] = ARM_SAFE_Q1_DEG;
    g_arm_kinematics_debug.reference_q_deg[1] = ARM_SAFE_Q2_DEG;
    g_arm_kinematics_debug.reference_q_deg[2] = ARM_SAFE_Q3_DEG;
    ArmForwardKinematics3DOF(ARM_SAFE_Q1_DEG, ARM_SAFE_Q2_DEG,
                             ARM_SAFE_Q3_DEG,
                             &g_arm_kinematics_debug.reference_wrist_center_mm);
}

static void ArmProcessAutoInit(uint32_t now_ms)
{
    Arm_DM_Auto_Init_Debug_s *init = &g_arm_dm_debug.auto_init;
    float seed_q_deg[3];
    uint8_t axis;

    if (init->enable == 0u) {
        if (init->state != ARM_DM_AUTO_INIT_IDLE) {
            ArmSyncAllCurrentTargets();
        }
        init->state = ARM_DM_AUTO_INIT_IDLE;
        init->axis = ARM_DM_TEST_NONE;
        init->result = ARM_COMMAND_NOT_READY;
        arm_runtime.auto_init_initialized = 0u;
        return;
    }

    if (arm_runtime.auto_init_initialized == 0u) {
        arm_runtime.auto_init_initialized = 1u;
        arm_runtime.auto_init_tick = now_ms;
        init->state = ARM_DM_AUTO_INIT_WAIT_READY;
        init->axis = ARM_DM_TEST_NONE;
        init->step = 0u;
        init->done = 0u;
        init->result = ARM_COMMAND_NOT_READY;
        init->start_deg = NAN;
        init->target_deg = NAN;
        init->elapsed_ms = 0u;
        init->cycle_count++;
        return;
    }

    if (!ArmAllFeedbackValid(HAL_GetTick()) ||
        !ArmAllTargetsSynced() ||
        !ArmAllMotorsEnabled()) {
        init->state = ARM_DM_AUTO_INIT_FAULT;
        init->result = ARM_COMMAND_NOT_READY;
        return;
    }

    if (!isfinite(init->speed_deg_s) || init->speed_deg_s <= 0.0f) {
        init->state = ARM_DM_AUTO_INIT_FAULT;
        init->result = ARM_COMMAND_INVALID;
        return;
    }

    init->elapsed_ms = now_ms - arm_runtime.auto_init_tick;
    switch (init->state) {
        case ARM_DM_AUTO_INIT_WAIT_READY:
            init->result = ARM_COMMAND_BUSY;
            if ((uint32_t)(now_ms - arm_runtime.auto_init_tick) <
                ARM_DM_AUTO_INIT_START_DELAY_MS) {
                break;
            }
            init->step = 0u;
            init->state = ARM_DM_AUTO_INIT_MOVE_AXIS;
            break;

        case ARM_DM_AUTO_INIT_MOVE_AXIS:
            seed_q_deg[0] = arm_joint[ARM_JOINT_BASE_YAW].feedback_deg;
            seed_q_deg[1] = arm_joint[ARM_JOINT_SHOULDER].feedback_deg;
            seed_q_deg[2] = arm_joint[ARM_JOINT_ELBOW].feedback_deg;
            /*
             * q1=0必须来自底座朝向+X后保存的达妙零点。若上电反馈已在
             * 正常q1范围外，禁止用一次大角度自动运动掩盖错误零点。
             */
            if (ArmCheckLimit(ARM_JOINT_BASE_YAW, seed_q_deg[0]) !=
                ARM_LIMIT_INSIDE_SOFT) {
                init->state = ARM_DM_AUTO_INIT_FAULT;
                init->result = ARM_COMMAND_PREFLIGHT_FAILED;
                break;
            }
            if (!ArmResolveHomePose(seed_q_deg, init->target_q_deg,
                                    &init->target_wrist_mm,
                                    &init->ik_status)) {
                init->state = ARM_DM_AUTO_INIT_FAULT;
                init->result = ARM_COMMAND_PREFLIGHT_FAILED;
                break;
            }
            init->axis = ARM_DM_TEST_SHOULDER_ELBOW;
            init->step = 1u;
            /* 兼容单值Watch字段，以大臂代表当前大臂/小臂联合阶段。 */
            init->start_deg =
                arm_joint[ARM_JOINT_SHOULDER].feedback_deg;
            init->target_deg = init->target_q_deg[ARM_JOINT_SHOULDER];
            if (!ArmStartAutoInitArmMotion(init->target_q_deg,
                                           init->speed_deg_s)) {
                init->state = ARM_DM_AUTO_INIT_FAULT;
                init->result = ARM_COMMAND_PREFLIGHT_FAILED;
                break;
            }
            arm_runtime.auto_init_tick = now_ms;
            init->elapsed_ms = 0u;
            init->result = ARM_COMMAND_OK;
            init->state = ARM_DM_AUTO_INIT_WAIT_AXIS;
            break;

        case ARM_DM_AUTO_INIT_WAIT_AXIS:
            if (ArmRunAutoInitArmMotion(
                    init->target_q_deg, init->speed_deg_s, now_ms,
                    ARM_DM_AUTO_INIT_STEP_TIMEOUT_MS)) {
                init->step = 2u;
                init->state = ARM_DM_AUTO_INIT_MOVE_SAFE;
            } else if (g_arm_state.fault_latched != ARM_FAULT_NONE) {
                init->state = ARM_DM_AUTO_INIT_FAULT;
                init->result = ARM_COMMAND_NOT_READY;
            }
            break;

        case ARM_DM_AUTO_INIT_MOVE_SAFE:
            axis = ARM_JOINT_BASE_YAW;
            if (init->step != 2u) {
                init->state = ARM_DM_AUTO_INIT_FAULT;
                init->result = ARM_COMMAND_PREFLIGHT_FAILED;
                break;
            }
            init->axis = ARM_DM_TEST_BASE;
            init->start_deg = arm_joint[axis].feedback_deg;
            init->target_deg = init->target_q_deg[axis];
            if (!ArmStartAxisMotion(axis, init->target_q_deg[axis],
                                    init->speed_deg_s, now_ms, 1u)) {
                init->state = ARM_DM_AUTO_INIT_FAULT;
                init->result = ARM_COMMAND_PREFLIGHT_FAILED;
                break;
            }
            arm_runtime.auto_init_tick = now_ms;
            init->elapsed_ms = 0u;
            init->result = ARM_COMMAND_OK;
            init->state = ARM_DM_AUTO_INIT_WAIT_SAFE;
            break;

        case ARM_DM_AUTO_INIT_WAIT_SAFE:
            axis = ARM_JOINT_BASE_YAW;
            if (ArmRunAxisMotion(axis, now_ms,
                                 ARM_DM_AUTO_INIT_STEP_TIMEOUT_MS, 1u)) {
                g_arm_state.active_axis = ARM_AXIS_NONE;
                init->axis = ARM_DM_TEST_NONE;
                init->done = 1u;
                init->result = ARM_COMMAND_OK;
                init->state = ARM_DM_AUTO_INIT_DONE;
            } else if (g_arm_state.fault_latched != ARM_FAULT_NONE) {
                init->state = ARM_DM_AUTO_INIT_FAULT;
                init->result = ARM_COMMAND_NOT_READY;
            }
            break;

        case ARM_DM_AUTO_INIT_DONE:
            /* 保留HOME精确目标，不在到位容差内改写为当时反馈角。 */
            init->axis = ARM_DM_TEST_NONE;
            init->done = 1u;
            init->result = ARM_COMMAND_OK;
            break;

        case ARM_DM_AUTO_INIT_FAULT:
            ArmTrajectoryCancel();
            ArmSyncAllCurrentTargets();
            break;

        case ARM_DM_AUTO_INIT_IDLE:
        default:
            arm_runtime.auto_init_initialized = 0u;
            break;
    }
}

static void ArmSetBootState(Arm_Boot_State_e state, uint32_t now_ms)
{
    g_arm_boot_debug.state = state;
    g_arm_boot_debug.state_tick = now_ms;
    g_arm_boot_debug.elapsed_ms = 0u;
}

static void ArmProcessBootSequence(uint32_t now_ms)
{
    g_arm_boot_debug.elapsed_ms =
        (uint32_t)(now_ms - g_arm_boot_debug.state_tick);

    switch (g_arm_boot_debug.state) {
        case ARM_BOOT_WAIT_MOTORS:
            ArmProcessEnableOnly(now_ms);
            if (g_arm_state.start_state == ARM_START_READY) {
                /*
                 * 三电机已使能并同步当前位置后，立即启用小臂同步带补偿。
                 * 必须在第一次自动初始化命令之前开启：大臂从机械零位
                 * q2=180deg转向前方时，小臂电机需要同步反向运动，才能
                 * 让两杆物理夹角在初始化过程中持续保持目标q3。
                 */
                arm_runtime.elbow_coupling_active =
                    ARM_ELBOW_SHOULDER_COUPLING_ENABLE != 0u ? 1u : 0u;
                /* ID1/ID2尚未初始化；大臂和小臂先同步HOME。 */
                ArmSetBootState(ARM_BOOT_AUTO_INIT, now_ms);
            }
            break;

        case ARM_BOOT_AUTO_INIT:
            ArmTrajectoryTask(now_ms);
            if (g_arm_dm_debug.auto_init.state != ARM_DM_AUTO_INIT_DONE) {
                ArmProcessAutoInit(now_ms);
            }
            if (g_arm_dm_debug.auto_init.state == ARM_DM_AUTO_INIT_FAULT) {
                g_arm_boot_debug.motion_result =
                    ARM_MOTION_RESULT_PREFLIGHT_FAILED;
                ArmSetBootState(ARM_BOOT_FAULT, now_ms);
            } else if (g_arm_dm_debug.auto_init.state ==
                       ARM_DM_AUTO_INIT_DONE) {
                arm_runtime.elbow_coupling_active = 1u;
                /* q2/q3同步HOME且q1随后HOME后，才初始化ID1/ID2。 */
                ArmToolInit();
                ArmSetBootState(ARM_BOOT_WAIT_TOOL, now_ms);
            }
            break;

        case ARM_BOOT_WAIT_TOOL:
            if (ArmToolReadyForMotion() && ArmToolTxIdle()) {
                ArmSetBootState(ARM_BOOT_STABILIZE, now_ms);
            } else if (ArmToolGetState()->init_state == ARM_TOOL_INIT_ERROR ||
                       g_arm_boot_debug.elapsed_ms >=
                           ARM_BOOT_TOOL_INIT_TIMEOUT_MS) {
                g_arm_boot_debug.motion_result =
                    ARM_MOTION_RESULT_NOT_READY;
                ArmSetBootState(ARM_BOOT_FAULT, now_ms);
            }
            break;

        case ARM_BOOT_STABILIZE:
            if (!ArmAllFeedbackValid(now_ms) ||
                !ArmAllTargetsSynced() || !ArmAllMotorsEnabled()) {
                ArmSetBootState(ARM_BOOT_FAULT, now_ms);
                break;
            }
            if (g_arm_boot_debug.elapsed_ms >=
                ARM_BOOT_STABILIZE_MS) {
#if ARM_TOOL_ENABLE != 0u
                ArmSetBootState(
                    ARM_BOOT_GRIPPER_READY_COMMAND,
                    now_ms);
#else
                /* 主臂台架模式不等待夹爪，三轴归正稳定后直接进入READY。 */
                ArmSetBootState(ARM_BOOT_READY, now_ms);
#endif
            }
            break;

        case ARM_BOOT_GRIPPER_READY_COMMAND:
        {
            Arm_Command_Result_e gripper_result =
                ArmToolSetGripper(ARM_GRIPPER_COMMAND_READY);

            if (gripper_result == ARM_COMMAND_OK) {
                ArmSetBootState(ARM_BOOT_GRIPPER_READY_WAIT, now_ms);
            } else if (gripper_result != ARM_COMMAND_BUSY) {
                g_arm_boot_debug.motion_result =
                    ARM_MOTION_RESULT_NOT_READY;
                ArmSetBootState(ARM_BOOT_FAULT, now_ms);
            }
            break;
        }

        case ARM_BOOT_GRIPPER_READY_WAIT:
            if (ArmToolGetState()->gripper_state == ARM_GRIPPER_READY) {
                ArmSetBootState(ARM_BOOT_READY, now_ms);
            } else if (ArmToolGripperFaulted() != 0u ||
                       g_arm_boot_debug.elapsed_ms >=
                           ARM_GRIPPER_CLOSE_DEADLINE_MS) {
                g_arm_boot_debug.motion_result =
                    ARM_MOTION_RESULT_NOT_READY;
                ArmSetBootState(ARM_BOOT_FAULT, now_ms);
            }
            break;

        case ARM_BOOT_READY:
            g_arm_state.mode = ARM_MODE_READY;
            g_arm_state.start_state = ARM_START_READY;
            ArmTrajectoryTask(now_ms);
            break;

        case ARM_BOOT_FAULT:
        default:
            ArmToolStopServo1Tracking();
            ArmToolClearPendingCommands();
            ArmTrajectoryCancel();
            g_arm_state.mode = ARM_MODE_FAULT;
            g_arm_state.start_state = ARM_START_FAULT;
            g_arm_state.active_axis = ARM_AXIS_NONE;
            break;
    }
}

void ArmProcessStartup(uint32_t now_ms)
{
    uint8_t escape_axis;
    float escape_target;

    if (g_arm_state.fault_latched != ARM_FAULT_NONE) {
        ArmProcessFaultReset(now_ms);
        return;
    }
    if (ArmTemperatureAtOrAbove(ARM_TEMPERATURE_DISABLE_C)) {
        ArmLatchFault(ARM_FAULT_OVER_TEMPERATURE);
        return;
    }
    if (ArmAnyTxFault()) {
        ArmLatchFault(ARM_FAULT_CAN_TX);
        return;
    }

    switch (g_arm_state.start_state) {
        case ARM_START_REGISTERED:
            if ((uint32_t)(now_ms - arm_runtime.boot_tick) <
                ARM_DM_POWER_ON_DELAY_MS) {
                break;
            }
            ArmSetStartState(ARM_START_WAIT_PASSIVE_FEEDBACK, now_ms);
            break;

        case ARM_START_WAIT_PASSIVE_FEEDBACK:
            if (ArmAllFeedbackValid(now_ms) ||
                (uint32_t)(now_ms - arm_runtime.state_tick) >=
                    ARM_PASSIVE_FEEDBACK_WAIT_MS) {
                ArmSetStartState(ARM_START_ENTER_ALL_MODES, now_ms);
            }
            break;

        case ARM_START_ENTER_ALL_MODES:
            if (!ArmEnterAllModes()) {
                ArmLatchFault(ARM_FAULT_CAN_TX);
                break;
            }
            ArmSetStartState(ARM_START_WAIT_ENABLE_CONFIRM, now_ms);
            break;

        case ARM_START_WAIT_ENABLE_CONFIRM:
            if (ArmAllFeedbackValid(now_ms) && ArmAllModesConfirmed()) {
                ArmSetStartState(ARM_START_SYNC_TARGETS, now_ms);
            } else if (ArmAllFeedbackValid(now_ms) &&
                       ArmAnyMotorStateFault()) {
                ArmLatchFault(ARM_FAULT_MOTOR_STATE);
            } else if ((uint32_t)(now_ms - arm_runtime.state_tick) >=
                       ARM_ENTER_MODE_FEEDBACK_WAIT_MS) {
                ArmLatchFault(ArmAllFeedbackValid(now_ms) ?
                    ARM_FAULT_MOTOR_ENABLE_TIMEOUT :
                    ARM_FAULT_FEEDBACK_TIMEOUT);
            }
            break;

        case ARM_START_SYNC_TARGETS:
            if (!ArmAllModesConfirmed()) {
                ArmLatchFault(ARM_FAULT_MOTOR_ENABLE_TIMEOUT);
            } else if (!ArmSyncAllCurrentTargets()) {
                ArmLatchFault(ARM_FAULT_CONFIG);
            } else {
                ArmSetStartState(ARM_START_VALIDATE_POSITION, now_ms);
            }
            break;

        case ARM_START_VALIDATE_POSITION:
            if (!ArmAllFeedbackValid(now_ms)) {
                ArmLatchFault(ARM_FAULT_FEEDBACK_TIMEOUT);
            } else if (ArmAnyMotorStateFault()) {
                ArmLatchFault(ARM_FAULT_MOTOR_STATE);
            } else if (ArmTemperatureAtOrAbove(ARM_TEMPERATURE_HOLD_C)) {
                ArmLatchFault(ARM_FAULT_OVER_TEMPERATURE);
            } else if (!ArmFindEscapeAxis(&escape_axis, &escape_target)) {
                ArmLatchFault(ARM_FAULT_HARD_BOUNDARY);
            } else {
                if (escape_axis == ARM_AXIS_NONE) {
                    ArmBeginReturnSequence(now_ms);
                } else {
                    Arm_Start_State_e state =
                        ArmEscapeStateForAxis(escape_axis);
                    ArmSetStartState(state, now_ms);
                    if (!ArmStartAxisMotion(escape_axis, escape_target,
                                            ARM_ESCAPE_SPEED_DEG_S,
                                            now_ms, 1u)) {
                        ArmLatchFault(ARM_FAULT_CAN_TX);
                    }
                }
            }
            break;

        case ARM_START_ESCAPE_ELBOW:
        case ARM_START_ESCAPE_SHOULDER:
        case ARM_START_ESCAPE_BASE:
            if (ArmTemperatureAtOrAbove(ARM_TEMPERATURE_HOLD_C)) {
                ArmLatchFault(ARM_FAULT_OVER_TEMPERATURE);
                break;
            }
            if (ArmRunAxisMotion(g_arm_state.active_axis, now_ms,
                                 ARM_ESCAPE_TIMEOUT_MS, 1u)) {
                if (!ArmFindEscapeAxis(&escape_axis, &escape_target)) {
                    ArmLatchFault(ARM_FAULT_HARD_BOUNDARY);
                } else if (escape_axis == ARM_AXIS_NONE) {
                    ArmBeginReturnSequence(now_ms);
                } else {
                    ArmSetStartState(ArmEscapeStateForAxis(escape_axis),
                                     now_ms);
                    if (!ArmStartAxisMotion(escape_axis, escape_target,
                                            ARM_ESCAPE_SPEED_DEG_S,
                                            now_ms, 1u)) {
                        ArmLatchFault(ARM_FAULT_CAN_TX);
                    }
                }
            }
            break;

        case ARM_START_RETURN_ELBOW:
            if (ArmRunAxisMotion(ARM_JOINT_ELBOW, now_ms,
                                 ARM_RETURN_TIMEOUT_MS, 0u)) {
                ArmSetStartState(ARM_START_RETURN_SHOULDER, now_ms);
                if (!ArmStartAxisMotion(ARM_JOINT_SHOULDER,
                                        ARM_SAFE_Q2_DEG,
                                        ARM_RETURN_SPEED_DEG_S,
                                        now_ms, 0u)) {
                    ArmLatchFault(ARM_FAULT_CAN_TX);
                }
            }
            break;

        case ARM_START_RETURN_SHOULDER:
            if (ArmRunAxisMotion(ARM_JOINT_SHOULDER, now_ms,
                                 ARM_RETURN_TIMEOUT_MS, 0u)) {
                ArmSetStartState(ARM_START_RETURN_BASE, now_ms);
                if (!ArmStartAxisMotion(ARM_JOINT_BASE_YAW,
                                        ARM_SAFE_Q1_DEG,
                                        ARM_RETURN_SPEED_DEG_S,
                                        now_ms, 0u)) {
                    ArmLatchFault(ARM_FAULT_CAN_TX);
                }
            }
            break;

        case ARM_START_RETURN_BASE:
            if (ArmRunAxisMotion(ARM_JOINT_BASE_YAW, now_ms,
                                 ARM_RETURN_TIMEOUT_MS, 0u)) {
                g_arm_state.active_axis = ARM_AXIS_NONE;
                g_arm_state.mode = ARM_MODE_READY;
                ArmSetStartState(ARM_START_READY, now_ms);
            }
            break;

        case ARM_START_READY:
            arm_runtime.elbow_coupling_active = 1u;
            if (!ArmAllFeedbackValid(now_ms)) {
                ArmLatchFault(ARM_FAULT_FEEDBACK_TIMEOUT);
            } else if (ArmAnyMotorStateFault()) {
                ArmLatchFault(ARM_FAULT_MOTOR_STATE);
            } else if (ArmTemperatureAtOrAbove(ARM_TEMPERATURE_HOLD_C)) {
                ArmTrajectoryCancel();
                ArmSyncAllCurrentTargets();
            }
            break;

        case ARM_START_FAULT:
        case ARM_START_ESTOP:
        default:
            break;
    }
}

void ArmInit(void)
{
    DM_Motor_Init_Config_s config;
    float self_test_error_mm = 0.0f;

    memset(&g_arm_state, 0, sizeof(g_arm_state));
    memset(&g_arm_dm_debug, 0, sizeof(g_arm_dm_debug));
    memset(&g_arm_home_joint_debug, 0, sizeof(g_arm_home_joint_debug));
    memset(&g_arm_kinematics_debug, 0, sizeof(g_arm_kinematics_debug));
    memset(&g_arm_control_debug, 0, sizeof(g_arm_control_debug));
    memset(&g_arm_teach_point, 0, sizeof(g_arm_teach_point));
    memset(&g_arm_host_status, 0, sizeof(g_arm_host_status));
    memset(&g_arm_boot_debug, 0, sizeof(g_arm_boot_debug));
    memset(&arm_joint, 0, sizeof(arm_joint));
    memset(&arm_runtime, 0, sizeof(arm_runtime));
    memset(&arm_command_mailbox, 0, sizeof(arm_command_mailbox));
    g_arm_host_status.state = ARM_HOST_STATE_STARTING;

    arm_joint[0].motor_zero_trim_rad = ARM_BASE_MOTOR_ZERO_TRIM_RAD;
    arm_joint[0].logical_zero_deg = ARM_BASE_LOGICAL_ZERO_DEG;
    arm_joint[0].direction = ARM_BASE_DIRECTION;
    arm_joint[0].soft_min_deg = ARM_Q1_SOFT_MIN_DEG;
    arm_joint[0].soft_max_deg = ARM_Q1_SOFT_MAX_DEG;
    arm_joint[0].escape_min_deg = ARM_Q1_ESCAPE_MIN_DEG;
    arm_joint[0].escape_max_deg = ARM_Q1_ESCAPE_MAX_DEG;
    arm_joint[1].motor_zero_trim_rad = ARM_SHOULDER_MOTOR_ZERO_TRIM_RAD;
    arm_joint[1].logical_zero_deg = ARM_SHOULDER_LOGICAL_ZERO_DEG;
    arm_joint[1].direction = ARM_SHOULDER_DIRECTION;
    arm_joint[1].soft_min_deg = ARM_Q2_SOFT_MIN_DEG;
    arm_joint[1].soft_max_deg = ARM_Q2_SOFT_MAX_DEG;
    arm_joint[1].escape_min_deg = ARM_Q2_ESCAPE_MIN_DEG;
    arm_joint[1].escape_max_deg = ARM_Q2_ESCAPE_MAX_DEG;
    arm_joint[2].motor_zero_trim_rad = ARM_ELBOW_MOTOR_ZERO_TRIM_RAD;
    arm_joint[2].logical_zero_deg = ARM_ELBOW_LOGICAL_ZERO_DEG;
    arm_joint[2].direction = ARM_ELBOW_DIRECTION;
    arm_joint[2].soft_min_deg = ARM_Q3_SOFT_MIN_DEG;
    arm_joint[2].soft_max_deg = ARM_Q3_SOFT_MAX_DEG;
    arm_joint[2].escape_min_deg = ARM_Q3_ESCAPE_MIN_DEG;
    arm_joint[2].escape_max_deg = ARM_Q3_ESCAPE_MAX_DEG;

    memset(&config, 0, sizeof(config));
    config.can_handle = &hcan1;
    config.control_type = MOTOR_CONTROL_POSITION_AND_SPEED;
    config.direction = MOTOR_DIRECTION_NORMAL;

    config.motor_id = ARM_BASE_MOTOR_ID;
    config.master_id = ARM_BASE_MASTER_ID;
    config.motor_type = DM4310;
    arm_joint[0].motor = DMMotorInit(&config);
    config.motor_id = ARM_SHOULDER_MOTOR_ID;
    config.master_id = ARM_SHOULDER_MASTER_ID;
    config.motor_type = DM4340;
    arm_joint[1].motor = DMMotorInit(&config);
    config.motor_id = ARM_ELBOW_MOTOR_ID;
    config.master_id = ARM_ELBOW_MASTER_ID;
    config.motor_type = DM4310;
    arm_joint[2].motor = DMMotorInit(&config);

    g_arm_state.config_valid = arm_joint[0].motor != NULL &&
        arm_joint[1].motor != NULL && arm_joint[2].motor != NULL &&
        arm_joint[0].motor->command_id == ARM_BASE_COMMAND_ID &&
        arm_joint[1].motor->command_id == ARM_SHOULDER_COMMAND_ID &&
        arm_joint[2].motor->command_id == ARM_ELBOW_COMMAND_ID;
    g_arm_state.kinematics_valid = ArmKinematicsSelfTest(
        &self_test_error_mm);
    g_arm_state.active_axis = ARM_AXIS_NONE;
    g_arm_state.start_state = ARM_START_REGISTERED;
    g_arm_state.mode = ARM_MODE_SAFE;
    g_arm_state.fault_reset_result = ARM_FAULT_RESET_NONE;
    arm_runtime.boot_tick = HAL_GetTick();
    arm_runtime.state_tick = arm_runtime.boot_tick;
    arm_runtime.auto_init_tick = arm_runtime.boot_tick;
    arm_runtime.auto_init_initialized = 0u;
    arm_runtime.elbow_coupling_active = 0u;
    g_arm_dm_debug.auto_init.enable = ARM_DM_AUTO_INIT_ENABLE;
    g_arm_dm_debug.auto_init.state = ARM_DM_AUTO_INIT_IDLE;
    g_arm_dm_debug.auto_init.axis = ARM_DM_TEST_NONE;
    g_arm_dm_debug.auto_init.step = 0u;
    g_arm_dm_debug.auto_init.done = 0u;
    g_arm_dm_debug.auto_init.result = ARM_COMMAND_NOT_READY;
    g_arm_dm_debug.auto_init.speed_deg_s = ARM_DM_AUTO_INIT_SPEED_DEG_S;
    g_arm_dm_debug.auto_init.start_deg = NAN;
    g_arm_dm_debug.auto_init.target_deg = NAN;
    /* 兼容保留target_tool_tip_mm字段名；其内容是ID1俯仰舵机轴心。 */
    g_arm_dm_debug.auto_init.target_tool_tip_mm.x_mm = ARM_USB_HOME_X_MM;
    g_arm_dm_debug.auto_init.target_tool_tip_mm.y_mm = ARM_USB_HOME_Y_MM;
    g_arm_dm_debug.auto_init.target_tool_tip_mm.z_mm = ARM_USB_HOME_Z_MM;
    g_arm_dm_debug.auto_init.target_wrist_mm.x_mm = 0.0f;
    g_arm_dm_debug.auto_init.target_wrist_mm.y_mm = 0.0f;
    g_arm_dm_debug.auto_init.target_wrist_mm.z_mm = 0.0f;
    g_arm_dm_debug.auto_init.ik_status = ARM_IK_INVALID_ARGUMENT;
    memset(g_arm_dm_debug.auto_init.target_q_deg, 0,
           sizeof(g_arm_dm_debug.auto_init.target_q_deg));
    g_arm_dm_debug.auto_init.elapsed_ms = 0u;
    g_arm_dm_debug.auto_init.cycle_count = 0u;
    /* 先使能三台达妙原位保持，随后q2/q3同步、q1单独自动HOME。 */
    g_arm_boot_debug.state = ARM_BOOT_WAIT_MOTORS;
    g_arm_boot_debug.motion_result = ARM_MOTION_RESULT_NOT_READY;
    g_arm_boot_debug.ik_status = ARM_IK_INVALID_ARGUMENT;
    g_arm_boot_debug.state_tick = arm_runtime.boot_tick;
#if ARM_BOOT_MODE == ARM_BOOT_MODE_TEACH_POINT
    /* 打点模式只建立USART6反馈轮询，不允许工具初始化状态机发送目标。 */
    ArmToolInitFeedbackOnly();
#elif ARM_BOOT_MODE == ARM_BOOT_MODE_FULL_INIT
    /* 完整上电时延迟ID1/ID2，保证q2/q3同步HOME后再初始化工具。 */
    ArmToolPrepareDeferredInit();
#else
    ArmToolInit();
#endif
    ArmTrajectoryInit();

    if (!g_arm_state.config_valid || !g_arm_state.kinematics_valid) {
        ArmLatchFault(ARM_FAULT_CONFIG);
        return;
    }
#if ARM_BOOT_MODE == ARM_BOOT_MODE_DM_SINGLE_AXIS_TEST
    g_arm_state.mode = ARM_MODE_DM_SINGLE_AXIS_TEST;
#elif ARM_BOOT_MODE == ARM_BOOT_MODE_DM_ENABLE_ONLY
    g_arm_state.mode = ARM_MODE_DM_ENABLE_ONLY;
#elif ARM_BOOT_MODE == ARM_BOOT_MODE_TEACH_POINT
    g_arm_state.mode = ARM_MODE_TEACH_POINT;
    /* 主控单独复位时也强制退出旧Motor Mode，随后不再发送控制帧。 */
    for (uint8_t axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
        (void)DMMotorDisable(arm_joint[axis].motor);
    }
#elif ARM_BOOT_MODE == ARM_BOOT_MODE_TOOL_SERVO_INIT_ONLY
    g_arm_state.mode = ARM_MODE_TEACH_POINT;
    g_arm_state.start_state = ARM_START_WAIT_PASSIVE_FEEDBACK;
#endif
}

void ArmTask(void)
{
    uint32_t now_ms = HAL_GetTick();

    ArmUpdateFeedback(now_ms);
    ArmToolUpdateSmallLinkPitch(g_arm_state.small_link_pitch_deg);
    ArmProcessCommandMailbox(now_ms);
    /*
     * 笛卡尔命令会在邮箱处理中同步完成整条路径预检。预检期间CAN中断
     * 仍会更新电机反馈时间，因此后续在线/温度/轨迹检查必须刷新时钟，
     * 不能继续使用进入ArmTask前的旧快照。
     */
    now_ms = HAL_GetTick();
#if ARM_BOOT_MODE == ARM_BOOT_MODE_DM_ENABLE_ONLY
    ArmProcessEnableOnly(now_ms);
#elif ARM_BOOT_MODE == ARM_BOOT_MODE_TOOL_SERVO_INIT_ONLY
    /*
     * 末端舵机初始化确认模式：只执行USART6总线舵机配置目标初始化，
     * 不使能三达妙、不执行主臂初始化、不下发测试点。
     */
    g_arm_state.start_state = ArmToolReadyForMotion() ?
        ARM_START_READY : ARM_START_WAIT_PASSIVE_FEEDBACK;
#elif ARM_BOOT_MODE == ARM_BOOT_MODE_DM_SINGLE_AXIS_TEST
    if (g_arm_state.fault_latched != ARM_FAULT_NONE) {
        ArmProcessFaultReset(now_ms);
    } else {
        if (ArmAnyPreviouslySeenMotorOffline(now_ms)) {
            ArmLatchFault(ARM_FAULT_FEEDBACK_TIMEOUT);
            goto arm_task_finish;
        }
        if (ArmTemperatureAtOrAbove(ARM_TEMPERATURE_DISABLE_C)) {
            ArmLatchFault(ARM_FAULT_OVER_TEMPERATURE);
            goto arm_task_finish;
        }
        if (ArmAnyTxFault()) {
            ArmLatchFault(ARM_FAULT_CAN_TX);
            goto arm_task_finish;
        }
        /* 反馈state含义尚未实机逐状态验收，联调阶段只观察，不据此失能。 */
        if (ArmTemperatureAtOrAbove(ARM_TEMPERATURE_HOLD_C)) {
            ArmTrajectoryCancel();
            g_arm_dm_debug.auto_init.result = ARM_COMMAND_NOT_READY;
            goto arm_task_finish;
        }
        ArmProcessBootSequence(now_ms);
    }
#elif ARM_BOOT_MODE == ARM_BOOT_MODE_TEACH_POINT
    if (g_arm_state.fault_latched != ARM_FAULT_NONE) {
        ArmProcessFaultReset(now_ms);
    }
    else {
        /*
         * 打点模式不经过正常启动状态机，同步带补偿标志必须在此常开，
         * 否则q3反馈是未补偿的电机原始角（2026-08-13实测偏差达
         * q2-180，导致打点关节角完全失真）。每周期重置可在故障复位
         * 清零后自愈；q2反馈无效时补偿函数内部自动回退为0。
         */
        arm_runtime.elbow_coupling_active =
            ARM_ELBOW_SHOULDER_COUPLING_ENABLE != 0u ? 1u : 0u;
        g_arm_state.start_state = ArmAllFeedbackValid(now_ms) ?
            ARM_START_READY : ARM_START_WAIT_PASSIVE_FEEDBACK;
    }
#else
    if (g_arm_state.fault_latched != ARM_FAULT_NONE) {
        ArmProcessFaultReset(now_ms);
    }
    else {
        ArmProcessStartup(now_ms);
        if (g_arm_state.mode == ARM_MODE_READY) {
            ArmTrajectoryTask(now_ms);
        }
    }
#endif
    /* 顺序执行到此统一跳收尾；部分BOOT模式不使用该标签，避免闲置告警。 */
    goto arm_task_finish;
arm_task_finish:
    ArmUpdateFeedback(now_ms);
    ArmToolUpdateSmallLinkPitch(g_arm_state.small_link_pitch_deg);
    if (g_arm_state.mode == ARM_MODE_READY &&
        g_arm_state.start_state == ARM_START_READY &&
        g_arm_state.fault_latched == ARM_FAULT_NONE &&
        g_arm_state.kinematics_valid != 0u &&
        ARM_BOOT_MODE != ARM_BOOT_MODE_TOOL_SERVO_INIT_ONLY) {
        const Arm_Tool_State_s *tool = ArmToolGetState();

        if (!ArmTrajectoryIsBusy() && !ArmTrajectoryRealtimeActive() &&
            tool->tool_pitch_target_valid != 0u) {
            (void)ArmToolTrackPitch(tool->tool_pitch_target_deg,
                g_arm_state.small_link_pitch_deg, now_ms);
        }
    }
    /* 统一推进USART6事务，动作发送与位置/电压查询保持单owner。 */
    ArmToolTask(now_ms);
#if ARM_TOOL_ENABLE != 0u
    (void)ArmToolGetCenterFromWrist(&g_arm_state.wrist_center,
        g_arm_state.q_feedback_deg[ARM_JOINT_BASE_YAW],
        ArmToolGetState()->tool_pitch_feedback_deg,
        &g_arm_state.tool_tip);
#else
    /* 无末端舵机时，调试和上位机位置统一退化为腕部轴心。 */
    g_arm_state.tool_tip = g_arm_state.wrist_center;
#endif
    {
        const Arm_Tool_State_s *tool = ArmToolGetState();

        g_arm_state.tool_ready = tool->tool_ready;
        memcpy(g_arm_state.tool_servo_target_pos,
               tool->servo_target_pos,
               sizeof(g_arm_state.tool_servo_target_pos));
        g_arm_state.tool_error_code = tool->error_code;
    }
    ArmUpdateTeachAndKinematicsDebug();
    ArmUpdateHostStatus();
}

void ArmStop(void)
{
    ArmCancelMotion();
}

const Arm_State_s *ArmGetState(void)
{
    return &g_arm_state;
}

const Arm_Motion_Debug_s *ArmGetMotionState(void)
{
    return &g_arm_motion_debug;
}

const Arm_Teach_Point_s *ArmGetTeachPoint(void)
{
    return &g_arm_teach_point;
}

#if ARM_BOOT_MODE == ARM_BOOT_MODE_TEACH_POINT
/*
 * 打点模式达妙反馈轮询。达妙MIT协议不主动上报，电机只有收到命令帧
 * 才回复一帧反馈；打点模式不运行DMMotorControl，初始化的一次失能后
 * 反馈会冻结。这里周期性重发失能命令充当查询帧：对已失能电机没有
 * 任何力矩影响，但每帧都触发一帧反馈回复。
 * 每20ms轮转一轴，每台电机约60ms刷新一次，小于100ms在线判定超时。
 */
#define ARM_TEACH_DM_POLL_INTERVAL_MS 20u

void ArmTeachPointFeedbackPoll(uint32_t now_ms)
{
    static uint32_t next_poll_tick;
    static uint8_t poll_axis;

    if ((int32_t)(now_ms - next_poll_tick) < 0) {
        return;
    }
    next_poll_tick = now_ms + ARM_TEACH_DM_POLL_INTERVAL_MS;
    (void)DMMotorDisable(arm_joint[poll_axis].motor);
    poll_axis++;
    if (poll_axis >= ARM_AXIS_COUNT) {
        poll_axis = 0u;
    }
}
#else
void ArmTeachPointFeedbackPoll(uint32_t now_ms)
{
    /* 非打点模式由DMMotorControl周期发送，命令帧本身即反馈来源。 */
    (void)now_ms;
}
#endif

uint8_t ArmBeginJointMove(const float target_q_deg[3])
{
    uint8_t control_mode_ready = g_arm_state.mode == ARM_MODE_READY;

#if ARM_BOOT_MODE == ARM_BOOT_MODE_DM_SINGLE_AXIS_TEST
    control_mode_ready = control_mode_ready ||
        g_arm_state.mode == ARM_MODE_DM_SINGLE_AXIS_TEST;
#endif
    if (!control_mode_ready ||
        g_arm_state.start_state != ARM_START_READY ||
        g_arm_state.fault_latched != ARM_FAULT_NONE ||
        !ArmJointPoseWithinSoftLimits(target_q_deg)) {
        return 0u;
    }
    /*
     * 轨迹层会从当前反馈开始逐周期写入插值参考；这里仅做接管前检查，
     * 不提前把最终目标送到达妙电机，避免首个2ms周期出现目标跳变。
     */
    return 1u;
}

uint8_t ArmUpdateJointReference(const float reference_q_deg[3])
{
    uint8_t control_mode_ready = g_arm_state.mode == ARM_MODE_READY;

#if ARM_BOOT_MODE == ARM_BOOT_MODE_DM_SINGLE_AXIS_TEST
    control_mode_ready = control_mode_ready ||
        g_arm_state.mode == ARM_MODE_DM_SINGLE_AXIS_TEST;
#endif
    if (reference_q_deg == NULL || !control_mode_ready ||
        g_arm_state.start_state != ARM_START_READY ||
        g_arm_state.fault_latched != ARM_FAULT_NONE ||
        !ArmJointPoseWithinSoftLimits(reference_q_deg)) {
        return 0u;
    }
    return ArmCommandPose(reference_q_deg, ARM_JOINT_COMMAND_SPEED_DEG_S, 0u);
}

uint8_t ArmSetJointTargetDeg(float q1_deg, float q2_deg, float q3_deg)
{
    float target[3] = {q1_deg, q2_deg, q3_deg};
    if (!ArmBeginJointMove(target)) {
        return 0u;
    }
    return ArmUpdateJointReference(target);
}

void ArmMotionStopMotors(void)
{
    ArmSyncAllCurrentTargets();
}

void ArmUpdateControllerDebugSnapshot(Arm_Control_Debug_s *debug)
{
    (void)debug;
}

static Arm_Command_Result_e ArmExecuteJointCommand(
    const Arm_Joint_Command_s *command)
{
    Arm_Motion_Result_e result;

    if (command == NULL || !ArmJointPoseWithinSoftLimits(command->q_deg)) {
        return ARM_COMMAND_INVALID;
    }
    if (g_arm_state.mode != ARM_MODE_READY ||
        g_arm_state.start_state != ARM_START_READY ||
        g_arm_state.fault_latched != ARM_FAULT_NONE ||
        ArmTemperatureAtOrAbove(ARM_TEMPERATURE_HOLD_C) ||
        !ArmToolReadyForMotion()) {
        return ARM_COMMAND_NOT_READY;
    }
    if (ArmTrajectoryIsBusy()) {
        return ARM_COMMAND_BUSY;
    }
    if (command->move_type == ARM_MOVE_DIRECT) {
        /* 直接模式保留原语义；同步俯仰只用于有时间轴的关节轨迹。 */
        if (command->tool_relative_pitch_valid != 0u ||
            command->waypoint_valid != 0u) {
            return ARM_COMMAND_UNSUPPORTED;
        }
        result = ArmTrajectorySetJointDirect(command->q_deg);
    } else {
        result = ArmTrajectoryMoveJointWithOptions(
            command->q_deg,
            command->waypoint_valid,
            command->waypoint_q_deg,
            command->tool_relative_pitch_valid,
            command->tool_relative_pitch_deg);
    }
    if (result == ARM_MOTION_RESULT_OK) {
        return ARM_COMMAND_OK;
    }
    if (result == ARM_MOTION_RESULT_BUSY) {
        return ARM_COMMAND_BUSY;
    }
    if (result == ARM_MOTION_RESULT_NOT_READY) {
        return ARM_COMMAND_NOT_READY;
    }
    return ARM_COMMAND_PREFLIGHT_FAILED;
}

static Arm_Command_Result_e ArmExecuteCartesianCommand(
    const Arm_Cartesian_Command_s *command)
{
    Arm_Motion_Result_e result;

    if (command == NULL) {
        return ARM_COMMAND_INVALID;
    }
    if (command->tool_yaw_valid != 0u) {
        return ARM_COMMAND_UNSUPPORTED;
    }
    if (command->control_point != ARM_CONTROL_POINT_WRIST_CENTER &&
        command->control_point != ARM_CONTROL_POINT_TOOL_CENTER) {
        return ARM_COMMAND_INVALID;
    }
    if (g_arm_state.mode != ARM_MODE_READY ||
        g_arm_state.start_state != ARM_START_READY ||
        g_arm_state.fault_latched != ARM_FAULT_NONE ||
        ArmTemperatureAtOrAbove(ARM_TEMPERATURE_HOLD_C) ||
        !ArmToolReadyForMotion()) {
        return ARM_COMMAND_NOT_READY;
    }
    if (ArmTrajectoryIsBusy()) {
        return ARM_COMMAND_BUSY;
    }
    result = ArmTrajectoryStageCartesianCommand(command);
    if (result == ARM_MOTION_RESULT_OK) {
        return ARM_COMMAND_OK;
    }
    if (result == ARM_MOTION_RESULT_BUSY) {
        return ARM_COMMAND_BUSY;
    }
    if (result == ARM_MOTION_RESULT_NOT_READY) {
        return ARM_COMMAND_NOT_READY;
    }
    if (result == ARM_MOTION_RESULT_INVALID) {
        return ARM_COMMAND_INVALID;
    }
    return ARM_COMMAND_PREFLIGHT_FAILED;
}

static Arm_Command_Result_e ArmExecuteRealtimeTarget(
    const Arm_Realtime_Cartesian_Target_s *target)
{
    if (target == NULL) {
        return ARM_COMMAND_INVALID;
    }
    if (target->tool_yaw_valid != 0u) {
        return ARM_COMMAND_UNSUPPORTED;
    }
    if (g_arm_state.mode != ARM_MODE_READY ||
        g_arm_state.start_state != ARM_START_READY ||
        g_arm_state.fault_latched != ARM_FAULT_NONE ||
        ArmTemperatureAtOrAbove(ARM_TEMPERATURE_HOLD_C) ||
        !ArmToolReadyForMotion()) {
        return ARM_COMMAND_NOT_READY;
    }
    return ArmTrajectorySubmitRealtimeTarget(target);
}

static Arm_Command_Result_e ArmExecuteToolCommand(
    const Arm_Command_Tool_s *command)
{
    uint8_t ready = g_arm_state.mode == ARM_MODE_READY &&
        g_arm_state.start_state == ARM_START_READY &&
        g_arm_state.fault_latched == ARM_FAULT_NONE &&
        !ArmTemperatureAtOrAbove(ARM_TEMPERATURE_HOLD_C);

    if (command == NULL) {
        return ARM_COMMAND_INVALID;
    }

    switch (command->action) {
        case ARM_TOOL_ACTION_SET_PITCH:
            if (!ready) {
                return ARM_COMMAND_NOT_READY;
            }
            if (!ArmToolPitchValidForPose(command->pitch_deg,
                                          g_arm_state.q_feedback_deg)) {
                return ARM_COMMAND_PREFLIGHT_FAILED;
            }
            return ArmToolSetPitchDeg(command->pitch_deg);

        case ARM_TOOL_ACTION_GRIPPER_READY:
            return ready ? ArmToolSetGripper(ARM_GRIPPER_COMMAND_READY) :
                           ARM_COMMAND_NOT_READY;

        case ARM_TOOL_ACTION_GRIPPER_OPEN:
            return ArmToolSetGripper(ARM_GRIPPER_COMMAND_OPEN);

        case ARM_TOOL_ACTION_GRIPPER_CLOSE:
            return ready ? ArmToolSetGripper(ARM_GRIPPER_COMMAND_CLOSE) :
                           ARM_COMMAND_NOT_READY;

        case ARM_TOOL_ACTION_RESET_SAFE:
        {
            return ArmToolSetGripper(ARM_GRIPPER_COMMAND_OPEN);
        }

        case ARM_TOOL_ACTION_NONE:
        default:
            return ARM_COMMAND_INVALID;
    }
}

static uint8_t ArmCommandTypeIsValid(Arm_Command_Type_e type)
{
    return type > ARM_COMMAND_TYPE_NONE &&
           type <= ARM_COMMAND_TYPE_FAULT_RESET;
}

static uint8_t ArmToolCommandAllowedBeforeReady(const Arm_Command_s *command)
{
    if (command == NULL || command->type != ARM_COMMAND_TYPE_TOOL) {
        return 0u;
    }
    return command->payload.tool.action == ARM_TOOL_ACTION_GRIPPER_OPEN ||
           command->payload.tool.action == ARM_TOOL_ACTION_RESET_SAFE;
}

static uint8_t ArmCommandIdIsNewer(uint32_t command_id)
{
    uint32_t latest = arm_command_mailbox.latest_received_command_id;

    return latest == 0u || (int32_t)(command_id - latest) > 0;
}

static uint8_t ArmCommandIsDuplicate(uint32_t command_id)
{
    if (command_id == 0u) {
        return 1u;
    }
    return !ArmCommandIdIsNewer(command_id) ||
           command_id == g_arm_host_status.pending_command_id ||
           command_id == g_arm_host_status.active_command_id ||
           command_id == g_arm_host_status.last_command_id;
}

static uint8_t ArmPriorityRequestPending(void)
{
    return arm_command_mailbox.estop_requested != 0u ||
           arm_command_mailbox.cancel_requested != 0u ||
           arm_command_mailbox.stop_realtime_requested != 0u ||
           arm_command_mailbox.fault_reset_requested != 0u;
}

static void ArmHostRecordInterrupted(uint32_t command_id,
                                     Arm_Command_Type_e type,
                                     Arm_Command_State_e state)
{
    if (command_id == 0u) {
        return;
    }
    g_arm_host_status.interrupted_command_id = command_id;
    g_arm_host_status.interrupted_command_type = type;
    g_arm_host_status.interrupted_command_state = state;
    if (g_arm_host_status.active_command_id == command_id) {
        g_arm_host_status.active_command_id = 0u;
        g_arm_host_status.active_command_type = ARM_COMMAND_TYPE_NONE;
        g_arm_host_status.active_command_state = ARM_COMMAND_STATE_NONE;
    }
}

static void ArmHostInterruptPending(Arm_Command_State_e state)
{
    if (arm_command_mailbox.pending != 0u) {
        ArmHostRecordInterrupted(arm_command_mailbox.command.command_id,
                                 arm_command_mailbox.command.type, state);
    }
    arm_command_mailbox.pending = 0u;
    g_arm_host_status.pending_command_id = 0u;
    g_arm_host_status.pending_command_type = ARM_COMMAND_TYPE_NONE;
}

static void ArmHostFinishCommand(uint32_t command_id,
                                 Arm_Command_Type_e type,
                                 Arm_Command_State_e state,
                                 Arm_Command_Result_e result)
{
    g_arm_host_status.last_command_id = command_id;
    g_arm_host_status.last_command_type = type;
    g_arm_host_status.last_command_state = state;
    g_arm_host_status.last_command_result = result;
    if (g_arm_host_status.active_command_id == command_id) {
        g_arm_host_status.active_command_id = 0u;
        g_arm_host_status.active_command_type = ARM_COMMAND_TYPE_NONE;
        g_arm_host_status.active_command_state = ARM_COMMAND_STATE_NONE;
    }
}

static void ArmHostFinishInterrupted(uint32_t command_id,
                                     Arm_Command_Type_e type,
                                     Arm_Command_State_e state,
                                     Arm_Command_Result_e result)
{
    if (command_id == 0u) {
        return;
    }
    ArmHostRecordInterrupted(command_id, type, state);
    g_arm_host_status.last_command_id = command_id;
    g_arm_host_status.last_command_type = type;
    g_arm_host_status.last_command_state = state;
    g_arm_host_status.last_command_result = result;
}

static void ArmHostStartCommand(uint32_t command_id,
                                Arm_Command_Type_e type,
                                Arm_Command_State_e state)
{
    g_arm_host_status.active_command_id = command_id;
    g_arm_host_status.active_command_type = type;
    g_arm_host_status.active_command_state = state;
}

Arm_Command_Result_e ArmSubmitCommand(const Arm_Command_s *command)
{
    uint32_t primask;

    if (command == NULL || command->command_id == 0u ||
        !ArmCommandTypeIsValid(command->type)) {
        return ARM_COMMAND_INVALID;
    }
    if (command->type == ARM_COMMAND_TYPE_EMERGENCY_STOP ||
        command->type == ARM_COMMAND_TYPE_CANCEL_MOTION ||
        command->type == ARM_COMMAND_TYPE_STOP_REALTIME ||
        command->type == ARM_COMMAND_TYPE_FAULT_RESET) {
        primask = __get_PRIMASK();
        __disable_irq();
        if (ArmCommandIsDuplicate(command->command_id)) {
            if (primask == 0u) {
                __enable_irq();
            }
            return ARM_COMMAND_DUPLICATE;
        }
        if (command->type == ARM_COMMAND_TYPE_EMERGENCY_STOP) {
            arm_command_mailbox.estop_command_id = command->command_id;
            arm_command_mailbox.estop_requested = 1u;
            arm_command_mailbox.cancel_requested = 0u;
            arm_command_mailbox.stop_realtime_requested = 0u;
            arm_command_mailbox.fault_reset_requested = 0u;
        } else if (ArmPriorityRequestPending()) {
            if (primask == 0u) {
                __enable_irq();
            }
            return ARM_COMMAND_BUSY;
        } else if (command->type == ARM_COMMAND_TYPE_CANCEL_MOTION) {
            arm_command_mailbox.cancel_command_id = command->command_id;
            arm_command_mailbox.cancel_requested = 1u;
        } else if (command->type == ARM_COMMAND_TYPE_STOP_REALTIME) {
            arm_command_mailbox.stop_realtime_command_id =
                command->command_id;
            arm_command_mailbox.stop_realtime_requested = 1u;
        } else {
            arm_command_mailbox.fault_reset_command_id =
                command->command_id;
            arm_command_mailbox.fault_reset_requested = 1u;
        }
        arm_command_mailbox.latest_received_command_id =
            command->command_id;
        if (primask == 0u) {
            __enable_irq();
        }
        return ARM_COMMAND_OK;
    }

    if (ArmCommandIsDuplicate(command->command_id)) {
        return ARM_COMMAND_DUPLICATE;
    }
    if ((g_arm_state.mode != ARM_MODE_READY ||
         g_arm_state.start_state != ARM_START_READY ||
         g_arm_state.fault_latched != ARM_FAULT_NONE ||
         ArmTemperatureAtOrAbove(ARM_TEMPERATURE_HOLD_C) ||
         !ArmToolReadyForMotion()) &&
        !ArmToolCommandAllowedBeforeReady(command)) {
        return ARM_COMMAND_NOT_READY;
    }
    if (command->type == ARM_COMMAND_TYPE_REALTIME_CARTESIAN) {
        if (arm_command_mailbox.pending != 0u ||
            g_arm_host_status.active_command_id != 0u ||
            (ArmTrajectoryIsBusy() &&
             !ArmTrajectoryRealtimeActive())) {
            return ARM_COMMAND_BUSY;
        }
    } else if (arm_command_mailbox.pending != 0u ||
               g_arm_host_status.active_command_id != 0u ||
               ArmTrajectoryIsBusy() ||
               ArmTrajectoryRealtimeActive()) {
        return ARM_COMMAND_BUSY;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (ArmCommandIsDuplicate(command->command_id)) {
        if (primask == 0u) {
            __enable_irq();
        }
        return ARM_COMMAND_DUPLICATE;
    }
    if (arm_command_mailbox.pending != 0u || ArmPriorityRequestPending()) {
        if (primask == 0u) {
            __enable_irq();
        }
        return ARM_COMMAND_BUSY;
    }
    arm_command_mailbox.command = *command;
    arm_command_mailbox.pending = 1u;
    arm_command_mailbox.latest_received_command_id = command->command_id;
    g_arm_host_status.pending_command_id = command->command_id;
    g_arm_host_status.pending_command_type = command->type;
    if (primask == 0u) {
        __enable_irq();
    }
    return ARM_COMMAND_OK;
}

static void ArmProcessCommandMailbox(uint32_t now_ms)
{
    Arm_Command_s command;
    Arm_Command_Result_e result;
    uint32_t command_id;

    if (arm_command_mailbox.estop_requested != 0u) {
        command_id = arm_command_mailbox.estop_command_id;
        arm_command_mailbox.estop_requested = 0u;
        arm_command_mailbox.cancel_requested = 0u;
        arm_command_mailbox.stop_realtime_requested = 0u;
        arm_command_mailbox.fault_reset_requested = 0u;
        arm_command_mailbox.active_fault_reset_request = 0u;
        if (arm_command_mailbox.pending != 0u) {
            ArmHostFinishInterrupted(arm_command_mailbox.command.command_id,
                arm_command_mailbox.command.type,
                ARM_COMMAND_STATE_FAULTED, ARM_COMMAND_NOT_READY);
        }
        ArmHostInterruptPending(ARM_COMMAND_STATE_FAULTED);
        ArmHostFinishInterrupted(g_arm_host_status.active_command_id,
            g_arm_host_status.active_command_type,
            ARM_COMMAND_STATE_FAULTED, ARM_COMMAND_NOT_READY);
        ArmHostStartCommand(command_id, ARM_COMMAND_TYPE_EMERGENCY_STOP,
                            ARM_COMMAND_STATE_RUNNING);
        ArmEmergencyStop();
        ArmHostFinishCommand(command_id, ARM_COMMAND_TYPE_EMERGENCY_STOP,
                             ARM_COMMAND_STATE_COMPLETED, ARM_COMMAND_OK);
        return;
    }

    if (arm_command_mailbox.cancel_requested != 0u) {
        command_id = arm_command_mailbox.cancel_command_id;
        arm_command_mailbox.cancel_requested = 0u;
        if (arm_command_mailbox.pending != 0u) {
            ArmHostFinishInterrupted(arm_command_mailbox.command.command_id,
                arm_command_mailbox.command.type,
                ARM_COMMAND_STATE_CANCELLED, ARM_COMMAND_OK);
        }
        ArmHostInterruptPending(ARM_COMMAND_STATE_CANCELLED);
        ArmHostFinishInterrupted(g_arm_host_status.active_command_id,
            g_arm_host_status.active_command_type,
            ARM_COMMAND_STATE_CANCELLED, ARM_COMMAND_OK);
        ArmCancelMotion();
        ArmHostFinishCommand(command_id, ARM_COMMAND_TYPE_CANCEL_MOTION,
                             ARM_COMMAND_STATE_COMPLETED, ARM_COMMAND_OK);
        return;
    }

    if (arm_command_mailbox.stop_realtime_requested != 0u) {
        command_id = arm_command_mailbox.stop_realtime_command_id;
        arm_command_mailbox.stop_realtime_requested = 0u;
        if (g_arm_host_status.active_command_type ==
            ARM_COMMAND_TYPE_REALTIME_CARTESIAN) {
            ArmHostFinishCommand(g_arm_host_status.active_command_id,
                g_arm_host_status.active_command_type,
                ARM_COMMAND_STATE_CANCELLED, ARM_COMMAND_OK);
        }
        ArmStopRealtimeTracking();
        ArmHostFinishCommand(command_id, ARM_COMMAND_TYPE_STOP_REALTIME,
                             ARM_COMMAND_STATE_COMPLETED, ARM_COMMAND_OK);
        return;
    }

    if (arm_command_mailbox.fault_reset_requested != 0u) {
        command_id = arm_command_mailbox.fault_reset_command_id;
        arm_command_mailbox.fault_reset_requested = 0u;
        if (arm_command_mailbox.pending != 0u) {
            ArmHostFinishInterrupted(arm_command_mailbox.command.command_id,
                arm_command_mailbox.command.type,
                ARM_COMMAND_STATE_FAULTED, ARM_COMMAND_NOT_READY);
        }
        ArmHostInterruptPending(ARM_COMMAND_STATE_FAULTED);
        ArmHostFinishInterrupted(g_arm_host_status.active_command_id,
            g_arm_host_status.active_command_type,
            ARM_COMMAND_STATE_FAULTED, ARM_COMMAND_NOT_READY);
        ArmHostStartCommand(command_id, ARM_COMMAND_TYPE_FAULT_RESET,
                            ARM_COMMAND_STATE_RUNNING);
        ArmRequestFaultReset();
        arm_command_mailbox.active_fault_reset_request =
            g_arm_state.fault_reset_request;
        /* 即使当前无故障也必须处理一次，保证NOT_FAULTED请求不会悬挂。 */
        ArmProcessFaultReset(now_ms);
        return;
    }

    if (arm_command_mailbox.pending == 0u) {
        return;
    }
    command = arm_command_mailbox.command;
    arm_command_mailbox.pending = 0u;
    g_arm_host_status.pending_command_id = 0u;
    g_arm_host_status.pending_command_type = ARM_COMMAND_TYPE_NONE;

    if (command.type == ARM_COMMAND_TYPE_JOINT) {
        Arm_Joint_Command_s joint_command;

        joint_command.command_id = command.command_id;
        joint_command.move_type = command.payload.joint.move_type;
        memcpy(joint_command.q_deg, command.payload.joint.q_deg,
               sizeof(joint_command.q_deg));
        joint_command.waypoint_valid =
            command.payload.joint.waypoint_valid;
        memcpy(joint_command.waypoint_q_deg,
               command.payload.joint.waypoint_q_deg,
               sizeof(joint_command.waypoint_q_deg));
        joint_command.tool_relative_pitch_valid =
            command.payload.joint.tool_relative_pitch_valid;
        joint_command.tool_relative_pitch_deg =
            command.payload.joint.tool_relative_pitch_deg;
        result = ArmExecuteJointCommand(&joint_command);
    } else if (command.type == ARM_COMMAND_TYPE_CARTESIAN) {
        Arm_Cartesian_Command_s cartesian_command;

        cartesian_command.command_id = command.command_id;
        cartesian_command.control_point =
            command.payload.cartesian.control_point;
        cartesian_command.move_type = command.payload.cartesian.move_type;
        cartesian_command.target_mm = command.payload.cartesian.target_mm;
        cartesian_command.max_speed_mm_s =
            command.payload.cartesian.max_speed_mm_s;
        cartesian_command.tool_pitch_valid =
            command.payload.cartesian.tool_pitch_valid;
        cartesian_command.tool_pitch_deg =
            command.payload.cartesian.tool_pitch_deg;
        cartesian_command.tool_yaw_valid =
            command.payload.cartesian.tool_yaw_valid;
        cartesian_command.tool_yaw_deg =
            command.payload.cartesian.tool_yaw_deg;
        result = ArmExecuteCartesianCommand(&cartesian_command);
    } else if (command.type == ARM_COMMAND_TYPE_REALTIME_CARTESIAN) {
        Arm_Realtime_Cartesian_Target_s realtime_target;

        realtime_target.command_id = command.command_id;
        realtime_target.control_point = command.payload.realtime.control_point;
        realtime_target.target_mm = command.payload.realtime.target_mm;
        realtime_target.max_speed_mm_s =
            command.payload.realtime.max_speed_mm_s;
        realtime_target.max_acceleration_mm_s2 =
            command.payload.realtime.max_acceleration_mm_s2;
        realtime_target.tool_pitch_valid =
            command.payload.realtime.tool_pitch_valid;
        realtime_target.tool_pitch_deg =
            command.payload.realtime.tool_pitch_deg;
        realtime_target.tool_yaw_valid =
            command.payload.realtime.tool_yaw_valid;
        realtime_target.tool_yaw_deg =
            command.payload.realtime.tool_yaw_deg;
        result = ArmExecuteRealtimeTarget(&realtime_target);
    } else if (command.type == ARM_COMMAND_TYPE_TOOL) {
        result = ArmExecuteToolCommand(&command.payload.tool);
    } else {
        result = ARM_COMMAND_UNSUPPORTED;
    }

    if (result == ARM_COMMAND_OK) {
        if (command.type == ARM_COMMAND_TYPE_REALTIME_CARTESIAN) {
            ArmHostFinishCommand(command.command_id, command.type,
                                 ARM_COMMAND_STATE_COMPLETED,
                                 ARM_COMMAND_OK);
        } else {
            ArmHostStartCommand(command.command_id, command.type,
                                ARM_COMMAND_STATE_RUNNING);
        }
    } else {
        ArmHostFinishCommand(command.command_id, command.type,
                             ARM_COMMAND_STATE_REJECTED, result);
    }
}

Arm_Command_Result_e ArmSubmitJointCommand(
    const Arm_Joint_Command_s *command)
{
    Arm_Command_s host_command;

    if (command == NULL) {
        return ARM_COMMAND_INVALID;
    }
    memset(&host_command, 0, sizeof(host_command));
    host_command.command_id = command->command_id;
    host_command.type = ARM_COMMAND_TYPE_JOINT;
    host_command.payload.joint.move_type = command->move_type;
    memcpy(host_command.payload.joint.q_deg, command->q_deg,
           sizeof(host_command.payload.joint.q_deg));
    host_command.payload.joint.waypoint_valid = command->waypoint_valid;
    memcpy(host_command.payload.joint.waypoint_q_deg,
           command->waypoint_q_deg,
           sizeof(host_command.payload.joint.waypoint_q_deg));
    host_command.payload.joint.tool_relative_pitch_valid =
        command->tool_relative_pitch_valid;
    host_command.payload.joint.tool_relative_pitch_deg =
        command->tool_relative_pitch_deg;
    return ArmSubmitCommand(&host_command);
}

Arm_Command_Result_e ArmSubmitCartesianCommand(
    const Arm_Cartesian_Command_s *command)
{
    Arm_Command_s host_command;

    if (command == NULL) {
        return ARM_COMMAND_INVALID;
    }
    memset(&host_command, 0, sizeof(host_command));
    host_command.command_id = command->command_id;
    host_command.type = ARM_COMMAND_TYPE_CARTESIAN;
    host_command.payload.cartesian.control_point = command->control_point;
    host_command.payload.cartesian.move_type = command->move_type;
    host_command.payload.cartesian.target_mm = command->target_mm;
    host_command.payload.cartesian.max_speed_mm_s = command->max_speed_mm_s;
    host_command.payload.cartesian.tool_pitch_valid =
        command->tool_pitch_valid;
    host_command.payload.cartesian.tool_pitch_deg = command->tool_pitch_deg;
    host_command.payload.cartesian.tool_yaw_valid = command->tool_yaw_valid;
    host_command.payload.cartesian.tool_yaw_deg = command->tool_yaw_deg;
    return ArmSubmitCommand(&host_command);
}

Arm_Command_Result_e ArmSubmitToolCenterCommand(
    const Arm_Tool_Center_Command_s *command)
{
    Arm_Cartesian_Command_s cartesian;

    if (command == NULL) {
        return ARM_COMMAND_INVALID;
    }
    memset(&cartesian, 0, sizeof(cartesian));
    cartesian.command_id = command->command_id;
    cartesian.control_point = ARM_CONTROL_POINT_TOOL_CENTER;
    cartesian.move_type = command->move_type;
    cartesian.target_mm = command->target_center_mm;
    cartesian.max_speed_mm_s = command->max_speed_mm_s;
    cartesian.tool_pitch_valid = command->tool_pitch_valid;
    cartesian.tool_pitch_deg = command->tool_pitch_deg;
    return ArmSubmitCartesianCommand(&cartesian);
}

Arm_Command_Result_e ArmSubmitRealtimeCartesianTarget(
    const Arm_Realtime_Cartesian_Target_s *target)
{
    Arm_Command_s host_command;

    if (target == NULL) {
        return ARM_COMMAND_INVALID;
    }
    memset(&host_command, 0, sizeof(host_command));
    host_command.command_id = target->command_id;
    host_command.type = ARM_COMMAND_TYPE_REALTIME_CARTESIAN;
    host_command.payload.realtime.control_point = target->control_point;
    host_command.payload.realtime.target_mm = target->target_mm;
    host_command.payload.realtime.max_speed_mm_s = target->max_speed_mm_s;
    host_command.payload.realtime.max_acceleration_mm_s2 =
        target->max_acceleration_mm_s2;
    host_command.payload.realtime.tool_pitch_valid =
        target->tool_pitch_valid;
    host_command.payload.realtime.tool_pitch_deg = target->tool_pitch_deg;
    host_command.payload.realtime.tool_yaw_valid = target->tool_yaw_valid;
    host_command.payload.realtime.tool_yaw_deg = target->tool_yaw_deg;
    return ArmSubmitCommand(&host_command);
}

void ArmStopRealtimeTracking(void)
{
    ArmTrajectoryStopRealtime();
}

void ArmCancelMotion(void)
{
    if (g_arm_state.fault_latched != ARM_FAULT_NONE) {
        return;
    }
    ArmTrajectoryCancel();
}

void ArmEmergencyStop(void)
{
    ArmLatchFault(ARM_FAULT_EMERGENCY_STOP);
}

Arm_Fault_Reset_Result_e ArmRequestFaultReset(void)
{
    g_arm_state.fault_reset_request++;
    g_arm_state.fault_reset_result = ARM_FAULT_RESET_PENDING;
    g_arm_dm_debug.fault_reset_request = g_arm_state.fault_reset_request;
    return ARM_FAULT_RESET_PENDING;
}

static void ArmUpdateHostStatus(void)
{
    const Arm_Tool_State_s *tool = ArmToolGetState();
    Arm_Position_s current_endpoint = g_arm_state.wrist_center;
    uint8_t all_motors_ready = g_arm_state.motor_online[0] != 0u &&
        g_arm_state.motor_online[1] != 0u &&
        g_arm_state.motor_online[2] != 0u &&
        g_arm_state.motor_enabled[0] != 0u &&
        g_arm_state.motor_enabled[1] != 0u &&
        g_arm_state.motor_enabled[2] != 0u;
    uint8_t ready = g_arm_state.config_valid != 0u &&
        g_arm_state.kinematics_valid != 0u &&
        g_arm_state.mode == ARM_MODE_READY &&
        g_arm_state.start_state == ARM_START_READY &&
        g_arm_state.fault_latched == ARM_FAULT_NONE &&
        g_arm_state.all_targets_synced != 0u &&
        all_motors_ready &&
        ArmToolReadyForMotion();
    uint8_t trajectory_busy = ArmTrajectoryIsBusy();
    uint8_t realtime_active = ArmTrajectoryRealtimeActive();

    if (g_arm_host_status.active_command_id != 0u &&
        g_arm_host_status.active_command_type ==
            ARM_COMMAND_TYPE_FAULT_RESET &&
        arm_command_mailbox.active_fault_reset_request != 0u &&
        g_arm_state.fault_reset_applied ==
            arm_command_mailbox.active_fault_reset_request) {
        if (g_arm_state.fault_reset_result == ARM_FAULT_RESET_OK) {
            /* 清错成功后仍需重新使能、同步、初始化并回安全姿态。 */
            if (ready) {
                ArmHostFinishCommand(g_arm_host_status.active_command_id,
                    ARM_COMMAND_TYPE_FAULT_RESET,
                    ARM_COMMAND_STATE_COMPLETED, ARM_COMMAND_OK);
                arm_command_mailbox.active_fault_reset_request = 0u;
            }
        } else {
            ArmHostFinishCommand(g_arm_host_status.active_command_id,
                ARM_COMMAND_TYPE_FAULT_RESET,
                ARM_COMMAND_STATE_REJECTED, ARM_COMMAND_NOT_READY);
            arm_command_mailbox.active_fault_reset_request = 0u;
        }
    }

    if (g_arm_host_status.active_command_id != 0u &&
        (g_arm_host_status.active_command_type == ARM_COMMAND_TYPE_JOINT ||
         g_arm_host_status.active_command_type ==
            ARM_COMMAND_TYPE_CARTESIAN)) {
        if (g_arm_state.fault_latched != ARM_FAULT_NONE) {
            ArmHostFinishCommand(g_arm_host_status.active_command_id,
                g_arm_host_status.active_command_type,
                ARM_COMMAND_STATE_FAULTED, ARM_COMMAND_NOT_READY);
        } else if (!trajectory_busy &&
                   g_arm_motion_debug.motion_state == ARM_MOTION_HOLDING &&
                   g_arm_motion_debug.trajectory_progress >= 1.0f) {
            ArmHostFinishCommand(g_arm_host_status.active_command_id,
                g_arm_host_status.active_command_type,
                ARM_COMMAND_STATE_COMPLETED, ARM_COMMAND_OK);
        } else if (g_arm_motion_debug.motion_state == ARM_MOTION_ABORTED ||
                   g_arm_motion_debug.motion_state >=
                       ARM_MOTION_ERROR_IK) {
            ArmHostFinishCommand(g_arm_host_status.active_command_id,
                g_arm_host_status.active_command_type,
                ARM_COMMAND_STATE_FAULTED,
                g_arm_motion_debug.motion_state == ARM_MOTION_ERROR_TIMEOUT ?
                    ARM_COMMAND_TIMEOUT : ARM_COMMAND_PREFLIGHT_FAILED);
        }
    }

    if (g_arm_host_status.active_command_id != 0u &&
        g_arm_host_status.active_command_type == ARM_COMMAND_TYPE_TOOL) {
        Arm_Tool_Action_e action =
            arm_command_mailbox.command.payload.tool.action;

        if (ArmToolGripperFaulted() != 0u) {
            ArmHostFinishCommand(g_arm_host_status.active_command_id,
                ARM_COMMAND_TYPE_TOOL, ARM_COMMAND_STATE_FAULTED,
                ARM_COMMAND_NOT_READY);
        } else if (action == ARM_TOOL_ACTION_SET_PITCH) {
            if (tool->servo_feedback_valid[0] == 0u) {
                ArmHostFinishCommand(g_arm_host_status.active_command_id,
                    ARM_COMMAND_TYPE_TOOL, ARM_COMMAND_STATE_FAULTED,
                    ARM_COMMAND_NOT_READY);
            } else if (tool->servo_arrived[0] != 0u) {
                ArmHostFinishCommand(g_arm_host_status.active_command_id,
                    ARM_COMMAND_TYPE_TOOL, ARM_COMMAND_STATE_COMPLETED,
                    ARM_COMMAND_OK);
            }
            /* 夹爪动作的反馈失鲜和超时统一由ID2状态机收敛为明确终态。 */
        } else if (ArmToolGripperActionComplete() != 0u) {
            ArmHostFinishCommand(g_arm_host_status.active_command_id,
                ARM_COMMAND_TYPE_TOOL, ARM_COMMAND_STATE_COMPLETED,
                ARM_COMMAND_OK);
        }
    }

    g_arm_host_status.update_count++;
    g_arm_host_status.ready = ready;
    g_arm_host_status.busy = trajectory_busy ||
        arm_command_mailbox.pending != 0u ||
        g_arm_host_status.active_command_id != 0u;
    g_arm_host_status.realtime_active = realtime_active;
    g_arm_host_status.realtime_timed_out =
        g_arm_control_debug.realtime_timed_out;
    g_arm_host_status.command_pending = arm_command_mailbox.pending;
    g_arm_host_status.fault_code = (uint32_t)g_arm_state.fault_latched;
    g_arm_host_status.fault_reset_result =
        (uint32_t)g_arm_state.fault_reset_result;
    memcpy(g_arm_host_status.motor_online, g_arm_state.motor_online,
           sizeof(g_arm_host_status.motor_online));
    memcpy(g_arm_host_status.motor_enabled, g_arm_state.motor_enabled,
           sizeof(g_arm_host_status.motor_enabled));
    memcpy(g_arm_host_status.q_feedback_deg, g_arm_state.q_feedback_deg,
           sizeof(g_arm_host_status.q_feedback_deg));
    memcpy(g_arm_host_status.q_target_deg, g_arm_state.q_target_deg,
           sizeof(g_arm_host_status.q_target_deg));
    /* 通用position字段继续保持ID1轴心，避免改变现有上位机协议语义。 */
    g_arm_host_status.position_mm = current_endpoint;
    g_arm_host_status.target_position_mm =
        g_arm_motion_debug.target_position_mm;
    g_arm_host_status.tool_ready = tool->tool_ready;
    memcpy(g_arm_host_status.servo_online, tool->servo_online,
           sizeof(g_arm_host_status.servo_online));
    memcpy(g_arm_host_status.servo_target_pos, tool->servo_target_pos,
           sizeof(g_arm_host_status.servo_target_pos));
    g_arm_host_status.tool_pitch_target_deg =
        tool->tool_pitch_target_deg;
    g_arm_host_status.tool_pitch_feedback_deg =
        tool->tool_pitch_feedback_deg;
    g_arm_host_status.tool_pitch_servo_pos =
        tool->tool_pitch_servo_pos;
    g_arm_host_status.gripper_state = (uint8_t)tool->gripper_state;
    g_arm_host_status.gripper_target_state =
        (uint8_t)tool->gripper_target_state;
    g_arm_host_status.gripper_target_pos = tool->gripper_target_pos;
    g_arm_host_status.gripper_feedback_pos = tool->gripper_feedback_pos;
    g_arm_host_status.gripper_position_error =
        tool->gripper_position_error;
    g_arm_host_status.gripper_stall_candidate =
        tool->gripper_stall_candidate;
    g_arm_host_status.gripper_stall_latched =
        tool->gripper_stall_latched;
    g_arm_host_status.wrist_center_mm = g_arm_state.wrist_center;
    g_arm_host_status.tool_tip_mm = g_arm_state.tool_tip;
    g_arm_host_status.tool_error_code = tool->error_code;
    g_arm_host_status.trajectory_progress =
        g_arm_motion_debug.trajectory_progress;
    memcpy(g_arm_host_status.mos_temperature_c,
           g_arm_state.mos_temperature_c,
           sizeof(g_arm_host_status.mos_temperature_c));
    memcpy(g_arm_host_status.rotor_temperature_c,
           g_arm_state.rotor_temperature_c,
           sizeof(g_arm_host_status.rotor_temperature_c));

    if (g_arm_state.mode == ARM_MODE_ESTOP) {
        g_arm_host_status.state = ARM_HOST_STATE_ESTOP;
    } else if (g_arm_state.fault_latched != ARM_FAULT_NONE) {
        g_arm_host_status.state = ARM_HOST_STATE_FAULT;
    } else if (realtime_active) {
        g_arm_host_status.state = ARM_HOST_STATE_REALTIME;
    } else if (trajectory_busy ||
               g_arm_host_status.active_command_id != 0u) {
        g_arm_host_status.state = ARM_HOST_STATE_MOVING;
    } else if (ready) {
        g_arm_host_status.state = ARM_HOST_STATE_READY;
    } else {
        g_arm_host_status.state = ARM_HOST_STATE_STARTING;
    }
}

uint8_t ArmGetHostStatus(Arm_Host_Status_s *status)
{
    uint32_t primask;

    if (status == NULL) {
        return 0u;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    *status = g_arm_host_status;
    if (primask == 0u) {
        __enable_irq();
    }
    return 1u;
}
