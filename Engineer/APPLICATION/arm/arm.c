#include "arm.h"

#include "arm_config.h"
#include "arm_internal.h"
#include "arm_kinematics.h"
#include "arm_trajectory.h"
#include "arm_wrist.h"
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
    uint32_t auto_point_tick;
    float direction_start_deg;
    float active_target_deg;
    float active_speed_deg_s;
    float auto_point_target_q_deg[3];
    uint8_t enter_mode_index;
    uint8_t disable_sent;
    uint8_t resetting;
    uint8_t auto_init_initialized;
    uint8_t auto_point_initialized;
    uint8_t auto_point_first_move;
    uint8_t elbow_coupling_active;
} Arm_Runtime_s;

Arm_State_s g_arm_state;
Arm_DM_Debug_s g_arm_dm_debug;
Arm_Kinematics_Debug_s g_arm_kinematics_debug;
Arm_Control_Debug_s g_arm_control_debug;
Arm_Teach_Point_s g_arm_teach_point;

static Arm_Joint_Motor_s arm_joint[ARM_AXIS_COUNT];
static Arm_Runtime_s arm_runtime;

static uint8_t ArmCommandPose(const float pose_q_deg[3],
                              float speed_deg_s,
                              uint8_t allow_escape);
static uint8_t ArmSetJointCommandForPose(uint8_t axis,
                                         float target_deg,
                                         const float pose_q_deg[3],
                                         float speed_deg_s,
                                         uint8_t allow_escape);

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

static uint8_t ArmSetJointCommandRaw(uint8_t axis,
                                    float target_deg,
                                    float speed_deg_s,
                                    uint8_t allow_escape)
{
    return ArmSetJointCommandForPose(axis, target_deg, NULL, speed_deg_s,
                                     allow_escape);
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

static uint8_t ArmPoseArrived(const float target_q_deg[3], uint32_t now_ms)
{
    uint8_t axis;

    if (target_q_deg == NULL) {
        arm_runtime.stable_tick = 0u;
        return 0u;
    }
    for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
        float error_deg;
        float speed_deg_s;

        if (!isfinite(target_q_deg[axis]) ||
            !isfinite(arm_joint[axis].feedback_deg)) {
            arm_runtime.stable_tick = 0u;
            return 0u;
        }
        error_deg = target_q_deg[axis] - arm_joint[axis].feedback_deg;
        speed_deg_s = ArmMotorVelocityToJointDegS(
            (Arm_Joint_e)axis, arm_joint[axis].motor->measure.velocity_rad_s);
        if (!isfinite(speed_deg_s) ||
            ArmAbs(error_deg) > ARM_ARRIVAL_ERROR_DEG ||
            ArmAbs(speed_deg_s) > ARM_ARRIVAL_SPEED_DEG_S) {
            arm_runtime.stable_tick = 0u;
            return 0u;
        }
    }
    if (arm_runtime.stable_tick == 0u) {
        arm_runtime.stable_tick = now_ms;
    }
    return (uint32_t)(now_ms - arm_runtime.stable_tick) >=
        ARM_ARRIVAL_STABLE_MS;
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
            debug->tx_count = motor->tx_count;
            debug->tx_fail_count = motor->tx_fail_count;
            debug->mode_command_count = motor->mode_command_count;
            debug->last_mode_tx_id = motor->last_mode_tx_id;
        }
        g_arm_state.q_feedback_deg[axis] = joint->feedback_deg;
        g_arm_state.q_target_deg[axis] = joint->target_deg;
    }
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
    g_arm_dm_debug.fault_reset_request = g_arm_state.fault_reset_request;
    g_arm_dm_debug.fault_reset_applied = g_arm_state.fault_reset_applied;
    g_arm_dm_debug.fault_reset_result = g_arm_state.fault_reset_result;
}

static void ArmUpdateTeachAndKinematicsDebug(void)
{
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
    g_arm_teach_point.small_link_pitch_deg =
        g_arm_state.small_link_pitch_deg;
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
    static const float init_target_q_deg[3] = {
        ARM_DM_AUTO_INIT_BASE_Q_DEG,
        ARM_DM_AUTO_INIT_SHOULDER_Q_DEG,
        ARM_DM_AUTO_INIT_ELBOW_Q_DEG
    };
    Arm_DM_Auto_Init_Debug_s *init = &g_arm_dm_debug.auto_init;
    uint8_t axis;
    float tracking_error;
    float axis_speed_deg_s;

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
            for (axis = 0u; axis < ARM_AXIS_COUNT; ++axis) {
                tracking_error = arm_joint[axis].target_deg -
                    arm_joint[axis].feedback_deg;
                axis_speed_deg_s = ArmMotorVelocityToJointDegS(
                    (Arm_Joint_e)axis,
                    arm_joint[axis].motor->measure.velocity_rad_s);
                if (!isfinite(tracking_error) ||
                    !isfinite(axis_speed_deg_s) ||
                    ArmAbs(tracking_error) > ARM_ARRIVAL_ERROR_DEG ||
                    ArmAbs(axis_speed_deg_s) > ARM_ARRIVAL_SPEED_DEG_S) {
                    init->result = ARM_COMMAND_BUSY;
                    break;
                }
            }
            if (axis < ARM_AXIS_COUNT) {
                break;
            }
            init->result = ARM_COMMAND_BUSY;
            if ((uint32_t)(now_ms - arm_runtime.auto_init_tick) <
                ARM_DM_AUTO_INIT_START_DELAY_MS) {
                break;
            }
            init->step = 0u;
            init->state = ARM_DM_AUTO_INIT_MOVE_AXIS;
            break;

        case ARM_DM_AUTO_INIT_MOVE_AXIS:
            if (!ArmJointPoseWithinSoftLimits(init_target_q_deg)) {
                init->state = ARM_DM_AUTO_INIT_FAULT;
                init->result = ARM_COMMAND_PREFLIGHT_FAILED;
                break;
            }
            if (!ArmCommandPose(init_target_q_deg, init->speed_deg_s, 0u)) {
                init->state = ARM_DM_AUTO_INIT_FAULT;
                init->result = ARM_COMMAND_PREFLIGHT_FAILED;
                break;
            }
            arm_runtime.auto_init_tick = now_ms;
            arm_runtime.stable_tick = 0u;
            init->axis = ARM_DM_TEST_NONE;
            init->step = 1u;
            init->start_deg = NAN;
            init->target_deg = NAN;
            memcpy(init->target_q_deg, init_target_q_deg,
                   sizeof(init->target_q_deg));
            init->elapsed_ms = 0u;
            init->result = ARM_COMMAND_OK;
            init->state = ARM_DM_AUTO_INIT_WAIT_AXIS;
            break;

        case ARM_DM_AUTO_INIT_WAIT_AXIS:
            if (ArmPoseArrived(init_target_q_deg, now_ms)) {
                init->axis = ARM_DM_TEST_NONE;
                init->done = 1u;
                init->result = ARM_COMMAND_OK;
                init->state = ARM_DM_AUTO_INIT_DONE;
                break;
            }
            if ((uint32_t)(now_ms - arm_runtime.auto_init_tick) >=
                ARM_DM_AUTO_INIT_STEP_TIMEOUT_MS) {
                init->state = ARM_DM_AUTO_INIT_FAULT;
                init->result = ARM_COMMAND_NOT_READY;
            }
            break;

        case ARM_DM_AUTO_INIT_DONE:
            ArmSyncAllCurrentTargets();
            init->axis = ARM_DM_TEST_NONE;
            init->done = 1u;
            init->result = ARM_COMMAND_OK;
            break;

        case ARM_DM_AUTO_INIT_FAULT:
            ArmSyncAllCurrentTargets();
            break;

        case ARM_DM_AUTO_INIT_IDLE:
        default:
            arm_runtime.auto_init_initialized = 0u;
            break;
    }
}

static void ArmProcessAutoPoint(uint32_t now_ms)
{
    static const float safe_q_deg[3] = {
        ARM_SAFE_Q1_DEG,
        ARM_SAFE_Q2_DEG,
        ARM_SAFE_Q3_DEG
    };
    static const Arm_Position_s auto_points[ARM_DM_AUTO_POINT_COUNT] = {
        {ARM_DM_AUTO_POINT_1_X_MM, ARM_DM_AUTO_POINT_1_Y_MM,
         ARM_DM_AUTO_POINT_1_Z_MM},
        {ARM_DM_AUTO_POINT_2_X_MM, ARM_DM_AUTO_POINT_2_Y_MM,
         ARM_DM_AUTO_POINT_2_Z_MM},
        {ARM_DM_AUTO_POINT_3_X_MM, ARM_DM_AUTO_POINT_3_Y_MM,
         ARM_DM_AUTO_POINT_3_Z_MM},
        {ARM_DM_AUTO_POINT_4_X_MM, ARM_DM_AUTO_POINT_4_Y_MM,
         ARM_DM_AUTO_POINT_4_Z_MM},
    };
    Arm_DM_Auto_Point_Debug_s *point = &g_arm_dm_debug.auto_point;
    Arm_IK_Result_s ik_result;
    Arm_Motion_Result_e motion_result;

    if (point->enable == 0u) {
        if (point->state != ARM_DM_AUTO_POINT_IDLE) {
            ArmTrajectoryCancel();
        }
        point->state = ARM_DM_AUTO_POINT_IDLE;
        point->axis = ARM_DM_TEST_NONE;
        point->result = ARM_COMMAND_NOT_READY;
        arm_runtime.auto_point_initialized = 0u;
        arm_runtime.auto_point_first_move = 1u;
        return;
    }

    if (g_arm_dm_debug.auto_init.done == 0u ||
        g_arm_dm_debug.auto_init.state != ARM_DM_AUTO_INIT_DONE) {
        point->state = ARM_DM_AUTO_POINT_WAIT_INIT;
        point->axis = ARM_DM_TEST_NONE;
        point->result = ARM_COMMAND_NOT_READY;
        return;
    }

    if (arm_runtime.auto_point_initialized == 0u) {
        arm_runtime.auto_point_initialized = 1u;
        arm_runtime.auto_point_tick = now_ms;
        point->state = ARM_DM_AUTO_POINT_SOLVE_IK;
        point->axis = ARM_DM_TEST_NONE;
        point->step = 0u;
        point->done = 0u;
        point->result = ARM_COMMAND_NOT_READY;
        point->ik_status = ARM_IK_INVALID_ARGUMENT;
        point->target_mm = auto_points[0];
        point->start_deg = NAN;
        point->target_deg = NAN;
        point->elapsed_ms = 0u;
        point->cycle_count = 0u;
        arm_runtime.auto_point_first_move = 1u;
        return;
    }

    if (!ArmAllFeedbackValid(HAL_GetTick()) ||
        !ArmAllTargetsSynced() ||
        !ArmAllMotorsEnabled()) {
        point->state = ARM_DM_AUTO_POINT_FAULT;
        point->result = ARM_COMMAND_NOT_READY;
        return;
    }
    if (!isfinite(point->speed_mm_s) || point->speed_mm_s <= 0.0f ||
        point->speed_mm_s > ARM_LINEAR_MAX_SPEED_MM_S) {
        point->state = ARM_DM_AUTO_POINT_FAULT;
        point->result = ARM_COMMAND_INVALID;
        return;
    }

    point->elapsed_ms = now_ms - arm_runtime.auto_point_tick;
    switch (point->state) {
        case ARM_DM_AUTO_POINT_SOLVE_IK:
            memset(&ik_result, 0, sizeof(ik_result));
            point->ik_status = ArmInverseKinematics3DOF(
                &point->target_mm, g_arm_state.q_feedback_deg, &ik_result);
            if (point->ik_status != ARM_IK_OK ||
                !ArmJointPoseWithinSoftLimits(ik_result.q_deg) ||
                !ArmAutoPoseIsSafe(ik_result.q_deg)) {
                point->state = ARM_DM_AUTO_POINT_FAULT;
                point->result = ARM_COMMAND_PREFLIGHT_FAILED;
                break;
            }
            memcpy(point->target_q_deg, ik_result.q_deg,
                   sizeof(point->target_q_deg));
            memcpy(arm_runtime.auto_point_target_q_deg, ik_result.q_deg,
                   sizeof(arm_runtime.auto_point_target_q_deg));
            point->result = ARM_COMMAND_OK;
            point->state = arm_runtime.auto_point_first_move != 0u ?
                ARM_DM_AUTO_POINT_START_CONTINUOUS :
                ARM_DM_AUTO_POINT_START_LINEAR;
            break;

        case ARM_DM_AUTO_POINT_START_CONTINUOUS:
            motion_result = ArmTrajectoryMoveJointThenLinear(
                safe_q_deg, &point->target_mm, point->speed_mm_s);
            if (motion_result != ARM_MOTION_RESULT_OK) {
                point->ik_status = g_arm_motion_debug.ik_status;
                point->state = ARM_DM_AUTO_POINT_FAULT;
                point->result = motion_result == ARM_MOTION_RESULT_BUSY ?
                    ARM_COMMAND_BUSY : ARM_COMMAND_PREFLIGHT_FAILED;
                break;
            }
            memcpy(point->target_q_deg, g_arm_motion_debug.target_q_deg,
                   sizeof(point->target_q_deg));
            memcpy(arm_runtime.auto_point_target_q_deg,
                   g_arm_motion_debug.target_q_deg,
                   sizeof(arm_runtime.auto_point_target_q_deg));
            arm_runtime.auto_point_tick = now_ms;
            arm_runtime.stable_tick = 0u;
            point->elapsed_ms = 0u;
            point->result = ARM_COMMAND_OK;
            point->state = ARM_DM_AUTO_POINT_WAIT_CONTINUOUS;
            break;

        case ARM_DM_AUTO_POINT_WAIT_CONTINUOUS:
            if (!ArmTrajectoryIsBusy() &&
                g_arm_motion_debug.motion_state == ARM_MOTION_HOLDING &&
                ArmPoseArrived(arm_runtime.auto_point_target_q_deg, now_ms)) {
                point->axis = ARM_DM_TEST_NONE;
                point->done = 1u;
                point->result = ARM_COMMAND_OK;
                arm_runtime.auto_point_first_move = 0u;
                point->state = ARM_DM_AUTO_POINT_WAIT_INTERVAL;
                break;
            }
            if ((uint32_t)(now_ms - arm_runtime.auto_point_tick) >=
                ARM_DM_AUTO_POINT_STEP_TIMEOUT_MS) {
                ArmTrajectoryCancel();
                point->state = ARM_DM_AUTO_POINT_FAULT;
                point->result = ARM_COMMAND_NOT_READY;
            }
            break;

        case ARM_DM_AUTO_POINT_START_LINEAR:
            motion_result = ArmMoveLinear(&point->target_mm,
                                          point->speed_mm_s);
            if (motion_result != ARM_MOTION_RESULT_OK) {
                point->ik_status = g_arm_motion_debug.ik_status;
                point->state = ARM_DM_AUTO_POINT_FAULT;
                point->result = motion_result == ARM_MOTION_RESULT_BUSY ?
                    ARM_COMMAND_BUSY : ARM_COMMAND_PREFLIGHT_FAILED;
                break;
            }
            memcpy(point->target_q_deg, g_arm_motion_debug.target_q_deg,
                   sizeof(point->target_q_deg));
            memcpy(arm_runtime.auto_point_target_q_deg,
                   g_arm_motion_debug.target_q_deg,
                   sizeof(arm_runtime.auto_point_target_q_deg));
            arm_runtime.auto_point_tick = now_ms;
            arm_runtime.stable_tick = 0u;
            point->done = 0u;
            point->elapsed_ms = 0u;
            point->result = ARM_COMMAND_OK;
            point->state = ARM_DM_AUTO_POINT_WAIT_LINEAR;
            break;

        case ARM_DM_AUTO_POINT_WAIT_LINEAR:
            if (!ArmTrajectoryIsBusy() &&
                g_arm_motion_debug.motion_state == ARM_MOTION_HOLDING &&
                ArmPoseArrived(arm_runtime.auto_point_target_q_deg, now_ms)) {
                point->axis = ARM_DM_TEST_NONE;
                point->done = 1u;
                point->result = ARM_COMMAND_OK;
                point->state = ARM_DM_AUTO_POINT_WAIT_INTERVAL;
                break;
            }
            if ((uint32_t)(now_ms - arm_runtime.auto_point_tick) >=
                ARM_DM_AUTO_POINT_STEP_TIMEOUT_MS) {
                ArmTrajectoryCancel();
                point->state = ARM_DM_AUTO_POINT_FAULT;
                point->result = ARM_COMMAND_NOT_READY;
            }
            break;

        case ARM_DM_AUTO_POINT_WAIT_INTERVAL:
            point->axis = ARM_DM_TEST_NONE;
            point->done = 1u;
            point->result = ARM_COMMAND_OK;
            if ((uint32_t)(now_ms - arm_runtime.auto_point_tick) >=
                ARM_DM_AUTO_POINT_INTERVAL_MS) {
                point->step++;
                if (point->step >= ARM_DM_AUTO_POINT_COUNT) {
                    point->step = 0u;
                    point->cycle_count++;
                }
                point->target_mm = auto_points[point->step];
                point->done = 0u;
                point->ik_status = ARM_IK_INVALID_ARGUMENT;
                point->state = ARM_DM_AUTO_POINT_SOLVE_IK;
            }
            break;

        case ARM_DM_AUTO_POINT_FAULT:
            if (ArmTrajectoryIsBusy()) {
                ArmTrajectoryCancel();
            }
            break;

        case ARM_DM_AUTO_POINT_IDLE:
        case ARM_DM_AUTO_POINT_WAIT_INIT:
        default:
            arm_runtime.auto_point_initialized = 0u;
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
    memset(&g_arm_kinematics_debug, 0, sizeof(g_arm_kinematics_debug));
    memset(&g_arm_control_debug, 0, sizeof(g_arm_control_debug));
    memset(&g_arm_teach_point, 0, sizeof(g_arm_teach_point));
    memset(&arm_joint, 0, sizeof(arm_joint));
    memset(&arm_runtime, 0, sizeof(arm_runtime));

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
    arm_runtime.auto_point_tick = arm_runtime.boot_tick;
    memset(arm_runtime.auto_point_target_q_deg, 0,
           sizeof(arm_runtime.auto_point_target_q_deg));
    arm_runtime.auto_init_initialized = 0u;
    arm_runtime.auto_point_initialized = 0u;
    arm_runtime.auto_point_first_move = 1u;
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
    g_arm_dm_debug.auto_init.target_q_deg[0] = ARM_DM_AUTO_INIT_BASE_Q_DEG;
    g_arm_dm_debug.auto_init.target_q_deg[1] =
        ARM_DM_AUTO_INIT_SHOULDER_Q_DEG;
    g_arm_dm_debug.auto_init.target_q_deg[2] = ARM_DM_AUTO_INIT_ELBOW_Q_DEG;
    g_arm_dm_debug.auto_init.elapsed_ms = 0u;
    g_arm_dm_debug.auto_init.cycle_count = 0u;
    g_arm_dm_debug.auto_point.enable = ARM_DM_AUTO_POINT_ENABLE;
    g_arm_dm_debug.auto_point.state = ARM_DM_AUTO_POINT_IDLE;
    g_arm_dm_debug.auto_point.axis = ARM_DM_TEST_NONE;
    g_arm_dm_debug.auto_point.step = 0u;
    g_arm_dm_debug.auto_point.done = 0u;
    g_arm_dm_debug.auto_point.result = ARM_COMMAND_NOT_READY;
    g_arm_dm_debug.auto_point.ik_status = ARM_IK_INVALID_ARGUMENT;
    g_arm_dm_debug.auto_point.target_mm.x_mm = ARM_DM_AUTO_POINT_1_X_MM;
    g_arm_dm_debug.auto_point.target_mm.y_mm = ARM_DM_AUTO_POINT_1_Y_MM;
    g_arm_dm_debug.auto_point.target_mm.z_mm = ARM_DM_AUTO_POINT_1_Z_MM;
    memset(g_arm_dm_debug.auto_point.target_q_deg, 0,
           sizeof(g_arm_dm_debug.auto_point.target_q_deg));
    g_arm_dm_debug.auto_point.speed_mm_s = ARM_DM_AUTO_POINT_SPEED_MM_S;
    g_arm_dm_debug.auto_point.start_deg = NAN;
    g_arm_dm_debug.auto_point.target_deg = NAN;
    g_arm_dm_debug.auto_point.elapsed_ms = 0u;
    g_arm_dm_debug.auto_point.cycle_count = 0u;
    ArmWristInit();
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
#endif
}

void ArmTask(void)
{
    uint32_t now_ms = HAL_GetTick();

    ArmUpdateFeedback(now_ms);
#if ARM_BOOT_MODE == ARM_BOOT_MODE_DM_ENABLE_ONLY
    ArmProcessEnableOnly(now_ms);
#elif ARM_BOOT_MODE == ARM_BOOT_MODE_DM_SINGLE_AXIS_TEST
    if (g_arm_state.fault_latched != ARM_FAULT_NONE) {
        ArmProcessFaultReset(now_ms);
    } else {
        /* 先完整复用已经实机通过的三轴使能和当前位置保持流程。 */
        ArmProcessEnableOnly(now_ms);
        if (g_arm_state.start_state != ARM_START_READY) {
            ArmUpdateFeedback(now_ms);
            ArmUpdateTeachAndKinematicsDebug();
            return;
        }
        if (ArmAnyPreviouslySeenMotorOffline(now_ms)) {
            ArmLatchFault(ARM_FAULT_FEEDBACK_TIMEOUT);
            ArmUpdateFeedback(now_ms);
            ArmUpdateTeachAndKinematicsDebug();
            return;
        }
        if (ArmTemperatureAtOrAbove(ARM_TEMPERATURE_DISABLE_C)) {
            ArmLatchFault(ARM_FAULT_OVER_TEMPERATURE);
            ArmUpdateFeedback(now_ms);
            ArmUpdateTeachAndKinematicsDebug();
            return;
        }
        if (ArmAnyTxFault()) {
            ArmLatchFault(ARM_FAULT_CAN_TX);
            ArmUpdateFeedback(now_ms);
            ArmUpdateTeachAndKinematicsDebug();
            return;
        }
        /* 反馈state含义尚未实机逐状态验收，联调阶段只观察，不据此失能。 */
        if (ArmTemperatureAtOrAbove(ARM_TEMPERATURE_HOLD_C)) {
            ArmTrajectoryCancel();
            g_arm_dm_debug.auto_init.result = ARM_COMMAND_NOT_READY;
            ArmUpdateFeedback(now_ms);
            ArmUpdateTeachAndKinematicsDebug();
            return;
        }
        ArmTrajectoryTask(now_ms);
        if (g_arm_dm_debug.auto_init.state != ARM_DM_AUTO_INIT_DONE) {
            ArmProcessAutoInit(now_ms);
        }
        if (g_arm_dm_debug.auto_init.state == ARM_DM_AUTO_INIT_DONE) {
            arm_runtime.elbow_coupling_active = 1u;
            g_arm_state.mode = ARM_MODE_READY;
            ArmProcessAutoPoint(now_ms);
        }
    }
#elif ARM_BOOT_MODE == ARM_BOOT_MODE_TEACH_POINT
    if (g_arm_state.fault_latched != ARM_FAULT_NONE) {
        ArmProcessFaultReset(now_ms);
    }
    else {
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
    ArmUpdateFeedback(now_ms);
    ArmUpdateTeachAndKinematicsDebug();
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

Arm_Command_Result_e ArmSubmitJointCommand(
    const Arm_Joint_Command_s *command)
{
    Arm_Motion_Result_e result;

    if (command == NULL || !ArmJointPoseWithinSoftLimits(command->q_deg)) {
        return ARM_COMMAND_INVALID;
    }
    if (g_arm_state.mode != ARM_MODE_READY ||
        g_arm_state.start_state != ARM_START_READY ||
        g_arm_state.fault_latched != ARM_FAULT_NONE ||
        ArmTemperatureAtOrAbove(ARM_TEMPERATURE_HOLD_C)) {
        return ARM_COMMAND_NOT_READY;
    }
    if (ArmTrajectoryIsBusy()) {
        return ARM_COMMAND_BUSY;
    }
    result = command->move_type == ARM_MOVE_DIRECT ?
        ArmTrajectorySetJointDirect(command->q_deg) :
        ArmTrajectoryMoveJoint(command->q_deg);
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

Arm_Command_Result_e ArmSubmitCartesianCommand(
    const Arm_Cartesian_Command_s *command)
{
    Arm_Motion_Result_e result;
    Arm_IK_Result_s ik_result;

    if (command == NULL || command->control_point !=
            ARM_CONTROL_POINT_WRIST_CENTER || command->tool_pitch_valid) {
        return command == NULL ? ARM_COMMAND_INVALID :
                                 ARM_COMMAND_UNSUPPORTED;
    }
    if (g_arm_state.mode != ARM_MODE_READY ||
        g_arm_state.start_state != ARM_START_READY ||
        g_arm_state.fault_latched != ARM_FAULT_NONE ||
        ArmTemperatureAtOrAbove(ARM_TEMPERATURE_HOLD_C)) {
        return ARM_COMMAND_NOT_READY;
    }
    if (ArmTrajectoryIsBusy()) {
        return ARM_COMMAND_BUSY;
    }
    memset(&ik_result, 0, sizeof(ik_result));
    if (command->move_type == ARM_MOVE_DIRECT) {
        result = ArmSetCartesianTarget(&command->target_mm, &ik_result);
    } else {
        result = ArmMoveLinear(&command->target_mm,
            command->max_speed_mm_s > 0.0f ? command->max_speed_mm_s :
                                             ARM_LINEAR_DEFAULT_SPEED_MM_S);
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
    if (result == ARM_MOTION_RESULT_INVALID) {
        return ARM_COMMAND_INVALID;
    }
    return ARM_COMMAND_PREFLIGHT_FAILED;
}

Arm_Command_Result_e ArmSubmitRealtimeCartesianTarget(
    const Arm_Realtime_Cartesian_Target_s *target)
{
    if (g_arm_state.mode != ARM_MODE_READY ||
        g_arm_state.start_state != ARM_START_READY ||
        g_arm_state.fault_latched != ARM_FAULT_NONE ||
        ArmTemperatureAtOrAbove(ARM_TEMPERATURE_HOLD_C)) {
        return ARM_COMMAND_NOT_READY;
    }
    return ArmTrajectorySubmitRealtimeTarget(target);
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
