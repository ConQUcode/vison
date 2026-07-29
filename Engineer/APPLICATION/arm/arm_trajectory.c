#include "arm_trajectory.h"

#include "arm_config.h"
#include "arm_internal.h"
#include "arm_kinematics.h"
#include "stm32f4xx_hal.h"
#include "math.h"
#include "string.h"

/*
 * 轨迹层只接收关节角或腕部轴心坐标，不直接决定电机是否初始化成功。
 * 空间直线在启动前按2mm间隔完整预检并缓存IK解；运行时每1ms只在相邻
 * 合法关节解间插值。采样越密主要增加预检计算量和RAM，不直接降低速度。
 * 时间参数采用五次曲线10t^3-15t^4+6t^5，使起止速度和加速度连续。
 */

#define ARM_CARTESIAN_BOOT_DELAY_MS        1u
#define ARM_LINEAR_MIN_DISTANCE_MM         0.01f
#define ARM_LINEAR_MIN_SPEED_MM_S          1.0f
#define ARM_LINEAR_QUINTIC_PEAK_FACTOR     1.875f

#define ARM_LINEAR_Q1_STEP_MAX_DEG         5.0f
#define ARM_LINEAR_Q2_STEP_MAX_DEG         2.0f
#define ARM_LINEAR_Q3_STEP_MAX_DEG         2.0f
/*
 * M2006当前角度环DeadBand=100 motor-deg，按实测映射约等于2.68 joint-deg。
 * 中间安全姿态若仍使用2deg到位阈值，会出现电机已进入死区但状态机永远
 * 等不到“到位”的矛盾。这里取4deg，只用于安全脱限中间点，不影响最终
 * 目标精度；真正的目标轨迹仍从实时反馈姿态重新做整段IK预检。
 */
#define ARM_STAGING_SETTLE_TOLERANCE_DEG   4.0f
#define ARM_STAGING_SETTLE_TIMEOUT_MS   8000u

typedef enum {
    ARM_TRAJECTORY_PATH_NONE = 0,
    ARM_TRAJECTORY_PATH_JOINT_STAGING,
    ARM_TRAJECTORY_PATH_CARTESIAN_LINEAR
} Arm_Trajectory_Path_e;

typedef struct {
    uint32_t state_start_tick;
    uint32_t trajectory_start_tick;
    uint32_t last_task_tick;
    uint32_t trajectory_duration_ms;
    uint16_t sample_count;
    uint8_t kinematics_self_test_passed;
    uint8_t reference_update_rejected;
    uint8_t pending_cartesian_valid;
    Arm_Trajectory_Path_e path_type;
    Arm_Cartesian_Command_s pending_cartesian;
    Arm_Position_s start_position;
    Arm_Position_s target_position;
    float sample_q_deg[ARM_LINEAR_MAX_SAMPLES][3];
} Arm_Cartesian_Runtime_s;

Arm_Motion_Debug_s g_arm_motion_debug;
static Arm_Cartesian_Runtime_s arm_cartesian_runtime;

static const Arm_Position_s arm_cartesian_test_point[3] = {
    {ARM_AUTO_TEST_POINT_1_X_MM, ARM_AUTO_TEST_POINT_1_Y_MM,
     ARM_AUTO_TEST_POINT_1_Z_MM},
    {ARM_AUTO_TEST_POINT_2_X_MM, ARM_AUTO_TEST_POINT_2_Y_MM,
     ARM_AUTO_TEST_POINT_2_Z_MM},
    {ARM_AUTO_TEST_POINT_3_X_MM, ARM_AUTO_TEST_POINT_3_Y_MM,
     ARM_AUTO_TEST_POINT_3_Z_MM},
};

static const float arm_safe_staging_q_deg[3] = {0.0f, 145.0f, -145.0f};

static float ArmCartesianClamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float ArmCartesianWrapTo180(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float ArmCartesianPositionDistance(const Arm_Position_s *a,
                                          const Arm_Position_s *b)
{
    float dx = b->x_mm - a->x_mm;
    float dy = b->y_mm - a->y_mm;
    float dz = b->z_mm - a->z_mm;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

static float ArmCartesianQuintic(float normalized_time)
{
    float t = ArmCartesianClamp(normalized_time, 0.0f, 1.0f);
    float t2 = t * t;
    float t3 = t2 * t;

    return t3 * (10.0f + t * (-15.0f + 6.0f * t));
}

static uint8_t ArmCartesianMotorsReady(void)
{
    return g_arm_calibration.calibration_valid &&
           g_arm_state.soft_limit_state == ARM_SOFT_LIMIT_COMPLETE &&
           g_arm_state.motor_online[0] && g_arm_state.motor_online[1] &&
           g_arm_state.motor_online[2];
}

static void ArmCartesianSetState(Arm_Motion_State_e state, uint32_t now_ms)
{
    g_arm_motion_debug.motion_state = state;
    arm_cartesian_runtime.state_start_tick = now_ms;
}

static void ArmCartesianUpdateDebug(void)
{
    memcpy(g_arm_motion_debug.current_q_deg,
           g_arm_state.q_feedback_deg,
           sizeof(g_arm_motion_debug.current_q_deg));
    for (uint8_t i = 0u; i < 3u; ++i) {
        g_arm_motion_debug.joint_error_deg[i] = i == 0u ?
            ArmCartesianWrapTo180(g_arm_motion_debug.target_q_deg[i] -
                                  g_arm_state.q_feedback_deg[i]) :
            g_arm_motion_debug.target_q_deg[i] -
                g_arm_state.q_feedback_deg[i];
    }
    g_arm_motion_debug.current_position_mm = g_arm_state.wrist_center;
    g_arm_motion_debug.position_error_mm = ArmCartesianPositionDistance(
        &g_arm_motion_debug.current_position_mm,
        &g_arm_motion_debug.target_position_mm);
    memcpy(g_arm_motion_debug.motor_online, g_arm_state.motor_online,
           sizeof(g_arm_motion_debug.motor_online));
    memcpy(g_arm_motion_debug.motor_enabled, g_arm_state.motor_enabled,
           sizeof(g_arm_motion_debug.motor_enabled));
    g_arm_motion_debug.trajectory_duration_ms =
        arm_cartesian_runtime.trajectory_duration_ms;
    g_arm_motion_debug.path_sample_count =
        arm_cartesian_runtime.sample_count;
}

static void ArmCartesianRecordRejected(Arm_IK_Status_e ik_status)
{
    g_arm_motion_debug.ik_status = ik_status;
    g_arm_motion_debug.path_preflight_passed = 0u;
    g_arm_motion_debug.command_accepted = 0u;
    g_arm_motion_debug.command_reject_count++;
}

static uint8_t ArmCartesianJointStepContinuous(const float previous_q_deg[3],
                                               const float next_q_deg[3])
{
    return fabsf(ArmCartesianWrapTo180(next_q_deg[0] - previous_q_deg[0])) <=
               ARM_LINEAR_Q1_STEP_MAX_DEG &&
           fabsf(next_q_deg[1] - previous_q_deg[1]) <=
               ARM_LINEAR_Q2_STEP_MAX_DEG &&
           fabsf(next_q_deg[2] - previous_q_deg[2]) <=
               ARM_LINEAR_Q3_STEP_MAX_DEG;
}

/*
 * 仅做计算，不改变PID、使能状态和电机参考值。
 * 用于首次命令：在机械臂还位于硬限位时，先确认安全姿态到最终目标
 * 的整条直线都有连续合法IK，确认后才允许开始脱离硬限位。
 */
static Arm_Motion_Result_e ArmCartesianPreflightFromPose(
    const float start_q_deg[3],
    const Arm_Position_s *target)
{
    Arm_IK_Result_s ik_result;
    Arm_Position_s start_position;
    Arm_Position_s sample_position;
    float previous_q_deg[3];
    float path_length_mm;
    uint16_t sample_count;

    ArmForwardKinematics3DOF(start_q_deg[0], start_q_deg[1], start_q_deg[2],
                             &start_position);
    path_length_mm = ArmCartesianPositionDistance(&start_position, target);
    if (!isfinite(path_length_mm)) {
        return ARM_MOTION_RESULT_INVALID;
    }
    sample_count = (uint16_t)ceilf(path_length_mm /
                                  ARM_LINEAR_SAMPLE_SPACING_MM) + 1u;
    if (sample_count < 2u) {
        sample_count = 2u;
    }
    if (sample_count > ARM_LINEAR_MAX_SAMPLES) {
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }
    memcpy(previous_q_deg, start_q_deg, sizeof(previous_q_deg));
    for (uint16_t i = 1u; i < sample_count; ++i) {
        float ratio = (float)i / (float)(sample_count - 1u);

        sample_position.x_mm = start_position.x_mm +
            ratio * (target->x_mm - start_position.x_mm);
        sample_position.y_mm = start_position.y_mm +
            ratio * (target->y_mm - start_position.y_mm);
        sample_position.z_mm = start_position.z_mm +
            ratio * (target->z_mm - start_position.z_mm);
        if (ArmInverseKinematics3DOF(&sample_position, previous_q_deg,
                                    &ik_result) != ARM_IK_OK ||
            ik_result.position_error_mm > ARM_LINEAR_FK_ERROR_MAX_MM ||
            !ArmJointPoseWithinSoftLimits(ik_result.q_deg) ||
            !ArmAutoPoseIsSafe(ik_result.q_deg) ||
            !ArmCartesianJointStepContinuous(previous_q_deg,
                                             ik_result.q_deg)) {
            return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
        }
        memcpy(previous_q_deg, ik_result.q_deg, sizeof(previous_q_deg));
    }
    return ARM_MOTION_RESULT_OK;
}

static uint32_t ArmCartesianDurationMs(float path_length_mm,
                                       float max_speed_mm_s,
                                       uint16_t sample_count)
{
    /* 先满足空间速度，再按整段和局部最陡IK变化增加时间，保证三轴限速。 */
    float duration_s = ARM_LINEAR_QUINTIC_PEAK_FACTOR * path_length_mm /
                       max_speed_mm_s;
    float joint_travel_deg[3] = {0.0f, 0.0f, 0.0f};
    const float joint_speed_deg_s[3] = {
        ARM_LINEAR_Q1_MAX_SPEED_DEG_S,
        ARM_LINEAR_Q2_MAX_SPEED_DEG_S,
        ARM_LINEAR_Q3_MAX_SPEED_DEG_S,
    };

    for (uint16_t i = 1u; i < sample_count; ++i) {
        joint_travel_deg[0] += fabsf(ArmCartesianWrapTo180(
            arm_cartesian_runtime.sample_q_deg[i][0] -
            arm_cartesian_runtime.sample_q_deg[i - 1u][0]));
        joint_travel_deg[1] += fabsf(
            arm_cartesian_runtime.sample_q_deg[i][1] -
            arm_cartesian_runtime.sample_q_deg[i - 1u][1]);
        joint_travel_deg[2] += fabsf(
            arm_cartesian_runtime.sample_q_deg[i][2] -
            arm_cartesian_runtime.sample_q_deg[i - 1u][2]);
    }
    for (uint8_t joint = 0u; joint < 3u; ++joint) {
        float joint_duration_s = ARM_LINEAR_QUINTIC_PEAK_FACTOR *
                                 joint_travel_deg[joint] /
                                 joint_speed_deg_s[joint];
        if (joint_duration_s > duration_s) {
            duration_s = joint_duration_s;
        }
    }
    for (uint16_t i = 1u; i < sample_count; ++i) {
        float segment_delta_deg[3];

        segment_delta_deg[0] = fabsf(ArmCartesianWrapTo180(
            arm_cartesian_runtime.sample_q_deg[i][0] -
            arm_cartesian_runtime.sample_q_deg[i - 1u][0]));
        segment_delta_deg[1] = fabsf(
            arm_cartesian_runtime.sample_q_deg[i][1] -
            arm_cartesian_runtime.sample_q_deg[i - 1u][1]);
        segment_delta_deg[2] = fabsf(
            arm_cartesian_runtime.sample_q_deg[i][2] -
            arm_cartesian_runtime.sample_q_deg[i - 1u][2]);
        for (uint8_t joint = 0u; joint < 3u; ++joint) {
            float segment_duration_s = ARM_LINEAR_QUINTIC_PEAK_FACTOR *
                segment_delta_deg[joint] * (float)(sample_count - 1u) /
                joint_speed_deg_s[joint];

            if (segment_duration_s > duration_s) {
                duration_s = segment_duration_s;
            }
        }
    }
    if (duration_s < 0.001f) {
        duration_s = 0.001f;
    }
    return (uint32_t)(duration_s * 1000.0f + 0.999f);
}

static void ArmCartesianInterpolateJointSamples(float progress,
                                                float reference_q_deg[3])
{
    float sample_position;
    float sample_fraction;
    uint16_t lower_index;
    uint16_t upper_index;

    if (arm_cartesian_runtime.sample_count <= 1u) {
        memcpy(reference_q_deg, arm_cartesian_runtime.sample_q_deg[0],
               sizeof(float) * 3u);
        return;
    }
    sample_position = ArmCartesianClamp(progress, 0.0f, 1.0f) *
                      (float)(arm_cartesian_runtime.sample_count - 1u);
    lower_index = (uint16_t)sample_position;
    if (lower_index >= arm_cartesian_runtime.sample_count - 1u) {
        lower_index = arm_cartesian_runtime.sample_count - 1u;
        upper_index = lower_index;
        sample_fraction = 0.0f;
    } else {
        upper_index = lower_index + 1u;
        sample_fraction = sample_position - (float)lower_index;
    }
    reference_q_deg[0] = ArmCartesianWrapTo180(
        arm_cartesian_runtime.sample_q_deg[lower_index][0] +
        sample_fraction * ArmCartesianWrapTo180(
            arm_cartesian_runtime.sample_q_deg[upper_index][0] -
            arm_cartesian_runtime.sample_q_deg[lower_index][0]));
    reference_q_deg[1] =
        arm_cartesian_runtime.sample_q_deg[lower_index][1] +
        sample_fraction *
            (arm_cartesian_runtime.sample_q_deg[upper_index][1] -
             arm_cartesian_runtime.sample_q_deg[lower_index][1]);
    reference_q_deg[2] =
        arm_cartesian_runtime.sample_q_deg[lower_index][2] +
        sample_fraction *
            (arm_cartesian_runtime.sample_q_deg[upper_index][2] -
             arm_cartesian_runtime.sample_q_deg[lower_index][2]);
}

static void ArmCartesianStartPreparedTrajectory(
    const Arm_Position_s *target,
    uint32_t duration_ms,
    Arm_Trajectory_Path_e path_type,
    Arm_Motion_State_e motion_state,
    uint32_t now_ms)
{
    uint16_t last_index = arm_cartesian_runtime.sample_count - 1u;

    arm_cartesian_runtime.target_position = *target;
    arm_cartesian_runtime.trajectory_duration_ms = duration_ms;
    arm_cartesian_runtime.trajectory_start_tick = now_ms;
    arm_cartesian_runtime.path_type = path_type;
    arm_cartesian_runtime.reference_update_rejected = 0u;
    memcpy(g_arm_motion_debug.trajectory_q_deg,
           arm_cartesian_runtime.sample_q_deg[0],
           sizeof(g_arm_motion_debug.trajectory_q_deg));
    memcpy(g_arm_motion_debug.target_q_deg,
           arm_cartesian_runtime.sample_q_deg[last_index],
           sizeof(g_arm_motion_debug.target_q_deg));
    g_arm_motion_debug.target_position_mm = *target;
    g_arm_motion_debug.ik_status = ARM_IK_OK;
    g_arm_motion_debug.path_preflight_passed = 1u;
    g_arm_motion_debug.command_accepted = 1u;
    g_arm_motion_debug.trajectory_progress = 0.0f;
    g_arm_motion_debug.trajectory_elapsed_ms = 0u;
    g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_NONE;
    ArmCartesianSetState(motion_state, now_ms);
}

static Arm_Motion_Result_e ArmCartesianStartBootStaging(
    const Arm_Position_s *target)
{
    Arm_IK_Result_s result;
    float start_q_deg[3];
    uint32_t duration_ms;
    uint32_t now_ms = HAL_GetTick();

    memset(&result, 0, sizeof(result));
    if (!ArmCartesianMotorsReady()) {
        ArmCartesianRecordRejected(ARM_IK_INVALID_ARGUMENT);
        return ARM_MOTION_RESULT_NOT_READY;
    }
    start_q_deg[0] = g_arm_state.q_feedback_deg[ARM_JOINT_BASE_YAW];
    start_q_deg[1] = g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER];
    start_q_deg[2] = g_arm_state.q_feedback_deg[ARM_JOINT_ELBOW];
    if (ArmInverseKinematics3DOF(target, start_q_deg, &result) != ARM_IK_OK ||
        result.position_error_mm > ARM_LINEAR_FK_ERROR_MAX_MM ||
        !ArmJointPoseWithinSoftLimits(result.q_deg) ||
        !ArmAutoPoseIsSafe(result.q_deg)) {
        ArmCartesianRecordRejected(result.status);
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }
    if (start_q_deg[0] < ARM_AUTO_Q1_MIN_DEG ||
        start_q_deg[0] > ARM_AUTO_Q1_MAX_DEG ||
        start_q_deg[1] < 0.0f || start_q_deg[1] > 180.0f ||
        start_q_deg[2] < -180.0f || start_q_deg[2] > -85.0f ||
        result.q_deg[1] > start_q_deg[1] ||
        result.q_deg[2] < start_q_deg[2]) {
        ArmCartesianRecordRejected(ARM_IK_COLLISION_RISK);
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }

    arm_cartesian_runtime.sample_count = 2u;
    memcpy(arm_cartesian_runtime.sample_q_deg[0], start_q_deg,
           sizeof(start_q_deg));
    memcpy(arm_cartesian_runtime.sample_q_deg[1], result.q_deg,
           sizeof(result.q_deg));
    arm_cartesian_runtime.start_position = g_arm_state.wrist_center;
    duration_ms = ArmCartesianDurationMs(0.0f,
                                        ARM_LINEAR_DEFAULT_SPEED_MM_S,
                                        arm_cartesian_runtime.sample_count);
    if (!ArmBeginJointMove(result.q_deg)) {
        ArmCartesianRecordRejected(ARM_IK_INVALID_ARGUMENT);
        return ARM_MOTION_RESULT_NOT_READY;
    }
    ArmCartesianStartPreparedTrajectory(
        target, duration_ms, ARM_TRAJECTORY_PATH_JOINT_STAGING,
        ARM_MOTION_STAGING, now_ms);
    return ARM_MOTION_RESULT_OK;
}

void ArmAbortMotion(Arm_Motion_Fault_e reason)
{
    g_arm_motion_debug.fault_code = reason;
    if (reason == ARM_MOTION_FAULT_ABORT) {
        g_arm_motion_debug.motion_state = ARM_MOTION_ABORTED;
        ArmMotionStopMotors();
    }
}

uint8_t ArmTrajectoryMotorHoldAllowed(void)
{
    return g_arm_motion_debug.motion_state != ARM_MOTION_ABORTED;
}

uint8_t ArmTrajectoryOwnsControl(void)
{
    return g_arm_motion_debug.motion_state == ARM_MOTION_STAGING ||
           g_arm_motion_debug.motion_state == ARM_MOTION_RUNNING ||
           g_arm_motion_debug.motion_state == ARM_MOTION_SETTLING ||
           g_arm_motion_debug.motion_state == ARM_MOTION_HOLDING;
}

uint8_t ArmTrajectoryIsBusy(void)
{
    return g_arm_motion_debug.motion_state == ARM_MOTION_STAGING ||
           g_arm_motion_debug.motion_state == ARM_MOTION_PREFLIGHT ||
           g_arm_motion_debug.motion_state == ARM_MOTION_RUNNING ||
           g_arm_motion_debug.motion_state == ARM_MOTION_SETTLING;
}

void ArmTrajectoryCancel(void)
{
    float hold_q_deg[3];

    if (!ArmCartesianMotorsReady()) {
        return;
    }
    hold_q_deg[0] = g_arm_state.q_feedback_deg[ARM_JOINT_BASE_YAW];
    hold_q_deg[1] = g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER];
    hold_q_deg[2] = g_arm_state.q_feedback_deg[ARM_JOINT_ELBOW];
    /* 取消动作允许在安全脱限走廊内保持当前姿态，因此这里只检查底层硬限位。 */
    ArmUpdateJointReference(hold_q_deg);
    memcpy(g_arm_motion_debug.trajectory_q_deg, hold_q_deg,
           sizeof(hold_q_deg));
    memcpy(g_arm_motion_debug.target_q_deg, hold_q_deg,
           sizeof(hold_q_deg));
    g_arm_motion_debug.target_position_mm = g_arm_state.wrist_center;
    g_arm_motion_debug.trajectory_progress = 1.0f;
    g_arm_motion_debug.command_accepted = 1u;
    g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_NONE;
    arm_cartesian_runtime.pending_cartesian_valid = 0u;
    arm_cartesian_runtime.sample_count = 1u;
    memcpy(arm_cartesian_runtime.sample_q_deg[0], hold_q_deg,
           sizeof(hold_q_deg));
    ArmCartesianSetState(ARM_MOTION_HOLDING, HAL_GetTick());
}

Arm_Motion_Result_e ArmTrajectoryMoveJoint(const float target_q_deg[3])
{
    Arm_Position_s target_position;
    uint32_t duration_ms;

    if (target_q_deg == NULL || ArmTrajectoryIsBusy()) {
        return target_q_deg == NULL ? ARM_MOTION_RESULT_INVALID :
                                     ARM_MOTION_RESULT_BUSY;
    }
    if (!ArmCartesianMotorsReady()) {
        return ARM_MOTION_RESULT_NOT_READY;
    }
    if (!ArmJointPoseWithinSoftLimits(target_q_deg) ||
        !ArmAutoPoseIsSafe(target_q_deg)) {
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }
    arm_cartesian_runtime.sample_count = 2u;
    memcpy(arm_cartesian_runtime.sample_q_deg[0],
           g_arm_state.q_feedback_deg, sizeof(float) * 3u);
    memcpy(arm_cartesian_runtime.sample_q_deg[1], target_q_deg,
           sizeof(float) * 3u);
    ArmForwardKinematics3DOF(target_q_deg[0], target_q_deg[1],
                             target_q_deg[2], &target_position);
    duration_ms = ArmCartesianDurationMs(
        0.0f, ARM_LINEAR_DEFAULT_SPEED_MM_S, 2u);
    if (!ArmBeginJointMove(target_q_deg)) {
        return ARM_MOTION_RESULT_NOT_READY;
    }
    ArmCartesianStartPreparedTrajectory(
        &target_position, duration_ms, ARM_TRAJECTORY_PATH_JOINT_STAGING,
        ARM_MOTION_RUNNING, HAL_GetTick());
    return ARM_MOTION_RESULT_OK;
}

Arm_Motion_Result_e ArmTrajectorySetJointDirect(const float target_q_deg[3])
{
    Arm_Position_s target_position;

    if (target_q_deg == NULL || ArmTrajectoryIsBusy()) {
        return target_q_deg == NULL ? ARM_MOTION_RESULT_INVALID :
                                     ARM_MOTION_RESULT_BUSY;
    }
    if (!ArmCartesianMotorsReady()) {
        return ARM_MOTION_RESULT_NOT_READY;
    }
    if (!ArmJointPoseWithinSoftLimits(target_q_deg) ||
        !ArmAutoPoseIsSafe(target_q_deg)) {
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }
    if (!ArmSetJointTargetDeg(target_q_deg[0], target_q_deg[1],
                              target_q_deg[2])) {
        return ARM_MOTION_RESULT_NOT_READY;
    }
    ArmForwardKinematics3DOF(target_q_deg[0], target_q_deg[1],
                             target_q_deg[2], &target_position);
    memcpy(g_arm_motion_debug.trajectory_q_deg, target_q_deg,
           sizeof(float) * 3u);
    memcpy(g_arm_motion_debug.target_q_deg, target_q_deg,
           sizeof(float) * 3u);
    g_arm_motion_debug.target_position_mm = target_position;
    g_arm_motion_debug.command_accepted = 1u;
    g_arm_motion_debug.path_preflight_passed = 1u;
    g_arm_motion_debug.trajectory_progress = 1.0f;
    g_arm_motion_debug.trajectory_duration_ms = 0u;
    g_arm_motion_debug.trajectory_elapsed_ms = 0u;
    ArmCartesianSetState(ARM_MOTION_HOLDING, HAL_GetTick());
    return ARM_MOTION_RESULT_OK;
}

Arm_Motion_Result_e ArmSetCartesianTarget(const Arm_Position_s *target,
                                          Arm_IK_Result_s *result)
{
    Arm_IK_Result_s local_result;
    float seed_q_deg[3];

    memset(&local_result, 0, sizeof(local_result));
    local_result.status = ARM_IK_INVALID_ARGUMENT;
    if (target == NULL || !isfinite(target->x_mm) ||
        !isfinite(target->y_mm) || !isfinite(target->z_mm)) {
        ArmCartesianRecordRejected(local_result.status);
        if (result != NULL) {
            *result = local_result;
        }
        return ARM_MOTION_RESULT_INVALID;
    }
    if (!ArmCartesianMotorsReady()) {
        ArmCartesianRecordRejected(local_result.status);
        if (result != NULL) {
            *result = local_result;
        }
        return ARM_MOTION_RESULT_NOT_READY;
    }

    seed_q_deg[0] = g_arm_state.q_feedback_deg[ARM_JOINT_BASE_YAW];
    seed_q_deg[1] = g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER];
    seed_q_deg[2] = g_arm_state.q_feedback_deg[ARM_JOINT_ELBOW];
    if (ArmInverseKinematics3DOF(target, seed_q_deg, &local_result) !=
            ARM_IK_OK ||
        local_result.position_error_mm > ARM_LINEAR_FK_ERROR_MAX_MM ||
        !ArmJointPoseWithinSoftLimits(local_result.q_deg) ||
        !ArmAutoPoseIsSafe(local_result.q_deg)) {
        ArmCartesianRecordRejected(local_result.status);
        if (result != NULL) {
            *result = local_result;
        }
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }

    /* All checks finish before the three references are changed. */
    if (!ArmUpdateJointReference(local_result.q_deg)) {
        ArmCartesianRecordRejected(local_result.status);
        if (result != NULL) {
            *result = local_result;
        }
        return ARM_MOTION_RESULT_NOT_READY;
    }

    memcpy(g_arm_motion_debug.trajectory_q_deg, local_result.q_deg,
           sizeof(g_arm_motion_debug.trajectory_q_deg));
    memcpy(g_arm_motion_debug.target_q_deg, local_result.q_deg,
           sizeof(g_arm_motion_debug.target_q_deg));
    g_arm_motion_debug.target_position_mm = *target;
    g_arm_motion_debug.ik_status = ARM_IK_OK;
    g_arm_motion_debug.path_preflight_passed = 1u;
    g_arm_motion_debug.command_accepted = 1u;
    g_arm_motion_debug.trajectory_progress = 1.0f;
    g_arm_motion_debug.trajectory_duration_ms = 0u;
    g_arm_motion_debug.trajectory_elapsed_ms = 0u;
    g_arm_motion_debug.path_sample_count = 1u;
    g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_NONE;
    ArmCartesianSetState(ARM_MOTION_HOLDING, HAL_GetTick());
    if (result != NULL) {
        *result = local_result;
    }
    return ARM_MOTION_RESULT_OK;
}

Arm_Motion_Result_e ArmMoveLinear(const Arm_Position_s *target,
                                  float max_speed_mm_s)
{
    Arm_IK_Result_s ik_result;
    Arm_Position_s sample_position;
    float start_q_deg[3];
    float previous_q_deg[3];
    float path_length_mm;
    uint16_t sample_count;
    uint32_t duration_ms;
    uint32_t now_ms = HAL_GetTick();

    memset(&ik_result, 0, sizeof(ik_result));
    ik_result.status = ARM_IK_INVALID_ARGUMENT;
    if (g_arm_motion_debug.motion_state == ARM_MOTION_STAGING ||
        g_arm_motion_debug.motion_state == ARM_MOTION_RUNNING ||
        g_arm_motion_debug.motion_state == ARM_MOTION_PREFLIGHT) {
        return ARM_MOTION_RESULT_BUSY;
    }
    if (target == NULL || !isfinite(target->x_mm) ||
        !isfinite(target->y_mm) || !isfinite(target->z_mm) ||
        !isfinite(max_speed_mm_s) ||
        max_speed_mm_s < ARM_LINEAR_MIN_SPEED_MM_S) {
        ArmCartesianRecordRejected(ik_result.status);
        return ARM_MOTION_RESULT_INVALID;
    }
    if (!ArmCartesianMotorsReady()) {
        ArmCartesianRecordRejected(ik_result.status);
        return ARM_MOTION_RESULT_NOT_READY;
    }

    start_q_deg[0] = g_arm_state.q_feedback_deg[ARM_JOINT_BASE_YAW];
    start_q_deg[1] = g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER];
    start_q_deg[2] = g_arm_state.q_feedback_deg[ARM_JOINT_ELBOW];
    if (!ArmJointPoseWithinSoftLimits(start_q_deg) ||
        !ArmAutoPoseIsSafe(start_q_deg)) {
        ArmCartesianRecordRejected(ARM_IK_COLLISION_RISK);
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }

    ArmForwardKinematics3DOF(start_q_deg[0], start_q_deg[1], start_q_deg[2],
                             &arm_cartesian_runtime.start_position);
    path_length_mm = ArmCartesianPositionDistance(
        &arm_cartesian_runtime.start_position, target);
    if (!isfinite(path_length_mm)) {
        ArmCartesianRecordRejected(ARM_IK_NUMERICAL_ERROR);
        return ARM_MOTION_RESULT_INVALID;
    }
    if (path_length_mm <= ARM_LINEAR_MIN_DISTANCE_MM) {
        return ArmSetCartesianTarget(target, NULL);
    }

    sample_count = (uint16_t)ceilf(path_length_mm /
                                  ARM_LINEAR_SAMPLE_SPACING_MM) + 1u;
    if (sample_count < 2u || sample_count > ARM_LINEAR_MAX_SAMPLES) {
        ArmCartesianRecordRejected(ARM_IK_INVALID_ARGUMENT);
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }

    arm_cartesian_runtime.sample_count = sample_count;
    memcpy(arm_cartesian_runtime.sample_q_deg[0], start_q_deg,
           sizeof(start_q_deg));
    memcpy(previous_q_deg, start_q_deg, sizeof(previous_q_deg));
    for (uint16_t i = 1u; i < sample_count; ++i) {
        float ratio = (float)i / (float)(sample_count - 1u);

        sample_position.x_mm = arm_cartesian_runtime.start_position.x_mm +
            ratio * (target->x_mm -
                     arm_cartesian_runtime.start_position.x_mm);
        sample_position.y_mm = arm_cartesian_runtime.start_position.y_mm +
            ratio * (target->y_mm -
                     arm_cartesian_runtime.start_position.y_mm);
        sample_position.z_mm = arm_cartesian_runtime.start_position.z_mm +
            ratio * (target->z_mm -
                     arm_cartesian_runtime.start_position.z_mm);
        if (ArmInverseKinematics3DOF(&sample_position, previous_q_deg,
                                    &ik_result) != ARM_IK_OK ||
            ik_result.position_error_mm > ARM_LINEAR_FK_ERROR_MAX_MM ||
            !ArmJointPoseWithinSoftLimits(ik_result.q_deg) ||
            !ArmAutoPoseIsSafe(ik_result.q_deg) ||
            !ArmCartesianJointStepContinuous(previous_q_deg,
                                             ik_result.q_deg)) {
            ArmCartesianRecordRejected(ik_result.status);
            return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
        }
        memcpy(arm_cartesian_runtime.sample_q_deg[i], ik_result.q_deg,
               sizeof(ik_result.q_deg));
        memcpy(previous_q_deg, ik_result.q_deg, sizeof(previous_q_deg));
    }

    duration_ms = ArmCartesianDurationMs(path_length_mm, max_speed_mm_s,
                                        sample_count);
    if (!ArmBeginJointMove(previous_q_deg)) {
        ArmCartesianRecordRejected(ARM_IK_INVALID_ARGUMENT);
        return ARM_MOTION_RESULT_NOT_READY;
    }
    ArmCartesianStartPreparedTrajectory(
        target, duration_ms, ARM_TRAJECTORY_PATH_CARTESIAN_LINEAR,
        ARM_MOTION_RUNNING, now_ms);
    return ARM_MOTION_RESULT_OK;
}

Arm_Motion_Result_e ArmTrajectoryStageCartesianCommand(
    const Arm_Cartesian_Command_s *command)
{
    Arm_Motion_Result_e result;
    Arm_Position_s staging_position;

    if (command == NULL) {
        return ARM_MOTION_RESULT_INVALID;
    }
    if (ArmTrajectoryIsBusy()) {
        return ARM_MOTION_RESULT_BUSY;
    }
    result = ArmCartesianPreflightFromPose(arm_safe_staging_q_deg,
                                           &command->target_mm);
    if (result != ARM_MOTION_RESULT_OK) {
        return result;
    }
    arm_cartesian_runtime.pending_cartesian = *command;
    arm_cartesian_runtime.pending_cartesian_valid = 1u;
    ArmForwardKinematics3DOF(arm_safe_staging_q_deg[0],
                             arm_safe_staging_q_deg[1],
                             arm_safe_staging_q_deg[2],
                             &staging_position);
    result = ArmCartesianStartBootStaging(&staging_position);
    if (result != ARM_MOTION_RESULT_OK) {
        arm_cartesian_runtime.pending_cartesian_valid = 0u;
    }
    return result;
}

void ArmTrajectoryInit(void)
{
    memset(&arm_cartesian_runtime, 0, sizeof(arm_cartesian_runtime));
    memset(&g_arm_motion_debug, 0, sizeof(g_arm_motion_debug));
    g_arm_motion_debug.motion_state = ARM_MOTION_IDLE;
    g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_NONE;
    arm_cartesian_runtime.last_task_tick = HAL_GetTick();
    arm_cartesian_runtime.kinematics_self_test_passed =
        ArmKinematicsSelfTest(NULL);
    if (!arm_cartesian_runtime.kinematics_self_test_passed) {
        g_arm_motion_debug.motion_state = ARM_MOTION_ERROR_IK;
        g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_SELF_TEST;
    }
}

static void ArmCartesianRunPreparedTrajectory(uint32_t now_ms)
{
    float normalized_time;
    float progress;
    float reference_q_deg[3];
    uint32_t elapsed_ms =
        (uint32_t)(now_ms - arm_cartesian_runtime.trajectory_start_tick);

    if (elapsed_ms >= arm_cartesian_runtime.trajectory_duration_ms) {
        elapsed_ms = arm_cartesian_runtime.trajectory_duration_ms;
    }
    normalized_time = arm_cartesian_runtime.trajectory_duration_ms > 0u ?
        (float)elapsed_ms /
            (float)arm_cartesian_runtime.trajectory_duration_ms : 1.0f;
    progress = ArmCartesianQuintic(normalized_time);
    ArmCartesianInterpolateJointSamples(progress, reference_q_deg);
    if (!ArmUpdateJointReference(reference_q_deg)) {
        if (!arm_cartesian_runtime.reference_update_rejected) {
            g_arm_motion_debug.command_reject_count++;
            arm_cartesian_runtime.reference_update_rejected = 1u;
        }
        g_arm_motion_debug.command_accepted = 0u;
        arm_cartesian_runtime.trajectory_start_tick++;
        return;
    }

    arm_cartesian_runtime.reference_update_rejected = 0u;
    g_arm_motion_debug.command_accepted = 1u;
    g_arm_motion_debug.trajectory_elapsed_ms = elapsed_ms;
    g_arm_motion_debug.trajectory_progress = progress;
    memcpy(g_arm_motion_debug.trajectory_q_deg, reference_q_deg,
           sizeof(g_arm_motion_debug.trajectory_q_deg));

    if (elapsed_ms >= arm_cartesian_runtime.trajectory_duration_ms) {
        uint16_t last_index = arm_cartesian_runtime.sample_count - 1u;

        ArmUpdateJointReference(
            arm_cartesian_runtime.sample_q_deg[last_index]);
        memcpy(g_arm_motion_debug.trajectory_q_deg,
               arm_cartesian_runtime.sample_q_deg[last_index],
               sizeof(g_arm_motion_debug.trajectory_q_deg));
        g_arm_motion_debug.trajectory_progress = 1.0f;
        if (arm_cartesian_runtime.pending_cartesian_valid) {
            ArmCartesianSetState(ARM_MOTION_SETTLING, now_ms);
        } else {
            ArmCartesianSetState(ARM_MOTION_HOLDING, now_ms);
        }
    }
}

void ArmTrajectoryTask(uint32_t now_ms)
{
    Arm_Motion_Result_e result;
#if ARM_BOOT_MODE == ARM_BOOT_MODE_AUTO_TEST
    uint8_t next_index;
#endif
    uint32_t task_delta_ms =
        (uint32_t)(now_ms - arm_cartesian_runtime.last_task_tick);

    arm_cartesian_runtime.last_task_tick = now_ms;
    if (g_arm_homing_abort) {
        ArmAbortMotion(ARM_MOTION_FAULT_ABORT);
        ArmCartesianUpdateDebug();
        return;
    }
    if (!arm_cartesian_runtime.kinematics_self_test_passed) {
        ArmCartesianUpdateDebug();
        return;
    }
    if (!ArmCartesianMotorsReady()) {
        /* Pause trajectory time and keep the previous references. */
        if (g_arm_motion_debug.motion_state == ARM_MOTION_STAGING ||
            g_arm_motion_debug.motion_state == ARM_MOTION_RUNNING) {
            arm_cartesian_runtime.trajectory_start_tick += task_delta_ms;
        } else if (g_arm_motion_debug.motion_state == ARM_MOTION_HOLDING) {
            arm_cartesian_runtime.state_start_tick += task_delta_ms;
        }
        g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_OFFLINE;
        g_arm_motion_debug.command_accepted = 0u;
        ArmCartesianUpdateDebug();
        return;
    }
    if (g_arm_motion_debug.fault_code == ARM_MOTION_FAULT_OFFLINE) {
        g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_NONE;
    }

    switch (g_arm_motion_debug.motion_state) {
        case ARM_MOTION_IDLE:
#if ARM_BOOT_MODE == ARM_BOOT_MODE_AUTO_TEST
            ArmCartesianSetState(ARM_MOTION_BOOT_DELAY, now_ms);
#endif
            break;

        case ARM_MOTION_BOOT_DELAY:
            if ((uint32_t)(now_ms - arm_cartesian_runtime.state_start_tick) <
                ARM_CARTESIAN_BOOT_DELAY_MS) {
                break;
            }
            result = ArmCartesianStartBootStaging(
                &arm_cartesian_test_point[0]);
            if (result == ARM_MOTION_RESULT_OK) {
                g_arm_motion_debug.sequence_index = 0u;
            } else {
                arm_cartesian_runtime.state_start_tick = now_ms;
            }
            break;

        case ARM_MOTION_STAGING:
        case ARM_MOTION_RUNNING:
            ArmCartesianRunPreparedTrajectory(now_ms);
            break;

        case ARM_MOTION_SETTLING:
        {
            uint8_t staging_reached =
                fabsf(ArmCartesianWrapTo180(
                    arm_safe_staging_q_deg[0] -
                    g_arm_state.q_feedback_deg[0])) <=
                    ARM_STAGING_SETTLE_TOLERANCE_DEG &&
                fabsf(arm_safe_staging_q_deg[1] -
                      g_arm_state.q_feedback_deg[1]) <=
                    ARM_STAGING_SETTLE_TOLERANCE_DEG &&
                fabsf(arm_safe_staging_q_deg[2] -
                      g_arm_state.q_feedback_deg[2]) <=
                    ARM_STAGING_SETTLE_TOLERANCE_DEG;
            uint8_t staging_timeout =
                (uint32_t)(now_ms - arm_cartesian_runtime.state_start_tick) >=
                ARM_STAGING_SETTLE_TIMEOUT_MS;

            /*
             * 正常到位后立即继续。若因机械死区未精确到中间点，超时后只有
             * 当前反馈仍在软限位和保守区域内才允许继续；ArmMoveLinear会从
             * 当前真实姿态重新预检整条路径，因此不会沿用理想中间点硬走。
             */
            if (staging_reached ||
                (staging_timeout &&
                 ArmJointPoseWithinSoftLimits(g_arm_state.q_feedback_deg) &&
                 ArmAutoPoseIsSafe(g_arm_state.q_feedback_deg))) {
                Arm_Cartesian_Command_s pending_command =
                    arm_cartesian_runtime.pending_cartesian;
                Arm_Motion_Result_e pending_result;

                arm_cartesian_runtime.pending_cartesian_valid = 0u;
                if (pending_command.move_type == ARM_MOVE_DIRECT) {
                    pending_result = ArmSetCartesianTarget(
                        &pending_command.target_mm, NULL);
                } else {
                    pending_result = ArmMoveLinear(
                        &pending_command.target_mm,
                        pending_command.max_speed_mm_s);
                }
                if (pending_result != ARM_MOTION_RESULT_OK) {
                    /* 拒绝新目标时保持安全中间姿态，绝不失能电机。 */
                    ArmCartesianSetState(ARM_MOTION_HOLDING, now_ms);
                }
            } else if (staging_timeout) {
                arm_cartesian_runtime.pending_cartesian_valid = 0u;
                ArmCartesianRecordRejected(ARM_IK_COLLISION_RISK);
                ArmCartesianSetState(ARM_MOTION_HOLDING, now_ms);
            }
            break;
        }

        case ARM_MOTION_HOLDING:
#if ARM_BOOT_MODE == ARM_BOOT_MODE_AUTO_TEST
            if ((uint32_t)(now_ms - arm_cartesian_runtime.state_start_tick) <
                ARM_LINEAR_HOLD_MS) {
                break;
            }
            next_index = g_arm_motion_debug.sequence_index + 1u;
            if (next_index >= 3u) {
                if (ARM_AUTO_TEST_LOOP == 0u) {
                    ArmCartesianSetState(ARM_MOTION_COMPLETE, now_ms);
                    break;
                }
                next_index = 0u;
            }
            result = ArmMoveLinear(&arm_cartesian_test_point[next_index],
                                   ARM_LINEAR_DEFAULT_SPEED_MM_S);
            if (result == ARM_MOTION_RESULT_OK) {
                g_arm_motion_debug.sequence_index = next_index;
            } else {
                arm_cartesian_runtime.state_start_tick = now_ms;
            }
#endif
            break;

        case ARM_MOTION_ABORTED:
        case ARM_MOTION_COMPLETE:
        case ARM_MOTION_ERROR_IK:
            break;

        default:
            /* A rejected command holds the last valid target and stays enabled. */
            ArmCartesianSetState(ARM_MOTION_HOLDING, now_ms);
            break;
    }
    ArmCartesianUpdateDebug();
}
