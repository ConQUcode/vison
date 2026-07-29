#include "arm_trajectory.h"

#include "arm_config.h"
#include "arm_internal.h"
#include "arm_kinematics.h"
#include "stm32f4xx_hal.h"
#include "math.h"
#include "string.h"

/*
 * 轨迹层只接收关节角或腕部轴心坐标，不直接决定电机是否初始化成功。
 * 空间直线在启动前按配置间隔完整预检并缓存IK解；执行时按配置周期
 * 根据当前笛卡尔插值点重新求IK，1ms任务在相邻两组合法解之间插值。
 * 预检采样密度不决定轨迹时间，时间参数由空间/关节速度及加速度共同决定。
 * 五次曲线10t^3-15t^4+6t^5保证起止速度和加速度均为0。
 */

#define ARM_LINEAR_MIN_DISTANCE_MM         0.01f
#define ARM_LINEAR_MIN_SPEED_MM_S          1.0f
#define ARM_LINEAR_MIN_ACCEL_MM_S2         1.0f
#define ARM_LINEAR_QUINTIC_PEAK_FACTOR     1.875f
#define ARM_LINEAR_QUINTIC_ACCEL_FACTOR    5.7735f

#define ARM_LINEAR_Q1_STEP_MAX_DEG         5.0f
#define ARM_LINEAR_Q2_STEP_MAX_DEG         2.0f
#define ARM_LINEAR_Q3_STEP_MAX_DEG         2.0f
#define ARM_COMPOSITE_JOINT_STEP_DEG        1.0f
#define ARM_SAMPLE_PROGRESS_EPSILON          0.000001f
#define ARM_COMPOSITE_BLEND_JOINT_INTERVALS 12u
#define ARM_COMPOSITE_BLEND_LINEAR_INTERVALS 12u

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
    uint8_t online_ik_valid;
    uint8_t realtime_active;
    uint8_t realtime_timed_out;
    Arm_Trajectory_Path_e path_type;
    Arm_Position_s start_position;
    Arm_Position_s target_position;
    Arm_Position_s realtime_reference_position;
    Arm_Position_s realtime_last_valid_position;
    Arm_Realtime_Cartesian_Target_s realtime_target;
    uint32_t last_ik_tick;
    uint32_t online_ik_interval_start_tick;
    uint32_t online_ik_interval_ms;
    uint32_t realtime_last_command_tick;
    float commanded_speed_mm_s;
    float commanded_accel_mm_s2;
    float path_length_mm;
    float online_ik_previous_q_deg[3];
    float online_ik_next_q_deg[3];
    float realtime_command_q_deg[3];
    float realtime_velocity_mm_s[3];
    float realtime_last_velocity_mm_s[3];
    float initial_target_error_deg[3];
    uint8_t target_error_valid[3];
    uint8_t target_crossed[3];
    float sample_q_deg[ARM_LINEAR_MAX_SAMPLES][3];
    float sample_progress[ARM_LINEAR_MAX_SAMPLES];
} Arm_Cartesian_Runtime_s;

Arm_Motion_Debug_s g_arm_motion_debug;
static Arm_Cartesian_Runtime_s arm_cartesian_runtime;

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

static float ArmCartesianQuinticVelocity(float normalized_time)
{
    float t = ArmCartesianClamp(normalized_time, 0.0f, 1.0f);
    float one_minus_t = 1.0f - t;

    return 30.0f * t * t * one_minus_t * one_minus_t;
}

static float ArmCartesianQuinticAcceleration(float normalized_time)
{
    float t = ArmCartesianClamp(normalized_time, 0.0f, 1.0f);

    return 60.0f * t * (1.0f - t) * (1.0f - 2.0f * t);
}

static uint8_t ArmCartesianMotorsReady(void)
{
    uint8_t control_mode_ready = g_arm_state.mode == ARM_MODE_READY;

#if ARM_BOOT_MODE == ARM_BOOT_MODE_DM_SINGLE_AXIS_TEST
    control_mode_ready = control_mode_ready ||
        g_arm_state.mode == ARM_MODE_DM_SINGLE_AXIS_TEST;
#endif
    return g_arm_state.config_valid && g_arm_state.kinematics_valid &&
           g_arm_state.all_targets_synced &&
           g_arm_state.start_state == ARM_START_READY &&
           control_mode_ready &&
           g_arm_state.fault_latched == ARM_FAULT_NONE &&
           g_arm_state.motor_online[0] && g_arm_state.motor_online[1] &&
           g_arm_state.motor_online[2];
}

static void ArmCartesianSetState(Arm_Motion_State_e state, uint32_t now_ms)
{
    g_arm_motion_debug.motion_state = state;
    arm_cartesian_runtime.state_start_tick = now_ms;
}

static void ArmCartesianResetControlStatistics(void)
{
    memset(g_arm_control_debug.peak_tracking_error_deg, 0,
           sizeof(g_arm_control_debug.peak_tracking_error_deg));
    memset(g_arm_control_debug.overshoot_deg, 0,
           sizeof(g_arm_control_debug.overshoot_deg));
    memset(g_arm_control_debug.tracking_error_warning, 0,
           sizeof(g_arm_control_debug.tracking_error_warning));
    memset(g_arm_control_debug.tracking_error_duration_ms, 0,
           sizeof(g_arm_control_debug.tracking_error_duration_ms));
    memset(arm_cartesian_runtime.target_error_valid, 0,
           sizeof(arm_cartesian_runtime.target_error_valid));
    memset(arm_cartesian_runtime.target_crossed, 0,
           sizeof(arm_cartesian_runtime.target_crossed));
}

static void ArmCartesianUpdateControlDebug(uint32_t now_ms,
                                           uint32_t task_delta_ms)
{
    float duration_s =
        (float)arm_cartesian_runtime.trajectory_duration_ms * 0.001f;
    float normalized_time = duration_s > 0.0f ?
        (float)g_arm_motion_debug.trajectory_elapsed_ms /
            (float)arm_cartesian_runtime.trajectory_duration_ms : 1.0f;

    memcpy(g_arm_control_debug.current_q_deg, g_arm_state.q_feedback_deg,
           sizeof(g_arm_control_debug.current_q_deg));
    memcpy(g_arm_control_debug.reference_q_deg,
           g_arm_motion_debug.trajectory_q_deg,
           sizeof(g_arm_control_debug.reference_q_deg));
    memcpy(g_arm_control_debug.target_q_deg,
           g_arm_motion_debug.target_q_deg,
           sizeof(g_arm_control_debug.target_q_deg));
    memcpy(g_arm_control_debug.motor_speed_deg_s,
           g_arm_state.motor_speed_dps,
           sizeof(g_arm_control_debug.motor_speed_deg_s));
    memcpy(g_arm_control_debug.motor_current, g_arm_state.motor_current,
           sizeof(g_arm_control_debug.motor_current));

    for (uint8_t joint = 0u; joint < 3u; ++joint) {
        float tracking_error = joint == 0u ?
            ArmCartesianWrapTo180(
                g_arm_motion_debug.trajectory_q_deg[joint] -
                g_arm_state.q_feedback_deg[joint]) :
            g_arm_motion_debug.trajectory_q_deg[joint] -
                g_arm_state.q_feedback_deg[joint];
        float target_error = joint == 0u ?
            ArmCartesianWrapTo180(g_arm_motion_debug.target_q_deg[joint] -
                                  g_arm_state.q_feedback_deg[joint]) :
            g_arm_motion_debug.target_q_deg[joint] -
                g_arm_state.q_feedback_deg[joint];

        g_arm_control_debug.tracking_error_deg[joint] = tracking_error;
        if (fabsf(tracking_error) >
            g_arm_control_debug.peak_tracking_error_deg[joint]) {
            g_arm_control_debug.peak_tracking_error_deg[joint] =
                fabsf(tracking_error);
        }
        if (!arm_cartesian_runtime.target_error_valid[joint] &&
            fabsf(target_error) > 0.01f) {
            arm_cartesian_runtime.initial_target_error_deg[joint] =
                target_error;
            arm_cartesian_runtime.target_error_valid[joint] = 1u;
        }
        if (arm_cartesian_runtime.target_error_valid[joint] &&
            arm_cartesian_runtime.initial_target_error_deg[joint] *
                target_error < 0.0f) {
            arm_cartesian_runtime.target_crossed[joint] = 1u;
        }
        if (arm_cartesian_runtime.target_crossed[joint] &&
            fabsf(target_error) >
                g_arm_control_debug.overshoot_deg[joint]) {
            g_arm_control_debug.overshoot_deg[joint] =
                fabsf(target_error);
        }
        if (fabsf(tracking_error) > ARM_TRACKING_ERROR_WARN_DEG) {
            g_arm_control_debug.tracking_error_warning[joint] = 1u;
            g_arm_control_debug.tracking_error_duration_ms[joint] +=
                task_delta_ms;
        } else {
            g_arm_control_debug.tracking_error_warning[joint] = 0u;
            g_arm_control_debug.tracking_error_duration_ms[joint] = 0u;
        }
    }

    if (arm_cartesian_runtime.realtime_active) {
        float dv[3];
        float dt_s = (float)task_delta_ms * 0.001f;

        g_arm_control_debug.trajectory_speed_mm_s = sqrtf(
            arm_cartesian_runtime.realtime_velocity_mm_s[0] *
                arm_cartesian_runtime.realtime_velocity_mm_s[0] +
            arm_cartesian_runtime.realtime_velocity_mm_s[1] *
                arm_cartesian_runtime.realtime_velocity_mm_s[1] +
            arm_cartesian_runtime.realtime_velocity_mm_s[2] *
                arm_cartesian_runtime.realtime_velocity_mm_s[2]);
        dv[0] = arm_cartesian_runtime.realtime_velocity_mm_s[0] -
                arm_cartesian_runtime.realtime_last_velocity_mm_s[0];
        dv[1] = arm_cartesian_runtime.realtime_velocity_mm_s[1] -
                arm_cartesian_runtime.realtime_last_velocity_mm_s[1];
        dv[2] = arm_cartesian_runtime.realtime_velocity_mm_s[2] -
                arm_cartesian_runtime.realtime_last_velocity_mm_s[2];
        g_arm_control_debug.trajectory_acceleration_mm_s2 = dt_s > 0.0f ?
            sqrtf(dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]) /
                dt_s : 0.0f;
    } else {
        g_arm_control_debug.trajectory_speed_mm_s =
            duration_s > 0.0f ?
            arm_cartesian_runtime.path_length_mm *
                ArmCartesianQuinticVelocity(normalized_time) /
                duration_s : 0.0f;
        g_arm_control_debug.trajectory_acceleration_mm_s2 =
            duration_s > 0.0f ?
            arm_cartesian_runtime.path_length_mm *
                fabsf(ArmCartesianQuinticAcceleration(normalized_time)) /
                (duration_s * duration_s) : 0.0f;
    }
    g_arm_control_debug.trajectory_elapsed_ms =
        g_arm_motion_debug.trajectory_elapsed_ms;
    g_arm_control_debug.trajectory_duration_ms =
        arm_cartesian_runtime.trajectory_duration_ms;
    g_arm_control_debug.settling_time_ms =
        g_arm_motion_debug.motion_state == ARM_MOTION_SETTLING ?
        (uint32_t)(now_ms - arm_cartesian_runtime.state_start_tick) : 0u;
    g_arm_control_debug.realtime_active =
        arm_cartesian_runtime.realtime_active;
    g_arm_control_debug.realtime_timed_out =
        arm_cartesian_runtime.realtime_timed_out;
    g_arm_control_debug.realtime_command_id =
        arm_cartesian_runtime.realtime_target.command_id;
    g_arm_control_debug.command_age_ms =
        arm_cartesian_runtime.realtime_active ?
        (uint32_t)(now_ms -
                   arm_cartesian_runtime.realtime_last_command_tick) : 0u;
    ArmUpdateControllerDebugSnapshot(&g_arm_control_debug);
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

static float ArmCartesianBuildJointSampleProgress(uint16_t sample_count)
{
    const float joint_speed_deg_s[3] = {
        ARM_LINEAR_Q1_MAX_SPEED_DEG_S,
        ARM_LINEAR_Q2_MAX_SPEED_DEG_S,
        ARM_LINEAR_Q3_MAX_SPEED_DEG_S,
    };
    float total_minimum_time_s = 0.0f;

    if (sample_count == 0u || sample_count > ARM_LINEAR_MAX_SAMPLES) {
        return 0.0f;
    }
    arm_cartesian_runtime.sample_progress[0] = 0.0f;
    for (uint16_t i = 1u; i < sample_count; ++i) {
        float segment_minimum_time_s = 0.0f;
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
            segment_minimum_time_s = fmaxf(segment_minimum_time_s,
                segment_delta_deg[joint] / joint_speed_deg_s[joint]);
        }

        /* 保证进度严格递增，允许缓存中出现重合的安全样本。 */
        if (segment_minimum_time_s < ARM_SAMPLE_PROGRESS_EPSILON) {
            segment_minimum_time_s = ARM_SAMPLE_PROGRESS_EPSILON;
        }
        total_minimum_time_s += segment_minimum_time_s;
        arm_cartesian_runtime.sample_progress[i] = total_minimum_time_s;
    }

    if (sample_count == 1u ||
        total_minimum_time_s < ARM_SAMPLE_PROGRESS_EPSILON) {
        arm_cartesian_runtime.sample_progress[0] = 0.0f;
        return 0.0f;
    }
    for (uint16_t i = 1u; i < sample_count; ++i) {
        arm_cartesian_runtime.sample_progress[i] /= total_minimum_time_s;
    }
    arm_cartesian_runtime.sample_progress[sample_count - 1u] = 1.0f;
    return total_minimum_time_s;
}

static uint8_t ArmCartesianBlendCompositeWaypoint(
    uint16_t waypoint_index,
    uint16_t sample_count)
{
    uint16_t before_count = ARM_COMPOSITE_BLEND_JOINT_INTERVALS;
    uint16_t after_count = ARM_COMPOSITE_BLEND_LINEAR_INTERVALS;
    uint16_t first_index;
    uint16_t last_index;
    uint16_t blend_interval_count;
    float control_q_deg[4][3];

    if (waypoint_index == 0u || waypoint_index + 1u >= sample_count) {
        return 1u;
    }
    if (before_count > waypoint_index) {
        before_count = waypoint_index;
    }
    if (after_count > sample_count - 1u - waypoint_index) {
        after_count = (uint16_t)(sample_count - 1u - waypoint_index);
    }
    if (before_count >= waypoint_index) {
        before_count = (uint16_t)(waypoint_index - 1u);
    }
    if (after_count >= sample_count - 1u - waypoint_index) {
        after_count = (uint16_t)(sample_count - 2u - waypoint_index);
    }
    if (before_count == 0u || after_count == 0u) {
        return 1u;
    }

    first_index = (uint16_t)(waypoint_index - before_count);
    last_index = (uint16_t)(waypoint_index + after_count);
    blend_interval_count = (uint16_t)(last_index - first_index);
    memcpy(control_q_deg[0], arm_cartesian_runtime.sample_q_deg[first_index],
           sizeof(control_q_deg[0]));
    memcpy(control_q_deg[3], arm_cartesian_runtime.sample_q_deg[last_index],
           sizeof(control_q_deg[3]));
    for (uint8_t joint = 0u; joint < 3u; ++joint) {
        float incoming_step_deg = joint == ARM_JOINT_BASE_YAW ?
            ArmCartesianWrapTo180(
                arm_cartesian_runtime.sample_q_deg[first_index][joint] -
                arm_cartesian_runtime.sample_q_deg[first_index - 1u][joint]) :
            arm_cartesian_runtime.sample_q_deg[first_index][joint] -
                arm_cartesian_runtime.sample_q_deg[first_index - 1u][joint];
        float outgoing_step_deg = joint == ARM_JOINT_BASE_YAW ?
            ArmCartesianWrapTo180(
                arm_cartesian_runtime.sample_q_deg[last_index + 1u][joint] -
                arm_cartesian_runtime.sample_q_deg[last_index][joint]) :
            arm_cartesian_runtime.sample_q_deg[last_index + 1u][joint] -
                arm_cartesian_runtime.sample_q_deg[last_index][joint];

        control_q_deg[1][joint] = control_q_deg[0][joint] +
            incoming_step_deg * (float)blend_interval_count / 3.0f;
        control_q_deg[2][joint] = control_q_deg[3][joint] -
            outgoing_step_deg * (float)blend_interval_count / 3.0f;
    }

    for (uint16_t i = first_index; i <= last_index; ++i) {
        float ratio = (float)(i - first_index) /
            (float)blend_interval_count;
        float one_minus_ratio = 1.0f - ratio;
        float one_minus_ratio_2 = one_minus_ratio * one_minus_ratio;
        float ratio_2 = ratio * ratio;
        float blended_q_deg[3];

        for (uint8_t joint = 0u; joint < 3u; ++joint) {
            blended_q_deg[joint] =
                one_minus_ratio_2 * one_minus_ratio *
                    control_q_deg[0][joint] +
                3.0f * one_minus_ratio_2 * ratio *
                    control_q_deg[1][joint] +
                3.0f * one_minus_ratio * ratio_2 *
                    control_q_deg[2][joint] +
                ratio_2 * ratio * control_q_deg[3][joint];
        }
        blended_q_deg[ARM_JOINT_BASE_YAW] = ArmCartesianWrapTo180(
            blended_q_deg[ARM_JOINT_BASE_YAW]);
        if (!ArmJointPoseWithinSoftLimits(blended_q_deg) ||
            !ArmAutoPoseIsSafe(blended_q_deg)) {
            return 0u;
        }
        memcpy(arm_cartesian_runtime.sample_q_deg[i], blended_q_deg,
               sizeof(blended_q_deg));
    }

    for (uint16_t i = first_index + 1u; i <= last_index + 1u; ++i) {
        if (!ArmCartesianJointStepContinuous(
                arm_cartesian_runtime.sample_q_deg[i - 1u],
                arm_cartesian_runtime.sample_q_deg[i])) {
            return 0u;
        }
    }
    return 1u;
}

static uint32_t ArmCartesianDurationMs(float path_length_mm,
                                       float max_speed_mm_s,
                                       float max_accel_mm_s2,
                                       uint16_t sample_count)
{
    /*
     * 五次曲线的峰值速度系数为1.875，峰值加速度系数为5.7735。
     * 先满足笛卡尔速度/加速度，再按整段及局部最陡IK变化延长时间。
     */
    float duration_s = 0.0f;
    float joint_travel_deg[3] = {0.0f, 0.0f, 0.0f};
    float joint_path_minimum_time_s =
        ArmCartesianBuildJointSampleProgress(sample_count);
    const float joint_speed_deg_s[3] = {
        ARM_LINEAR_Q1_MAX_SPEED_DEG_S,
        ARM_LINEAR_Q2_MAX_SPEED_DEG_S,
        ARM_LINEAR_Q3_MAX_SPEED_DEG_S,
    };
    const float joint_accel_deg_s2[3] = {
        ARM_LINEAR_Q1_MAX_ACCEL_DEG_S2,
        ARM_LINEAR_Q2_MAX_ACCEL_DEG_S2,
        ARM_LINEAR_Q3_MAX_ACCEL_DEG_S2,
    };

    if (path_length_mm > ARM_LINEAR_MIN_DISTANCE_MM) {
        float speed_duration_s = ARM_LINEAR_QUINTIC_PEAK_FACTOR *
            path_length_mm / max_speed_mm_s;
        float accel_duration_s = sqrtf(ARM_LINEAR_QUINTIC_ACCEL_FACTOR *
            path_length_mm / max_accel_mm_s2);

        duration_s = fmaxf(speed_duration_s, accel_duration_s);
    }
    duration_s = fmaxf(duration_s,
        ARM_LINEAR_QUINTIC_PEAK_FACTOR * joint_path_minimum_time_s);

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
        float speed_duration_s = ARM_LINEAR_QUINTIC_PEAK_FACTOR *
            joint_travel_deg[joint] / joint_speed_deg_s[joint];
        float accel_duration_s = sqrtf(ARM_LINEAR_QUINTIC_ACCEL_FACTOR *
            joint_travel_deg[joint] / joint_accel_deg_s2[joint]);

        duration_s = fmaxf(duration_s,
                           fmaxf(speed_duration_s, accel_duration_s));
    }
    for (uint16_t i = 1u; i < sample_count; ++i) {
        float segment_delta_deg[3];
        float segment_progress =
            arm_cartesian_runtime.sample_progress[i] -
            arm_cartesian_runtime.sample_progress[i - 1u];

        if (segment_progress < ARM_SAMPLE_PROGRESS_EPSILON) {
            continue;
        }

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
            float segment_speed_duration_s = ARM_LINEAR_QUINTIC_PEAK_FACTOR *
                segment_delta_deg[joint] /
                (segment_progress * joint_speed_deg_s[joint]);
            float segment_accel_duration_s = sqrtf(
                ARM_LINEAR_QUINTIC_ACCEL_FACTOR *
                segment_delta_deg[joint] *
                1.0f /
                (segment_progress * joint_accel_deg_s2[joint]));

            duration_s = fmaxf(duration_s,
                fmaxf(segment_speed_duration_s,
                      segment_accel_duration_s));
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
    float clamped_progress;
    float progress_span;
    float sample_fraction;
    uint16_t lower_index;
    uint16_t upper_index;

    if (arm_cartesian_runtime.sample_count <= 1u) {
        memcpy(reference_q_deg, arm_cartesian_runtime.sample_q_deg[0],
               sizeof(float) * 3u);
        return;
    }
    clamped_progress = ArmCartesianClamp(progress, 0.0f, 1.0f);
    if (clamped_progress >= 1.0f) {
        lower_index = arm_cartesian_runtime.sample_count - 1u;
        upper_index = lower_index;
        sample_fraction = 0.0f;
    } else {
        uint16_t search_low = 0u;
        uint16_t search_high = arm_cartesian_runtime.sample_count - 1u;

        while ((uint16_t)(search_low + 1u) < search_high) {
            uint16_t middle = (uint16_t)(search_low +
                (search_high - search_low) / 2u);

            if (arm_cartesian_runtime.sample_progress[middle] <=
                clamped_progress) {
                search_low = middle;
            } else {
                search_high = middle;
            }
        }
        lower_index = search_low;
        upper_index = search_high;
        progress_span =
            arm_cartesian_runtime.sample_progress[upper_index] -
            arm_cartesian_runtime.sample_progress[lower_index];
        sample_fraction = progress_span > ARM_SAMPLE_PROGRESS_EPSILON ?
            (clamped_progress -
             arm_cartesian_runtime.sample_progress[lower_index]) /
                progress_span : 0.0f;
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

static void ArmCartesianPositionAtProgress(float progress,
                                           Arm_Position_s *position)
{
    position->x_mm = arm_cartesian_runtime.start_position.x_mm +
        progress * (arm_cartesian_runtime.target_position.x_mm -
                    arm_cartesian_runtime.start_position.x_mm);
    position->y_mm = arm_cartesian_runtime.start_position.y_mm +
        progress * (arm_cartesian_runtime.target_position.y_mm -
                    arm_cartesian_runtime.start_position.y_mm);
    position->z_mm = arm_cartesian_runtime.start_position.z_mm +
        progress * (arm_cartesian_runtime.target_position.z_mm -
                    arm_cartesian_runtime.start_position.z_mm);
}

static uint8_t ArmCartesianSolveOnlineIK(uint32_t now_ms,
                                        float progress)
{
    Arm_Position_s sample_position;
    Arm_IK_Result_s result;
    float seed_q_deg[3];

    ArmCartesianPositionAtProgress(progress, &sample_position);
    if (arm_cartesian_runtime.online_ik_valid) {
        memcpy(seed_q_deg, arm_cartesian_runtime.online_ik_next_q_deg,
               sizeof(seed_q_deg));
        memcpy(arm_cartesian_runtime.online_ik_previous_q_deg,
               arm_cartesian_runtime.online_ik_next_q_deg,
               sizeof(arm_cartesian_runtime.online_ik_previous_q_deg));
    } else {
        memcpy(seed_q_deg, g_arm_motion_debug.trajectory_q_deg,
               sizeof(seed_q_deg));
        memcpy(arm_cartesian_runtime.online_ik_previous_q_deg, seed_q_deg,
               sizeof(seed_q_deg));
    }
    memset(&result, 0, sizeof(result));
    if (ArmInverseKinematics3DOF(&sample_position, seed_q_deg, &result) !=
            ARM_IK_OK ||
        result.position_error_mm > ARM_LINEAR_FK_ERROR_MAX_MM ||
        !ArmJointPoseWithinSoftLimits(result.q_deg) ||
        !ArmAutoPoseIsSafe(result.q_deg) ||
        !ArmCartesianJointStepContinuous(seed_q_deg, result.q_deg)) {
        g_arm_motion_debug.ik_status = result.status;
        g_arm_motion_debug.command_accepted = 0u;
        if (!arm_cartesian_runtime.reference_update_rejected) {
            g_arm_motion_debug.command_reject_count++;
            arm_cartesian_runtime.reference_update_rejected = 1u;
        }
        return 0u;
    }

    memcpy(arm_cartesian_runtime.online_ik_next_q_deg, result.q_deg,
           sizeof(result.q_deg));
    arm_cartesian_runtime.online_ik_interval_start_tick = now_ms;
    arm_cartesian_runtime.online_ik_interval_ms = ARM_LINEAR_IK_UPDATE_MS;
    arm_cartesian_runtime.last_ik_tick = now_ms;
    arm_cartesian_runtime.online_ik_valid = 1u;
    arm_cartesian_runtime.reference_update_rejected = 0u;
    g_arm_motion_debug.ik_status = ARM_IK_OK;
    g_arm_motion_debug.command_accepted = 1u;
    return 1u;
}

static void ArmCartesianInterpolateOnlineIK(uint32_t now_ms,
                                            float reference_q_deg[3])
{
    float ratio = arm_cartesian_runtime.online_ik_interval_ms > 0u ?
        (float)(now_ms -
                arm_cartesian_runtime.online_ik_interval_start_tick) /
            (float)arm_cartesian_runtime.online_ik_interval_ms : 1.0f;

    ratio = ArmCartesianClamp(ratio, 0.0f, 1.0f);
    reference_q_deg[0] = ArmCartesianWrapTo180(
        arm_cartesian_runtime.online_ik_previous_q_deg[0] +
        ratio * ArmCartesianWrapTo180(
            arm_cartesian_runtime.online_ik_next_q_deg[0] -
            arm_cartesian_runtime.online_ik_previous_q_deg[0]));
    reference_q_deg[1] =
        arm_cartesian_runtime.online_ik_previous_q_deg[1] +
        ratio * (arm_cartesian_runtime.online_ik_next_q_deg[1] -
                 arm_cartesian_runtime.online_ik_previous_q_deg[1]);
    reference_q_deg[2] =
        arm_cartesian_runtime.online_ik_previous_q_deg[2] +
        ratio * (arm_cartesian_runtime.online_ik_next_q_deg[2] -
                 arm_cartesian_runtime.online_ik_previous_q_deg[2]);
}

static void ArmCartesianStartPreparedTrajectory(
    const Arm_Position_s *target,
    uint32_t duration_ms,
    float commanded_speed_mm_s,
    float commanded_accel_mm_s2,
    Arm_Trajectory_Path_e path_type,
    Arm_Motion_State_e motion_state,
    uint32_t now_ms)
{
    uint16_t last_index = arm_cartesian_runtime.sample_count - 1u;

    arm_cartesian_runtime.target_position = *target;
    arm_cartesian_runtime.trajectory_duration_ms = duration_ms;
    arm_cartesian_runtime.trajectory_start_tick = now_ms;
    arm_cartesian_runtime.commanded_speed_mm_s = commanded_speed_mm_s;
    arm_cartesian_runtime.commanded_accel_mm_s2 = commanded_accel_mm_s2;
    arm_cartesian_runtime.path_type = path_type;
    arm_cartesian_runtime.reference_update_rejected = 0u;
    arm_cartesian_runtime.online_ik_valid = 0u;
    arm_cartesian_runtime.last_ik_tick = now_ms;
    arm_cartesian_runtime.online_ik_interval_start_tick = now_ms;
    arm_cartesian_runtime.online_ik_interval_ms = ARM_LINEAR_IK_UPDATE_MS;
    arm_cartesian_runtime.realtime_active = 0u;
    arm_cartesian_runtime.realtime_timed_out = 0u;
    arm_cartesian_runtime.path_length_mm = ArmCartesianPositionDistance(
        &arm_cartesian_runtime.start_position, target);
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
    ArmCartesianResetControlStatistics();
    ArmCartesianSetState(motion_state, now_ms);
}

void ArmAbortMotion(Arm_Motion_Fault_e reason)
{
    g_arm_motion_debug.fault_code = reason;
    g_arm_motion_debug.motion_state = ARM_MOTION_ABORTED;
    g_arm_motion_debug.command_accepted = 0u;
    arm_cartesian_runtime.realtime_active = 0u;
    arm_cartesian_runtime.realtime_timed_out = 0u;
    arm_cartesian_runtime.online_ik_valid = 0u;
    arm_cartesian_runtime.sample_count = 0u;
    memset(arm_cartesian_runtime.realtime_velocity_mm_s, 0,
           sizeof(arm_cartesian_runtime.realtime_velocity_mm_s));
}

uint8_t ArmTrajectoryMotorHoldAllowed(void)
{
    return g_arm_motion_debug.motion_state != ARM_MOTION_ABORTED;
}

uint8_t ArmTrajectoryOwnsControl(void)
{
    return arm_cartesian_runtime.realtime_active ||
           g_arm_motion_debug.motion_state == ARM_MOTION_STAGING ||
           g_arm_motion_debug.motion_state == ARM_MOTION_RUNNING ||
           g_arm_motion_debug.motion_state == ARM_MOTION_SETTLING ||
           g_arm_motion_debug.motion_state == ARM_MOTION_HOLDING;
}

uint8_t ArmTrajectoryIsBusy(void)
{
    return arm_cartesian_runtime.realtime_active ||
           g_arm_motion_debug.motion_state == ARM_MOTION_STAGING ||
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
    arm_cartesian_runtime.realtime_active = 0u;
    arm_cartesian_runtime.realtime_timed_out = 0u;
    memset(arm_cartesian_runtime.realtime_velocity_mm_s, 0,
           sizeof(arm_cartesian_runtime.realtime_velocity_mm_s));
    arm_cartesian_runtime.sample_count = 1u;
    memcpy(arm_cartesian_runtime.sample_q_deg[0], hold_q_deg,
           sizeof(hold_q_deg));
    ArmCartesianSetState(ARM_MOTION_HOLDING, HAL_GetTick());
}

static float ArmCartesianVectorLength(const float vector[3])
{
    return sqrtf(vector[0] * vector[0] + vector[1] * vector[1] +
                 vector[2] * vector[2]);
}

static void ArmCartesianLimitVector(float vector[3], float max_length)
{
    float length = ArmCartesianVectorLength(vector);

    if (length > max_length && length > 0.000001f) {
        float scale = max_length / length;

        vector[0] *= scale;
        vector[1] *= scale;
        vector[2] *= scale;
    }
}

static void ArmCartesianSetVectorLength(float vector[3], float length)
{
    float current_length = ArmCartesianVectorLength(vector);

    if (current_length <= 0.000001f || length <= 0.0f) {
        vector[0] = 0.0f;
        vector[1] = 0.0f;
        vector[2] = 0.0f;
        return;
    }
    vector[0] *= length / current_length;
    vector[1] *= length / current_length;
    vector[2] *= length / current_length;
}

Arm_Command_Result_e ArmTrajectorySubmitRealtimeTarget(
    const Arm_Realtime_Cartesian_Target_s *target)
{
    Arm_IK_Result_s result;
    float seed_q_deg[3];
    float speed_mm_s;
    float accel_mm_s2;
    uint32_t now_ms = HAL_GetTick();

    if (target == NULL || !isfinite(target->target_mm.x_mm) ||
        !isfinite(target->target_mm.y_mm) ||
        !isfinite(target->target_mm.z_mm) ||
        !isfinite(target->max_speed_mm_s) ||
        !isfinite(target->max_acceleration_mm_s2) ||
        target->max_speed_mm_s < 0.0f ||
        target->max_acceleration_mm_s2 < 0.0f) {
        return ARM_COMMAND_INVALID;
    }
    if (!ArmCartesianMotorsReady()) {
        return ARM_COMMAND_NOT_READY;
    }

    speed_mm_s = target->max_speed_mm_s > 0.0f ?
        target->max_speed_mm_s : ARM_LINEAR_DEFAULT_SPEED_MM_S;
    accel_mm_s2 = target->max_acceleration_mm_s2 > 0.0f ?
        target->max_acceleration_mm_s2 :
        ARM_REALTIME_DEFAULT_ACCEL_MM_S2;
    if (speed_mm_s < ARM_LINEAR_MIN_SPEED_MM_S ||
        speed_mm_s > ARM_LINEAR_MAX_SPEED_MM_S ||
        accel_mm_s2 < ARM_LINEAR_MIN_ACCEL_MM_S2 ||
        accel_mm_s2 > ARM_LINEAR_MAX_ACCEL_MM_S2) {
        return ARM_COMMAND_INVALID;
    }
    if (!ArmJointPoseWithinSoftLimits(g_arm_state.q_feedback_deg) ||
        !ArmAutoPoseIsSafe(g_arm_state.q_feedback_deg)) {
        return ARM_COMMAND_NOT_READY;
    }

    if (arm_cartesian_runtime.realtime_active) {
        memcpy(seed_q_deg, arm_cartesian_runtime.realtime_command_q_deg,
               sizeof(seed_q_deg));
    } else {
        memcpy(seed_q_deg, g_arm_state.q_feedback_deg, sizeof(seed_q_deg));
    }
    memset(&result, 0, sizeof(result));
    g_arm_motion_debug.realtime_reject_reason =
        ARM_REALTIME_REJECT_NONE;
    if (ArmInverseKinematics3DOF(&target->target_mm, seed_q_deg, &result) !=
        ARM_IK_OK) {
        g_arm_motion_debug.realtime_reject_reason =
            ARM_REALTIME_REJECT_IK;
    } else if (result.position_error_mm > ARM_LINEAR_FK_ERROR_MAX_MM) {
        g_arm_motion_debug.realtime_reject_reason =
            ARM_REALTIME_REJECT_FK_ERROR;
    } else if (!ArmJointPoseWithinSoftLimits(result.q_deg)) {
        g_arm_motion_debug.realtime_reject_reason =
            ARM_REALTIME_REJECT_SOFT_LIMIT;
    } else if (!ArmAutoPoseIsSafe(result.q_deg)) {
        g_arm_motion_debug.realtime_reject_reason =
            ARM_REALTIME_REJECT_AUTO_REGION;
    }
    if (g_arm_motion_debug.realtime_reject_reason !=
        ARM_REALTIME_REJECT_NONE) {
        /* 非法目标不覆盖单槽缓存，也不触碰当前参考和使能状态。 */
        g_arm_motion_debug.last_realtime_reject_reason =
            g_arm_motion_debug.realtime_reject_reason;
        ArmCartesianRecordRejected(result.status);
        return ARM_COMMAND_PREFLIGHT_FAILED;
    }

    if (!arm_cartesian_runtime.realtime_active) {
        if (!ArmBeginJointMove(result.q_deg)) {
            return ARM_COMMAND_NOT_READY;
        }
        arm_cartesian_runtime.realtime_reference_position =
            g_arm_state.wrist_center;
        arm_cartesian_runtime.realtime_last_valid_position =
            g_arm_state.wrist_center;
        arm_cartesian_runtime.start_position = g_arm_state.wrist_center;
        memset(arm_cartesian_runtime.realtime_velocity_mm_s, 0,
               sizeof(arm_cartesian_runtime.realtime_velocity_mm_s));
        memset(arm_cartesian_runtime.realtime_last_velocity_mm_s, 0,
               sizeof(arm_cartesian_runtime.realtime_last_velocity_mm_s));
        memcpy(arm_cartesian_runtime.online_ik_previous_q_deg, seed_q_deg,
               sizeof(seed_q_deg));
        memcpy(arm_cartesian_runtime.online_ik_next_q_deg, seed_q_deg,
               sizeof(seed_q_deg));
        memcpy(arm_cartesian_runtime.realtime_command_q_deg, result.q_deg,
               sizeof(result.q_deg));
        arm_cartesian_runtime.online_ik_valid = 1u;
        arm_cartesian_runtime.last_ik_tick = now_ms;
        arm_cartesian_runtime.online_ik_interval_start_tick = now_ms;
        arm_cartesian_runtime.online_ik_interval_ms =
            ARM_LINEAR_IK_UPDATE_MS;
        arm_cartesian_runtime.realtime_active = 1u;
        arm_cartesian_runtime.trajectory_start_tick = now_ms;
        ArmCartesianResetControlStatistics();
    }

    arm_cartesian_runtime.realtime_target = *target;
    memcpy(arm_cartesian_runtime.realtime_command_q_deg, result.q_deg,
           sizeof(result.q_deg));
    arm_cartesian_runtime.realtime_target.max_speed_mm_s = speed_mm_s;
    arm_cartesian_runtime.realtime_target.max_acceleration_mm_s2 =
        accel_mm_s2;
    arm_cartesian_runtime.realtime_last_command_tick = now_ms;
    arm_cartesian_runtime.realtime_timed_out = 0u;
    arm_cartesian_runtime.commanded_speed_mm_s = speed_mm_s;
    arm_cartesian_runtime.commanded_accel_mm_s2 = accel_mm_s2;
    arm_cartesian_runtime.target_position = target->target_mm;
    arm_cartesian_runtime.path_length_mm = ArmCartesianPositionDistance(
        &arm_cartesian_runtime.realtime_reference_position,
        &target->target_mm);
    memcpy(g_arm_motion_debug.target_q_deg, result.q_deg,
           sizeof(result.q_deg));
    g_arm_motion_debug.target_position_mm = target->target_mm;
    g_arm_motion_debug.ik_status = ARM_IK_OK;
    g_arm_motion_debug.command_accepted = 1u;
    g_arm_motion_debug.path_preflight_passed = 1u;
    g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_NONE;
    ArmCartesianSetState(ARM_MOTION_RUNNING, now_ms);
    return ARM_COMMAND_OK;
}

void ArmTrajectoryStopRealtime(void)
{
    if (!arm_cartesian_runtime.realtime_active) {
        return;
    }
    arm_cartesian_runtime.realtime_active = 0u;
    arm_cartesian_runtime.realtime_timed_out = 0u;
    memset(arm_cartesian_runtime.realtime_velocity_mm_s, 0,
           sizeof(arm_cartesian_runtime.realtime_velocity_mm_s));
    ArmTrajectoryCancel();
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
    arm_cartesian_runtime.start_position = g_arm_state.wrist_center;
    duration_ms = ArmCartesianDurationMs(0.0f,
        ARM_LINEAR_DEFAULT_SPEED_MM_S, ARM_LINEAR_MAX_ACCEL_MM_S2, 2u);
    if (!ArmBeginJointMove(target_q_deg)) {
        return ARM_MOTION_RESULT_NOT_READY;
    }
    ArmCartesianStartPreparedTrajectory(
        &target_position, duration_ms, ARM_LINEAR_DEFAULT_SPEED_MM_S,
        ARM_LINEAR_MAX_ACCEL_MM_S2, ARM_TRAJECTORY_PATH_JOINT_STAGING,
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
        max_speed_mm_s < ARM_LINEAR_MIN_SPEED_MM_S ||
        max_speed_mm_s > ARM_LINEAR_MAX_SPEED_MM_S) {
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
        ARM_LINEAR_MAX_ACCEL_MM_S2, sample_count);
    if (!ArmBeginJointMove(previous_q_deg)) {
        ArmCartesianRecordRejected(ARM_IK_INVALID_ARGUMENT);
        return ARM_MOTION_RESULT_NOT_READY;
    }
    ArmCartesianStartPreparedTrajectory(
        target, duration_ms, max_speed_mm_s, ARM_LINEAR_MAX_ACCEL_MM_S2,
        ARM_TRAJECTORY_PATH_CARTESIAN_LINEAR,
        ARM_MOTION_RUNNING, now_ms);
    return ARM_MOTION_RESULT_OK;
}

Arm_Motion_Result_e ArmTrajectoryMoveJointThenLinear(
    const float waypoint_q_deg[3],
    const Arm_Position_s *target,
    float max_speed_mm_s)
{
    Arm_IK_Result_s ik_result;
    Arm_Position_s waypoint_position;
    Arm_Position_s sample_position;
    float start_q_deg[3];
    float previous_q_deg[3];
    float max_joint_delta_deg = 0.0f;
    float cartesian_length_mm;
    float effective_length_mm;
    uint16_t joint_interval_count;
    uint16_t cartesian_interval_count;
    uint16_t total_sample_count;
    uint32_t duration_ms;
    uint32_t now_ms = HAL_GetTick();

    memset(&ik_result, 0, sizeof(ik_result));
    ik_result.status = ARM_IK_INVALID_ARGUMENT;
    if (waypoint_q_deg == NULL || target == NULL ||
        !isfinite(target->x_mm) || !isfinite(target->y_mm) ||
        !isfinite(target->z_mm) || !isfinite(max_speed_mm_s) ||
        max_speed_mm_s < ARM_LINEAR_MIN_SPEED_MM_S ||
        max_speed_mm_s > ARM_LINEAR_MAX_SPEED_MM_S) {
        ArmCartesianRecordRejected(ik_result.status);
        return ARM_MOTION_RESULT_INVALID;
    }
    if (ArmTrajectoryIsBusy()) {
        return ARM_MOTION_RESULT_BUSY;
    }
    if (!ArmCartesianMotorsReady()) {
        ArmCartesianRecordRejected(ik_result.status);
        return ARM_MOTION_RESULT_NOT_READY;
    }
    start_q_deg[0] = g_arm_state.q_feedback_deg[ARM_JOINT_BASE_YAW];
    start_q_deg[1] = g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER];
    start_q_deg[2] = g_arm_state.q_feedback_deg[ARM_JOINT_ELBOW];
    if (!ArmJointPoseWithinSoftLimits(start_q_deg) ||
        !ArmAutoPoseIsSafe(start_q_deg) ||
        !ArmJointPoseWithinSoftLimits(waypoint_q_deg) ||
        !ArmAutoPoseIsSafe(waypoint_q_deg)) {
        ArmCartesianRecordRejected(ARM_IK_COLLISION_RISK);
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }

    for (uint8_t joint = 0u; joint < 3u; ++joint) {
        float delta_deg = joint == ARM_JOINT_BASE_YAW ?
            fabsf(ArmCartesianWrapTo180(
                waypoint_q_deg[joint] - start_q_deg[joint])) :
            fabsf(waypoint_q_deg[joint] - start_q_deg[joint]);

        max_joint_delta_deg = fmaxf(max_joint_delta_deg, delta_deg);
    }
    joint_interval_count = (uint16_t)ceilf(
        max_joint_delta_deg / ARM_COMPOSITE_JOINT_STEP_DEG);
    if (joint_interval_count < 1u) {
        joint_interval_count = 1u;
    }

    ArmForwardKinematics3DOF(waypoint_q_deg[0], waypoint_q_deg[1],
                             waypoint_q_deg[2], &waypoint_position);
    cartesian_length_mm = ArmCartesianPositionDistance(&waypoint_position,
                                                       target);
    if (!isfinite(cartesian_length_mm) ||
        cartesian_length_mm <= ARM_LINEAR_MIN_DISTANCE_MM) {
        ArmCartesianRecordRejected(ARM_IK_INVALID_ARGUMENT);
        return ARM_MOTION_RESULT_INVALID;
    }
    cartesian_interval_count = (uint16_t)ceilf(
        cartesian_length_mm / ARM_LINEAR_SAMPLE_SPACING_MM);
    if (cartesian_interval_count < 1u) {
        cartesian_interval_count = 1u;
    }
    total_sample_count = (uint16_t)(1u + joint_interval_count +
                                    cartesian_interval_count);
    if (total_sample_count > ARM_LINEAR_MAX_SAMPLES) {
        ArmCartesianRecordRejected(ARM_IK_INVALID_ARGUMENT);
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }

    arm_cartesian_runtime.sample_count = total_sample_count;
    memcpy(arm_cartesian_runtime.sample_q_deg[0], start_q_deg,
           sizeof(start_q_deg));
    for (uint16_t i = 1u; i <= joint_interval_count; ++i) {
        float ratio = (float)i / (float)joint_interval_count;
        float *sample_q_deg = arm_cartesian_runtime.sample_q_deg[i];

        sample_q_deg[0] = ArmCartesianWrapTo180(start_q_deg[0] +
            ratio * ArmCartesianWrapTo180(waypoint_q_deg[0] -
                                           start_q_deg[0]));
        sample_q_deg[1] = start_q_deg[1] +
            ratio * (waypoint_q_deg[1] - start_q_deg[1]);
        sample_q_deg[2] = start_q_deg[2] +
            ratio * (waypoint_q_deg[2] - start_q_deg[2]);
        if (!ArmJointPoseWithinSoftLimits(sample_q_deg) ||
            !ArmAutoPoseIsSafe(sample_q_deg)) {
            ArmCartesianRecordRejected(ARM_IK_COLLISION_RISK);
            return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
        }
    }

    memcpy(previous_q_deg, waypoint_q_deg, sizeof(previous_q_deg));
    for (uint16_t i = 1u; i <= cartesian_interval_count; ++i) {
        float ratio = (float)i / (float)cartesian_interval_count;
        uint16_t sample_index = (uint16_t)(joint_interval_count + i);

        sample_position.x_mm = waypoint_position.x_mm +
            ratio * (target->x_mm - waypoint_position.x_mm);
        sample_position.y_mm = waypoint_position.y_mm +
            ratio * (target->y_mm - waypoint_position.y_mm);
        sample_position.z_mm = waypoint_position.z_mm +
            ratio * (target->z_mm - waypoint_position.z_mm);
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
        memcpy(arm_cartesian_runtime.sample_q_deg[sample_index],
               ik_result.q_deg, sizeof(ik_result.q_deg));
        memcpy(previous_q_deg, ik_result.q_deg, sizeof(previous_q_deg));
    }

    /*
     * 精确经过安全姿态会把“主要转大臂”和“主要展小臂”
     * 两个不同切向拼成尖角。用局部三次Bezier圆角替换尖角，
     * 保留安全区域的必经约束，但不强制电机在该点换向或降速。
     */
    if (!ArmCartesianBlendCompositeWaypoint(joint_interval_count,
                                             total_sample_count)) {
        ArmCartesianRecordRejected(ARM_IK_COLLISION_RISK);
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }

    /*
     * 两段样本共用一个五次时间轴。用等效路径长度补偿关节过渡
     * 占用的样本比例，保证后半段笛卡尔速度不超过设定值。
     */
    effective_length_mm = cartesian_length_mm *
        (float)(total_sample_count - 1u) /
        (float)cartesian_interval_count;
    duration_ms = ArmCartesianDurationMs(effective_length_mm,
        max_speed_mm_s, ARM_LINEAR_MAX_ACCEL_MM_S2, total_sample_count);
    if (!ArmBeginJointMove(previous_q_deg)) {
        ArmCartesianRecordRejected(ARM_IK_INVALID_ARGUMENT);
        return ARM_MOTION_RESULT_NOT_READY;
    }
    arm_cartesian_runtime.start_position = g_arm_state.wrist_center;
    ArmCartesianStartPreparedTrajectory(
        target, duration_ms, max_speed_mm_s, ARM_LINEAR_MAX_ACCEL_MM_S2,
        ARM_TRAJECTORY_PATH_JOINT_STAGING,
        ARM_MOTION_RUNNING, now_ms);
    return ARM_MOTION_RESULT_OK;
}

Arm_Motion_Result_e ArmTrajectoryStageCartesianCommand(
    const Arm_Cartesian_Command_s *command)
{
    if (command == NULL) {
        return ARM_MOTION_RESULT_INVALID;
    }
    if (ArmTrajectoryIsBusy()) {
        return ARM_MOTION_RESULT_BUSY;
    }
    if (!ArmCartesianMotorsReady()) {
        return ARM_MOTION_RESULT_NOT_READY;
    }
    if (command->move_type == ARM_MOVE_DIRECT) {
        return ArmSetCartesianTarget(&command->target_mm, NULL);
    }
    return ArmMoveLinear(&command->target_mm,
        command->max_speed_mm_s > 0.0f ? command->max_speed_mm_s :
                                         ARM_LINEAR_DEFAULT_SPEED_MM_S);
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
    if (arm_cartesian_runtime.path_type ==
        ARM_TRAJECTORY_PATH_CARTESIAN_LINEAR) {
        if (!arm_cartesian_runtime.online_ik_valid ||
            (uint32_t)(now_ms - arm_cartesian_runtime.last_ik_tick) >=
                ARM_LINEAR_IK_UPDATE_MS) {
            /* 在线IK失败时冻结上一合法参考，电机继续角度闭环保持。 */
            if (!ArmCartesianSolveOnlineIK(now_ms, progress) &&
                !arm_cartesian_runtime.online_ik_valid) {
                arm_cartesian_runtime.trajectory_start_tick++;
                return;
            }
        }
        ArmCartesianInterpolateOnlineIK(now_ms, reference_q_deg);
    } else {
        ArmCartesianInterpolateJointSamples(progress, reference_q_deg);
    }
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
        ArmCartesianSetState(ARM_MOTION_HOLDING, now_ms);
    }
}

static void ArmCartesianRunRealtime(uint32_t now_ms, uint32_t task_delta_ms)
{
    Arm_IK_Result_s result;
    Arm_Position_s candidate_position;
    float dt_s = (float)task_delta_ms * 0.001f;
    float position_error[3];
    float desired_velocity[3];
    float acceleration[3];
    float remaining_distance;
    float reference_step[3];
    float reference_q_deg[3];

    memcpy(arm_cartesian_runtime.realtime_last_velocity_mm_s,
           arm_cartesian_runtime.realtime_velocity_mm_s,
           sizeof(arm_cartesian_runtime.realtime_last_velocity_mm_s));

    if ((uint32_t)(now_ms -
                   arm_cartesian_runtime.realtime_last_command_tick) >
        ARM_REALTIME_COMMAND_TIMEOUT_MS) {
        arm_cartesian_runtime.realtime_timed_out = 1u;
        memset(arm_cartesian_runtime.realtime_velocity_mm_s, 0,
               sizeof(arm_cartesian_runtime.realtime_velocity_mm_s));
        /* 断流冻结当前生成参考，角度环保持，不退出实时模式也不失能。 */
        return;
    }
    arm_cartesian_runtime.realtime_timed_out = 0u;
    if (dt_s <= 0.0f) {
        return;
    }

    position_error[0] = arm_cartesian_runtime.realtime_target.target_mm.x_mm -
        arm_cartesian_runtime.realtime_reference_position.x_mm;
    position_error[1] = arm_cartesian_runtime.realtime_target.target_mm.y_mm -
        arm_cartesian_runtime.realtime_reference_position.y_mm;
    position_error[2] = arm_cartesian_runtime.realtime_target.target_mm.z_mm -
        arm_cartesian_runtime.realtime_reference_position.z_mm;
    remaining_distance = ArmCartesianVectorLength(position_error);
    {
        float braking_speed = sqrtf(2.0f *
            arm_cartesian_runtime.realtime_target.max_acceleration_mm_s2 *
            remaining_distance);
        float desired_speed = fminf(
            arm_cartesian_runtime.realtime_target.max_speed_mm_s,
            braking_speed);

        memcpy(desired_velocity, position_error, sizeof(desired_velocity));
        ArmCartesianSetVectorLength(desired_velocity, desired_speed);
    }
    acceleration[0] = (desired_velocity[0] -
        arm_cartesian_runtime.realtime_velocity_mm_s[0]) / dt_s;
    acceleration[1] = (desired_velocity[1] -
        arm_cartesian_runtime.realtime_velocity_mm_s[1]) / dt_s;
    acceleration[2] = (desired_velocity[2] -
        arm_cartesian_runtime.realtime_velocity_mm_s[2]) / dt_s;
    ArmCartesianLimitVector(acceleration,
        arm_cartesian_runtime.realtime_target.max_acceleration_mm_s2);
    for (uint8_t axis = 0u; axis < 3u; ++axis) {
        arm_cartesian_runtime.realtime_velocity_mm_s[axis] +=
            acceleration[axis] * dt_s;
    }
    ArmCartesianLimitVector(arm_cartesian_runtime.realtime_velocity_mm_s,
        arm_cartesian_runtime.realtime_target.max_speed_mm_s);

    reference_step[0] = arm_cartesian_runtime.realtime_velocity_mm_s[0] *
        dt_s;
    reference_step[1] = arm_cartesian_runtime.realtime_velocity_mm_s[1] *
        dt_s;
    reference_step[2] = arm_cartesian_runtime.realtime_velocity_mm_s[2] *
        dt_s;
    /*
     * 最新目标覆盖模式下，参考点是用速度积分生成的。若已经到达或下一步
     * 会越过目标，直接夹到目标点并清速度，避免积分冲出工作空间后触发
     * IK_OUT_OF_REACH，看起来像“到一半停住”。
     */
    if (remaining_distance <= 0.05f ||
        reference_step[0] * position_error[0] +
        reference_step[1] * position_error[1] +
        reference_step[2] * position_error[2] >=
            remaining_distance * remaining_distance) {
        arm_cartesian_runtime.realtime_reference_position =
            arm_cartesian_runtime.realtime_target.target_mm;
        memset(arm_cartesian_runtime.realtime_velocity_mm_s, 0,
               sizeof(arm_cartesian_runtime.realtime_velocity_mm_s));
    } else {
        arm_cartesian_runtime.realtime_reference_position.x_mm +=
            reference_step[0];
        arm_cartesian_runtime.realtime_reference_position.y_mm +=
            reference_step[1];
        arm_cartesian_runtime.realtime_reference_position.z_mm +=
            reference_step[2];
    }
    candidate_position = arm_cartesian_runtime.realtime_reference_position;
    if ((uint32_t)(now_ms - arm_cartesian_runtime.last_ik_tick) >=
        ARM_LINEAR_IK_UPDATE_MS) {
        Arm_Realtime_Reject_Reason_e reject_reason =
            ARM_REALTIME_REJECT_NONE;

        memset(&result, 0, sizeof(result));
        if (ArmInverseKinematics3DOF(
                &candidate_position,
                arm_cartesian_runtime.online_ik_next_q_deg,
                &result) != ARM_IK_OK) {
            reject_reason = ARM_REALTIME_REJECT_IK;
        } else if (result.position_error_mm > ARM_LINEAR_FK_ERROR_MAX_MM) {
            reject_reason = ARM_REALTIME_REJECT_FK_ERROR;
        } else if (!ArmJointPoseWithinSoftLimits(result.q_deg)) {
            reject_reason = ARM_REALTIME_REJECT_SOFT_LIMIT;
        } else if (!ArmAutoPoseIsSafe(result.q_deg)) {
            reject_reason = ARM_REALTIME_REJECT_AUTO_REGION;
        } else if (!ArmCartesianJointStepContinuous(
                       arm_cartesian_runtime.online_ik_next_q_deg,
                       result.q_deg)) {
            reject_reason = ARM_REALTIME_REJECT_CONTINUITY;
        }

        if (reject_reason == ARM_REALTIME_REJECT_NONE) {
            memcpy(arm_cartesian_runtime.online_ik_previous_q_deg,
                   arm_cartesian_runtime.online_ik_next_q_deg,
                   sizeof(arm_cartesian_runtime.online_ik_previous_q_deg));
            memcpy(arm_cartesian_runtime.online_ik_next_q_deg,
                   result.q_deg, sizeof(result.q_deg));
            arm_cartesian_runtime.realtime_last_valid_position =
                candidate_position;
            arm_cartesian_runtime.online_ik_interval_start_tick = now_ms;
            arm_cartesian_runtime.online_ik_interval_ms =
                ARM_LINEAR_IK_UPDATE_MS;
            arm_cartesian_runtime.last_ik_tick = now_ms;
            g_arm_motion_debug.ik_status = ARM_IK_OK;
            g_arm_motion_debug.command_accepted = 1u;
            g_arm_motion_debug.realtime_reject_reason =
                ARM_REALTIME_REJECT_NONE;
        } else {
            /* 局部IK拒绝时保留上一合法位置和角度参考。 */
            memset(arm_cartesian_runtime.realtime_velocity_mm_s, 0,
                   sizeof(arm_cartesian_runtime.realtime_velocity_mm_s));
            arm_cartesian_runtime.realtime_reference_position =
                arm_cartesian_runtime.realtime_last_valid_position;
            g_arm_motion_debug.ik_status = result.status;
            g_arm_motion_debug.command_accepted = 0u;
            g_arm_motion_debug.realtime_reject_reason = reject_reason;
            g_arm_motion_debug.last_realtime_reject_reason = reject_reason;
            g_arm_motion_debug.command_reject_count++;
        }
    }

    ArmCartesianInterpolateOnlineIK(now_ms, reference_q_deg);
    if (ArmUpdateJointReference(reference_q_deg)) {
        memcpy(g_arm_motion_debug.trajectory_q_deg, reference_q_deg,
               sizeof(reference_q_deg));
    }
    g_arm_motion_debug.trajectory_elapsed_ms =
        (uint32_t)(now_ms - arm_cartesian_runtime.trajectory_start_tick);
    g_arm_motion_debug.trajectory_progress = 0.0f;
}

void ArmTrajectoryTask(uint32_t now_ms)
{
    uint32_t task_delta_ms =
        (uint32_t)(now_ms - arm_cartesian_runtime.last_task_tick);

    arm_cartesian_runtime.last_task_tick = now_ms;
    if (!arm_cartesian_runtime.kinematics_self_test_passed) {
        ArmCartesianUpdateDebug();
        ArmCartesianUpdateControlDebug(now_ms, task_delta_ms);
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
        ArmCartesianUpdateControlDebug(now_ms, task_delta_ms);
        return;
    }
    if (g_arm_motion_debug.fault_code == ARM_MOTION_FAULT_OFFLINE) {
        g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_NONE;
    }

    if (arm_cartesian_runtime.realtime_active) {
        ArmCartesianRunRealtime(now_ms, task_delta_ms);
        ArmCartesianUpdateDebug();
        ArmCartesianUpdateControlDebug(now_ms, task_delta_ms);
        return;
    }

    switch (g_arm_motion_debug.motion_state) {
        case ARM_MOTION_IDLE:
        case ARM_MOTION_BOOT_DELAY:
            ArmCartesianSetState(ARM_MOTION_HOLDING, now_ms);
            break;

        case ARM_MOTION_STAGING:
        case ARM_MOTION_RUNNING:
            ArmCartesianRunPreparedTrajectory(now_ms);
            break;

        case ARM_MOTION_SETTLING:
        case ARM_MOTION_HOLDING:
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
    ArmCartesianUpdateControlDebug(now_ms, task_delta_ms);
}
