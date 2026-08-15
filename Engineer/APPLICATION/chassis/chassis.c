/**
 * @file chassis.c
 * @brief 双轮里程计、IMU 航向 PID 和通用相对直线/转角命令执行器。
 */

#include "chassis.h"

#include "chassis_config.h"
#include "DJI_motor.h"
#include "daemon.h"
#include "stm32f4xx_hal.h"

#include <math.h>
#include <string.h>

#define CHASSIS_PI 3.14159265358979323846f
#define CHASSIS_DEG_TO_RAD (CHASSIS_PI / 180.0f)
#define CHASSIS_RAD_TO_DEG (180.0f / CHASSIS_PI)

Chassis_Debug_s g_chassis_debug;

static DJIMotor_Instance *chassis_left_motor;
static DJIMotor_Instance *chassis_right_motor;
static attitude_t *chassis_imu;
static uint32_t chassis_last_control_tick;
static uint32_t chassis_imu_stable_tick;
static uint32_t chassis_stop_stable_tick;
static float chassis_left_zero_angle_deg;
static float chassis_right_zero_angle_deg;
static float chassis_last_left_distance_m;
static float chassis_last_right_distance_m;
static float chassis_last_imu_yaw_deg;
static float chassis_ramped_linear_m_s;
static float chassis_ramped_angular_rad_s;
static uint32_t chassis_latest_command_id;
static Chassis_State_e chassis_stop_terminal_state;

static float ChassisClamp(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static float ChassisSign(float value)
{
    return value >= 0.0f ? 1.0f : -1.0f;
}

static float ChassisMoveToward(float current, float target, float max_step)
{
    if (target > current + max_step) return current + max_step;
    if (target < current - max_step) return current - max_step;
    return target;
}

static float ChassisMotorDegSToWheelMS(float motor_deg_s, float sign)
{
    return sign * motor_deg_s * CHASSIS_DEG_TO_RAD /
        CHASSIS_REDUCTION_RATIO * CHASSIS_WHEEL_RADIUS_M;
}

static float ChassisWheelMSToMotorDegS(float wheel_m_s)
{
    return wheel_m_s / CHASSIS_WHEEL_RADIUS_M *
        CHASSIS_REDUCTION_RATIO * CHASSIS_RAD_TO_DEG;
}

static float ChassisAngleToDistance(float angle_deg, float zero_deg,
                                    float feedback_sign)
{
    return feedback_sign * (angle_deg - zero_deg) * CHASSIS_DEG_TO_RAD /
        CHASSIS_REDUCTION_RATIO * CHASSIS_WHEEL_RADIUS_M;
}

static uint8_t ChassisImuFinite(void)
{
    return chassis_imu != NULL && isfinite(chassis_imu->Yaw) &&
        isfinite(chassis_imu->YawTotalAngle) &&
        isfinite(chassis_imu->Gyro[Z]);
}

static uint8_t ChassisMotorsOnline(void)
{
    uint8_t left_online = chassis_left_motor != NULL &&
        chassis_left_motor->daemon != NULL &&
        DaemonIsOnline(chassis_left_motor->daemon);
    uint8_t right_online = chassis_right_motor != NULL &&
        chassis_right_motor->daemon != NULL &&
        DaemonIsOnline(chassis_right_motor->daemon);

    g_chassis_debug.left_online = left_online;
    g_chassis_debug.right_online = right_online;
    return left_online && right_online;
}

static void ChassisSetWheelTargets(float left_m_s, float right_m_s)
{
    float left_motor_deg_s;
    float right_motor_deg_s;

    left_m_s = ChassisClamp(left_m_s, -CHASSIS_MAX_WHEEL_SPEED_M_S,
                            CHASSIS_MAX_WHEEL_SPEED_M_S);
    right_m_s = ChassisClamp(right_m_s, -CHASSIS_MAX_WHEEL_SPEED_M_S,
                             CHASSIS_MAX_WHEEL_SPEED_M_S);
    left_motor_deg_s = ChassisWheelMSToMotorDegS(left_m_s) *
        CHASSIS_LEFT_COMMAND_SIGN;
    right_motor_deg_s = ChassisWheelMSToMotorDegS(right_m_s) *
        CHASSIS_RIGHT_COMMAND_SIGN;
    g_chassis_debug.left_target_m_s = left_m_s;
    g_chassis_debug.right_target_m_s = right_m_s;
    g_chassis_debug.left_target_motor_deg_s = left_motor_deg_s;
    g_chassis_debug.right_target_motor_deg_s = right_motor_deg_s;
    if (chassis_left_motor != NULL) {
        DJIMotorSetRef(chassis_left_motor, left_motor_deg_s);
    }
    if (chassis_right_motor != NULL) {
        DJIMotorSetRef(chassis_right_motor, right_motor_deg_s);
    }
}

static void ChassisSetBodyVelocityTargets(float linear_m_s,
                                          float angular_rad_s)
{
    float left_m_s = linear_m_s - angular_rad_s *
        CHASSIS_TRACK_WIDTH_M * 0.5f;
    float right_m_s = linear_m_s + angular_rad_s *
        CHASSIS_TRACK_WIDTH_M * 0.5f;
    float max_abs_m_s = fmaxf(fabsf(left_m_s), fabsf(right_m_s));
    float scale = 1.0f;

    if (max_abs_m_s > CHASSIS_MAX_WHEEL_SPEED_M_S) {
        scale = CHASSIS_MAX_WHEEL_SPEED_M_S / max_abs_m_s;
        left_m_s *= scale;
        right_m_s *= scale;
    }
    g_chassis_debug.velocity_wheel_scale = scale;
    ChassisSetWheelTargets(left_m_s, right_m_s);
}

static void ChassisZeroTargets(void)
{
    g_chassis_debug.linear_command_m_s = 0.0f;
    g_chassis_debug.angular_command_rad_s = 0.0f;
    ChassisSetBodyVelocityTargets(0.0f, 0.0f);
}

static void ChassisResetHeadingControl(void)
{
    g_chassis_debug.heading_pid_integral_deg_s = 0.0f;
    g_chassis_debug.heading_pid_p_rad_s = 0.0f;
    g_chassis_debug.heading_pid_i_rad_s = 0.0f;
    g_chassis_debug.heading_pid_d_rad_s = 0.0f;
    g_chassis_debug.heading_correction_rad_s = 0.0f;
}

static void ChassisResetMotion(void)
{
    chassis_ramped_linear_m_s = 0.0f;
    chassis_ramped_angular_rad_s = 0.0f;
    ChassisResetHeadingControl();
    g_chassis_debug.turn_pid_integral_deg_s = 0.0f;
    g_chassis_debug.turn_pid_p_rad_s = 0.0f;
    g_chassis_debug.turn_pid_i_rad_s = 0.0f;
    g_chassis_debug.turn_pid_d_rad_s = 0.0f;
    g_chassis_debug.turn_output_rad_s = 0.0f;
    g_chassis_debug.turn_stable_tick = 0u;
}

static void ChassisSetState(Chassis_State_e state, uint32_t now_ms)
{
    g_chassis_debug.state = state;
    g_chassis_debug.state_tick = now_ms;
}

static void ChassisLatchFault(Chassis_Fault_e fault)
{
    if (g_chassis_debug.state != CHASSIS_STATE_FAULT) {
        if (fault == CHASSIS_FAULT_MOTOR_OFFLINE) {
            g_chassis_debug.motor_offline_count++;
        } else if (fault == CHASSIS_FAULT_IMU_INVALID) {
            g_chassis_debug.imu_fault_count++;
        }
    }
    g_chassis_debug.fault = fault;
    g_chassis_debug.state = CHASSIS_STATE_FAULT;
    g_chassis_debug.motion_enabled = 0u;
    ChassisResetMotion();
    ChassisZeroTargets();
    if (chassis_left_motor != NULL) DJIMotorStop(chassis_left_motor);
    if (chassis_right_motor != NULL) DJIMotorStop(chassis_right_motor);
}

static void ChassisCaptureZero(uint32_t now_ms)
{
    chassis_left_zero_angle_deg = chassis_left_motor->measure.total_angle;
    chassis_right_zero_angle_deg = chassis_right_motor->measure.total_angle;
    chassis_last_left_distance_m = 0.0f;
    chassis_last_right_distance_m = 0.0f;
    chassis_last_imu_yaw_deg = CHASSIS_IMU_YAW_SIGN *
        chassis_imu->YawTotalAngle;
    g_chassis_debug.left_distance_m = 0.0f;
    g_chassis_debug.right_distance_m = 0.0f;
    g_chassis_debug.travel_distance_m = 0.0f;
    g_chassis_debug.yaw_zero_deg = chassis_last_imu_yaw_deg;
    g_chassis_debug.yaw_deg = 0.0f;
    g_chassis_debug.wheel_yaw_deg = 0.0f;
    g_chassis_debug.x_m = 0.0f;
    g_chassis_debug.y_m = 0.0f;
    g_chassis_debug.state_tick = now_ms;
}

static void ChassisUpdateOdometry(void)
{
    float left_distance;
    float right_distance;
    float dl;
    float dr;
    float ds;
    float yaw_deg;
    float yaw_mid_rad;

    g_chassis_debug.left_total_angle_deg = chassis_left_motor->measure.total_angle;
    g_chassis_debug.right_total_angle_deg = chassis_right_motor->measure.total_angle;
    left_distance = ChassisAngleToDistance(g_chassis_debug.left_total_angle_deg,
        chassis_left_zero_angle_deg, CHASSIS_LEFT_FEEDBACK_SIGN);
    right_distance = ChassisAngleToDistance(g_chassis_debug.right_total_angle_deg,
        chassis_right_zero_angle_deg, CHASSIS_RIGHT_FEEDBACK_SIGN);
    dl = left_distance - chassis_last_left_distance_m;
    dr = right_distance - chassis_last_right_distance_m;
    ds = 0.5f * (dl + dr);
    yaw_deg = CHASSIS_IMU_YAW_SIGN * chassis_imu->YawTotalAngle -
        g_chassis_debug.yaw_zero_deg;
    yaw_mid_rad = 0.5f * (chassis_last_imu_yaw_deg -
        g_chassis_debug.yaw_zero_deg + yaw_deg) * CHASSIS_DEG_TO_RAD;
    g_chassis_debug.x_m += ds * cosf(yaw_mid_rad);
    g_chassis_debug.y_m += ds * sinf(yaw_mid_rad);
    g_chassis_debug.yaw_deg = yaw_deg;
    g_chassis_debug.wheel_yaw_deg =
        (right_distance - left_distance) / CHASSIS_TRACK_WIDTH_M *
        CHASSIS_RAD_TO_DEG;
    g_chassis_debug.imu_wheel_yaw_error_deg = yaw_deg -
        g_chassis_debug.wheel_yaw_deg;
    g_chassis_debug.left_distance_m = left_distance;
    g_chassis_debug.right_distance_m = right_distance;
    g_chassis_debug.travel_distance_m = 0.5f *
        (left_distance + right_distance);
    g_chassis_debug.left_speed_m_s = ChassisMotorDegSToWheelMS(
        chassis_left_motor->measure.speed_aps, CHASSIS_LEFT_FEEDBACK_SIGN);
    g_chassis_debug.right_speed_m_s = ChassisMotorDegSToWheelMS(
        chassis_right_motor->measure.speed_aps, CHASSIS_RIGHT_FEEDBACK_SIGN);
    g_chassis_debug.imu_gyro_z_rad_s = CHASSIS_IMU_YAW_SIGN *
        chassis_imu->Gyro[Z];
    g_chassis_debug.velocity_actual_vx_mm_s = 500.0f *
        (g_chassis_debug.left_speed_m_s +
         g_chassis_debug.right_speed_m_s);
    g_chassis_debug.velocity_actual_wz_rad_s =
        g_chassis_debug.imu_gyro_z_rad_s;
    chassis_last_left_distance_m = left_distance;
    chassis_last_right_distance_m = right_distance;
    chassis_last_imu_yaw_deg = CHASSIS_IMU_YAW_SIGN *
        chassis_imu->YawTotalAngle;
    g_chassis_debug.left_feedback_count = chassis_left_motor->feedback_count;
    g_chassis_debug.right_feedback_count = chassis_right_motor->feedback_count;
}

static uint8_t ChassisFeedbackHealthy(void)
{
    if (!ChassisMotorsOnline()) {
        ChassisLatchFault(CHASSIS_FAULT_MOTOR_OFFLINE);
        return 0u;
    }
    if (g_chassis_debug.imu_healthy == 0u) {
        ChassisLatchFault(CHASSIS_FAULT_IMU_INVALID);
        return 0u;
    }
    return 1u;
}

static float ChassisHeadingPid(float error_deg, float dt_s)
{
    float old_integral = g_chassis_debug.heading_pid_integral_deg_s;
    float raw_output;
    float output;

    if (fabsf(error_deg) <= CHASSIS_HEADING_INTEGRAL_ZONE_DEG) {
        g_chassis_debug.heading_pid_integral_deg_s = ChassisClamp(
            old_integral + error_deg * dt_s,
            -CHASSIS_HEADING_INTEGRAL_LIMIT_DEG_S,
            CHASSIS_HEADING_INTEGRAL_LIMIT_DEG_S);
    } else {
        g_chassis_debug.heading_pid_integral_deg_s = 0.0f;
    }
    g_chassis_debug.heading_pid_p_rad_s =
        CHASSIS_HEADING_KP_RAD_S_PER_DEG * error_deg;
    g_chassis_debug.heading_pid_i_rad_s =
        CHASSIS_HEADING_KI_RAD_S_PER_DEG_S *
        g_chassis_debug.heading_pid_integral_deg_s;
    g_chassis_debug.heading_pid_d_rad_s =
        -CHASSIS_HEADING_KD * g_chassis_debug.imu_gyro_z_rad_s;
    raw_output = g_chassis_debug.heading_pid_p_rad_s +
        g_chassis_debug.heading_pid_i_rad_s +
        g_chassis_debug.heading_pid_d_rad_s;
    output = ChassisClamp(raw_output, -CHASSIS_HEADING_MAX_OUTPUT_RAD_S,
                          CHASSIS_HEADING_MAX_OUTPUT_RAD_S);
    if (raw_output != output && raw_output * error_deg > 0.0f) {
        g_chassis_debug.heading_pid_integral_deg_s = old_integral;
        g_chassis_debug.heading_pid_i_rad_s =
            CHASSIS_HEADING_KI_RAD_S_PER_DEG_S * old_integral;
        output = ChassisClamp(g_chassis_debug.heading_pid_p_rad_s +
            g_chassis_debug.heading_pid_i_rad_s +
            g_chassis_debug.heading_pid_d_rad_s,
            -CHASSIS_HEADING_MAX_OUTPUT_RAD_S,
            CHASSIS_HEADING_MAX_OUTPUT_RAD_S);
    }
    g_chassis_debug.heading_correction_rad_s = output;
    return output;
}

static float ChassisTurnPid(float error_deg, float dt_s)
{
    float old_integral = g_chassis_debug.turn_pid_integral_deg_s;
    float raw_output;
    float output;

    if (fabsf(error_deg) <= CHASSIS_TURN_INTEGRAL_ZONE_DEG) {
        g_chassis_debug.turn_pid_integral_deg_s = ChassisClamp(
            old_integral + error_deg * dt_s,
            -CHASSIS_TURN_INTEGRAL_LIMIT_DEG_S,
            CHASSIS_TURN_INTEGRAL_LIMIT_DEG_S);
    } else {
        g_chassis_debug.turn_pid_integral_deg_s = 0.0f;
    }
    g_chassis_debug.turn_pid_p_rad_s =
        CHASSIS_TURN_KP_RAD_S_PER_DEG * error_deg;
    g_chassis_debug.turn_pid_i_rad_s = CHASSIS_TURN_KI_RAD_S_PER_DEG_S *
        g_chassis_debug.turn_pid_integral_deg_s;
    g_chassis_debug.turn_pid_d_rad_s =
        -CHASSIS_TURN_KD * g_chassis_debug.imu_gyro_z_rad_s;
    raw_output = g_chassis_debug.turn_pid_p_rad_s +
        g_chassis_debug.turn_pid_i_rad_s +
        g_chassis_debug.turn_pid_d_rad_s;
    output = ChassisClamp(raw_output, -CHASSIS_TURN_MAX_RATE_RAD_S,
                          CHASSIS_TURN_MAX_RATE_RAD_S);
    if (raw_output != output && raw_output * error_deg > 0.0f) {
        g_chassis_debug.turn_pid_integral_deg_s = old_integral;
        g_chassis_debug.turn_pid_i_rad_s =
            CHASSIS_TURN_KI_RAD_S_PER_DEG_S * old_integral;
        output = ChassisClamp(g_chassis_debug.turn_pid_p_rad_s +
            g_chassis_debug.turn_pid_i_rad_s +
            g_chassis_debug.turn_pid_d_rad_s,
            -CHASSIS_TURN_MAX_RATE_RAD_S,
            CHASSIS_TURN_MAX_RATE_RAD_S);
    }
    return output;
}

static void ChassisBeginCommand(uint32_t now_ms)
{
    ChassisResetMotion();
    g_chassis_debug.segment_start_left_m = g_chassis_debug.left_distance_m;
    g_chassis_debug.segment_start_right_m = g_chassis_debug.right_distance_m;
    g_chassis_debug.segment_left_distance_m = 0.0f;
    g_chassis_debug.segment_right_distance_m = 0.0f;
    g_chassis_debug.segment_distance_m = 0.0f;
    g_chassis_debug.actual_distance_mm = 0.0f;
    g_chassis_debug.actual_angle_deg = 0.0f;
    g_chassis_debug.heading_target_deg = g_chassis_debug.yaw_deg;
    g_chassis_debug.turn_start_yaw_deg = g_chassis_debug.yaw_deg;
    g_chassis_debug.turn_target_yaw_deg = g_chassis_debug.yaw_deg +
        g_chassis_debug.target_angle_deg;
    g_chassis_debug.command_start_tick = now_ms;
    g_chassis_debug.test_start_tick = now_ms;
    if (g_chassis_debug.command_type ==
        CHASSIS_COMMAND_RELATIVE_STRAIGHT) {
        g_chassis_debug.straight_count++;
    }
    DJIMotorEnable(chassis_left_motor);
    DJIMotorEnable(chassis_right_motor);
    g_chassis_debug.motion_enabled = 1u;
    ChassisSetState(CHASSIS_STATE_RUNNING, now_ms);
}

static void ChassisUpdateCommandMeasurements(void)
{
    g_chassis_debug.segment_left_distance_m =
        g_chassis_debug.left_distance_m -
        g_chassis_debug.segment_start_left_m;
    g_chassis_debug.segment_right_distance_m =
        g_chassis_debug.right_distance_m -
        g_chassis_debug.segment_start_right_m;
    g_chassis_debug.segment_distance_m = 0.5f *
        (g_chassis_debug.segment_left_distance_m +
         g_chassis_debug.segment_right_distance_m);
    g_chassis_debug.actual_distance_mm =
        g_chassis_debug.segment_distance_m * 1000.0f;
    g_chassis_debug.actual_angle_deg = g_chassis_debug.yaw_deg -
        g_chassis_debug.turn_start_yaw_deg;
}

static void ChassisBeginStopping(uint32_t now_ms,
                                 Chassis_State_e terminal_state)
{
    chassis_stop_terminal_state = terminal_state;
    chassis_stop_stable_tick = 0u;
    g_chassis_debug.velocity_heading_hold_active = 0u;
    if (g_chassis_debug.command_type ==
            CHASSIS_COMMAND_BODY_VELOCITY) {
        g_chassis_debug.velocity_target_vx_mm_s = 0.0f;
        g_chassis_debug.velocity_target_wz_rad_s = 0.0f;
    }
    if (terminal_state == CHASSIS_STATE_COMPLETED) {
        ChassisResetMotion();
        ChassisZeroTargets();
    }
    ChassisSetState(CHASSIS_STATE_STOPPING, now_ms);
}

static void ChassisApplyVelocityTarget(float vx_mm_s, float wz_rad_s,
                                       uint32_t now_ms)
{
    uint8_t old_heading_hold =
        g_chassis_debug.velocity_heading_hold_active;
    uint8_t new_heading_hold;

    if (fabsf(vx_mm_s) <= CHASSIS_VELOCITY_LINEAR_ZERO_MM_S) {
        vx_mm_s = 0.0f;
    }
    if (fabsf(wz_rad_s) <= CHASSIS_VELOCITY_ANGULAR_ZERO_RAD_S) {
        wz_rad_s = 0.0f;
    }
    new_heading_hold = (uint8_t)(vx_mm_s != 0.0f && wz_rad_s == 0.0f);
    if (new_heading_hold != 0u && old_heading_hold == 0u) {
        g_chassis_debug.heading_target_deg = g_chassis_debug.yaw_deg;
        g_chassis_debug.velocity_heading_capture_count++;
        ChassisResetHeadingControl();
    } else if (new_heading_hold == 0u && old_heading_hold != 0u) {
        ChassisResetHeadingControl();
    }
    g_chassis_debug.velocity_heading_hold_active = new_heading_hold;
    g_chassis_debug.velocity_target_vx_mm_s = vx_mm_s;
    g_chassis_debug.velocity_target_wz_rad_s = wz_rad_s;
    g_chassis_debug.velocity_command_tick = now_ms;
}

static void ChassisRunVelocity(uint32_t now_ms, float dt_s)
{
    float desired_linear_m_s;
    float desired_angular_rad_s;

    ChassisUpdateCommandMeasurements();
    if ((uint32_t)(now_ms - g_chassis_debug.velocity_command_tick) >=
            CHASSIS_VELOCITY_COMMAND_TIMEOUT_MS) {
        g_chassis_debug.velocity_timeout_count++;
        ChassisBeginStopping(now_ms, CHASSIS_STATE_CANCELLED);
        return;
    }
    desired_linear_m_s =
        g_chassis_debug.velocity_target_vx_mm_s * 0.001f;
    if (g_chassis_debug.velocity_heading_hold_active != 0u) {
        g_chassis_debug.heading_error_deg =
            g_chassis_debug.heading_target_deg - g_chassis_debug.yaw_deg;
        desired_angular_rad_s = ChassisHeadingPid(
            g_chassis_debug.heading_error_deg, dt_s);
    } else {
        g_chassis_debug.heading_error_deg = 0.0f;
        g_chassis_debug.heading_correction_rad_s = 0.0f;
        desired_angular_rad_s =
            g_chassis_debug.velocity_target_wz_rad_s;
    }
    chassis_ramped_linear_m_s = ChassisMoveToward(
        chassis_ramped_linear_m_s, desired_linear_m_s,
        CHASSIS_MAX_LINEAR_ACCEL_M_S2 * dt_s);
    chassis_ramped_angular_rad_s = ChassisMoveToward(
        chassis_ramped_angular_rad_s, desired_angular_rad_s,
        CHASSIS_TURN_MAX_ACCEL_RAD_S2 * dt_s);
    g_chassis_debug.linear_command_m_s = chassis_ramped_linear_m_s;
    g_chassis_debug.angular_command_rad_s =
        chassis_ramped_angular_rad_s;
    ChassisSetBodyVelocityTargets(chassis_ramped_linear_m_s,
                                  chassis_ramped_angular_rad_s);
}

static void ChassisRunStraight(uint32_t now_ms, float dt_s)
{
    float direction = ChassisSign(g_chassis_debug.straight_target_distance_m);
    float target_abs_m = fabsf(g_chassis_debug.straight_target_distance_m);
    float progress_m;
    float remaining_m;
    float desired_abs_m_s;
    float desired_linear_m_s;
    float max_linear_step;
    float angular_rad_s = 0.0f;

    ChassisUpdateCommandMeasurements();
    progress_m = direction * g_chassis_debug.segment_distance_m;
    if ((uint32_t)(now_ms - g_chassis_debug.command_start_tick) >=
            CHASSIS_DIRECTION_CHECK_MS &&
        (direction * g_chassis_debug.segment_left_distance_m <
             CHASSIS_DIRECTION_CHECK_MIN_M ||
         direction * g_chassis_debug.segment_right_distance_m <
             CHASSIS_DIRECTION_CHECK_MIN_M)) {
        g_chassis_debug.direction_fault_count++;
        ChassisLatchFault(CHASSIS_FAULT_DIRECTION);
        return;
    }
    if ((uint32_t)(now_ms - g_chassis_debug.command_start_tick) >=
        CHASSIS_STRAIGHT_TIMEOUT_MS) {
        ChassisLatchFault(CHASSIS_FAULT_TIMEOUT);
        return;
    }
    if (fabsf(g_chassis_debug.segment_distance_m) >
        target_abs_m + CHASSIS_EXCESS_DISTANCE_M) {
        ChassisLatchFault(CHASSIS_FAULT_EXCESS_DISTANCE);
        return;
    }
    remaining_m = target_abs_m - progress_m;
    if (remaining_m <= g_chassis_debug.straight_tolerance_m) {
        ChassisBeginStopping(now_ms, CHASSIS_STATE_COMPLETED);
        return;
    }
    desired_abs_m_s = ChassisClamp(CHASSIS_TEST_POSITION_KP * remaining_m,
        CHASSIS_TEST_MIN_SPEED_M_S, CHASSIS_TEST_MAX_SPEED_M_S);
    if (remaining_m < CHASSIS_TEST_DECEL_DISTANCE_M) {
        desired_abs_m_s = ChassisClamp(
            CHASSIS_TEST_MAX_SPEED_M_S * remaining_m /
                CHASSIS_TEST_DECEL_DISTANCE_M,
            CHASSIS_TEST_MIN_SPEED_M_S, CHASSIS_TEST_MAX_SPEED_M_S);
    }
    desired_linear_m_s = direction * desired_abs_m_s;
    max_linear_step = CHASSIS_MAX_LINEAR_ACCEL_M_S2 * dt_s;
    if (desired_linear_m_s > chassis_ramped_linear_m_s + max_linear_step) {
        chassis_ramped_linear_m_s += max_linear_step;
    } else if (desired_linear_m_s <
               chassis_ramped_linear_m_s - max_linear_step) {
        chassis_ramped_linear_m_s -= max_linear_step;
    } else {
        chassis_ramped_linear_m_s = desired_linear_m_s;
    }
    g_chassis_debug.heading_error_deg =
        g_chassis_debug.heading_target_deg - g_chassis_debug.yaw_deg;
    if (g_chassis_debug.heading_mode == CHASSIS_HEADING_HOLD_START) {
        angular_rad_s = ChassisHeadingPid(
            g_chassis_debug.heading_error_deg, dt_s);
    }
    g_chassis_debug.linear_command_m_s = chassis_ramped_linear_m_s;
    g_chassis_debug.angular_command_rad_s = angular_rad_s;
    ChassisSetBodyVelocityTargets(chassis_ramped_linear_m_s,
                                  angular_rad_s);
}

static void ChassisRunTurn(uint32_t now_ms, float dt_s)
{
    float direction = ChassisSign(g_chassis_debug.target_angle_deg);
    float desired_angular_rad_s;
    float max_angular_step;

    ChassisUpdateCommandMeasurements();
    g_chassis_debug.turn_error_deg = g_chassis_debug.turn_target_yaw_deg -
        g_chassis_debug.yaw_deg;
    if ((uint32_t)(now_ms - g_chassis_debug.command_start_tick) >=
            CHASSIS_TURN_DIRECTION_CHECK_MS &&
        direction * g_chassis_debug.actual_angle_deg <
            CHASSIS_TURN_DIRECTION_CHECK_DEG) {
        g_chassis_debug.turn_direction_fault_count++;
        ChassisLatchFault(CHASSIS_FAULT_TURN_DIRECTION);
        return;
    }
    if ((uint32_t)(now_ms - g_chassis_debug.command_start_tick) >=
        CHASSIS_TURN_TIMEOUT_MS) {
        ChassisLatchFault(CHASSIS_FAULT_TIMEOUT);
        return;
    }
    desired_angular_rad_s = ChassisTurnPid(
        g_chassis_debug.turn_error_deg, dt_s);
    max_angular_step = CHASSIS_TURN_MAX_ACCEL_RAD_S2 * dt_s;
    if (desired_angular_rad_s > chassis_ramped_angular_rad_s +
        max_angular_step) {
        chassis_ramped_angular_rad_s += max_angular_step;
    } else if (desired_angular_rad_s < chassis_ramped_angular_rad_s -
               max_angular_step) {
        chassis_ramped_angular_rad_s -= max_angular_step;
    } else {
        chassis_ramped_angular_rad_s = desired_angular_rad_s;
    }
    g_chassis_debug.turn_output_rad_s = chassis_ramped_angular_rad_s;
    g_chassis_debug.linear_command_m_s = 0.0f;
    g_chassis_debug.angular_command_rad_s = chassis_ramped_angular_rad_s;
    ChassisSetBodyVelocityTargets(0.0f,
                                  chassis_ramped_angular_rad_s);
    if (fabsf(g_chassis_debug.turn_error_deg) <=
            CHASSIS_TURN_ERROR_TOLERANCE_DEG &&
        fabsf(g_chassis_debug.imu_gyro_z_rad_s) <=
            CHASSIS_TURN_GYRO_TOLERANCE_RAD_S) {
        if (g_chassis_debug.turn_stable_tick == 0u) {
            g_chassis_debug.turn_stable_tick = now_ms;
        }
        if ((uint32_t)(now_ms - g_chassis_debug.turn_stable_tick) >=
            CHASSIS_TURN_STABLE_MS) {
            ChassisBeginStopping(now_ms, CHASSIS_STATE_COMPLETED);
        }
    } else {
        g_chassis_debug.turn_stable_tick = 0u;
    }
}

static void ChassisRunStopping(uint32_t now_ms, float dt_s)
{
    float mean_abs_speed;

    if (chassis_stop_terminal_state == CHASSIS_STATE_CANCELLED) {
        float linear_step = CHASSIS_MAX_LINEAR_ACCEL_M_S2 * dt_s;
        float angular_step = CHASSIS_TURN_MAX_ACCEL_RAD_S2 * dt_s;

        if (chassis_ramped_linear_m_s > linear_step) {
            chassis_ramped_linear_m_s -= linear_step;
        } else if (chassis_ramped_linear_m_s < -linear_step) {
            chassis_ramped_linear_m_s += linear_step;
        } else {
            chassis_ramped_linear_m_s = 0.0f;
        }
        if (chassis_ramped_angular_rad_s > angular_step) {
            chassis_ramped_angular_rad_s -= angular_step;
        } else if (chassis_ramped_angular_rad_s < -angular_step) {
            chassis_ramped_angular_rad_s += angular_step;
        } else {
            chassis_ramped_angular_rad_s = 0.0f;
        }
        g_chassis_debug.linear_command_m_s = chassis_ramped_linear_m_s;
        g_chassis_debug.angular_command_rad_s =
            chassis_ramped_angular_rad_s;
        ChassisSetBodyVelocityTargets(chassis_ramped_linear_m_s,
                                      chassis_ramped_angular_rad_s);
        if (chassis_ramped_linear_m_s != 0.0f ||
            chassis_ramped_angular_rad_s != 0.0f) {
            chassis_stop_stable_tick = 0u;
            return;
        }
    } else {
        ChassisZeroTargets();
    }
    if ((uint32_t)(now_ms - g_chassis_debug.state_tick) >=
        CHASSIS_STOP_TIMEOUT_MS) {
        ChassisLatchFault(CHASSIS_FAULT_TIMEOUT);
        return;
    }
    mean_abs_speed = 0.5f *
        (fabsf(g_chassis_debug.left_speed_m_s) +
         fabsf(g_chassis_debug.right_speed_m_s));
    if (mean_abs_speed > CHASSIS_STOP_SPEED_M_S) {
        chassis_stop_stable_tick = 0u;
        return;
    }
    if (chassis_stop_stable_tick == 0u) {
        chassis_stop_stable_tick = now_ms;
        return;
    }
    if ((uint32_t)(now_ms - chassis_stop_stable_tick) <
        CHASSIS_STOP_STABLE_MS) {
        return;
    }
    DJIMotorStop(chassis_left_motor);
    DJIMotorStop(chassis_right_motor);
    g_chassis_debug.motion_enabled = 0u;
    if (chassis_stop_terminal_state == CHASSIS_STATE_COMPLETED) {
        g_chassis_debug.completed_count++;
        g_chassis_debug.cycle_count++;
    } else {
        g_chassis_debug.cancelled_count++;
    }
    ChassisSetState(chassis_stop_terminal_state, now_ms);
}

static uint8_t ChassisCommandValid(const Chassis_Command_s *command)
{
    if (command == NULL || command->command_id == 0u ||
        !isfinite(command->distance_mm) ||
        !isfinite(command->angle_deg) ||
        !isfinite(command->tolerance_mm) ||
        command->tolerance_mm < CHASSIS_COMMAND_MIN_TOLERANCE_MM ||
        command->tolerance_mm > CHASSIS_COMMAND_MAX_TOLERANCE_MM) {
        return 0u;
    }
    if (command->heading_mode != CHASSIS_HEADING_NONE &&
        command->heading_mode != CHASSIS_HEADING_HOLD_START) {
        return 0u;
    }
    if (command->type == CHASSIS_COMMAND_RELATIVE_STRAIGHT) {
        return fabsf(command->distance_mm) > command->tolerance_mm &&
            fabsf(command->distance_mm) <= CHASSIS_COMMAND_MAX_DISTANCE_MM &&
            command->angle_deg == 0.0f;
    }
    if (command->type == CHASSIS_COMMAND_RELATIVE_TURN) {
        return command->distance_mm == 0.0f &&
            fabsf(command->angle_deg) > CHASSIS_TURN_ERROR_TOLERANCE_DEG &&
            fabsf(command->angle_deg) <= CHASSIS_COMMAND_MAX_TURN_DEG;
    }
    return 0u;
}

static uint8_t ChassisVelocityCommandValid(
    const Chassis_Velocity_Command_s *command)
{
    return (uint8_t)(command != NULL && command->command_id != 0u &&
        isfinite(command->vx_mm_s) && isfinite(command->wz_rad_s) &&
        fabsf(command->vx_mm_s) <= CHASSIS_VELOCITY_MAX_LINEAR_MM_S &&
        fabsf(command->wz_rad_s) <=
            CHASSIS_VELOCITY_MAX_ANGULAR_RAD_S);
}

uint8_t ChassisInit(attitude_t *imu)
{
    Motor_Init_Config_s config;

    memset(&g_chassis_debug, 0, sizeof(g_chassis_debug));
    memset(&config, 0, sizeof(config));
    chassis_imu = imu;
    if (chassis_imu == NULL) {
        g_chassis_debug.fault = CHASSIS_FAULT_INIT;
        g_chassis_debug.state = CHASSIS_STATE_FAULT;
        return 0u;
    }
    config.can_init_config.can_handle = &hcan2;
    config.controller_param_init_config.speed_PID.Kp = CHASSIS_SPEED_PID_KP;
    config.controller_param_init_config.speed_PID.Ki = CHASSIS_SPEED_PID_KI;
    config.controller_param_init_config.speed_PID.Kd = CHASSIS_SPEED_PID_KD;
    config.controller_param_init_config.speed_PID.IntegralLimit =
        CHASSIS_SPEED_PID_I_LIMIT;
    config.controller_param_init_config.speed_PID.MaxOut =
        CHASSIS_MOTOR_CURRENT_LIMIT;
    config.controller_param_init_config.speed_PID.Improve =
        (PID_Improvement_e)(PID_Trapezoid_Intergral |
        PID_Integral_Limit | PID_Derivative_On_Measurement);
    config.controller_param_init_config.current_PID.Kp =
        CHASSIS_CURRENT_PID_KP;
    config.controller_param_init_config.current_PID.Ki =
        CHASSIS_CURRENT_PID_KI;
    config.controller_param_init_config.current_PID.IntegralLimit =
        CHASSIS_CURRENT_PID_I_LIMIT;
    config.controller_param_init_config.current_PID.MaxOut =
        CHASSIS_MOTOR_CURRENT_LIMIT;
    config.controller_param_init_config.current_PID.Improve =
        (PID_Improvement_e)(PID_Trapezoid_Intergral |
        PID_Integral_Limit | PID_Derivative_On_Measurement);
    config.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
    config.controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
    config.controller_setting_init_config.outer_loop_type = SPEED_LOOP;
    config.controller_setting_init_config.close_loop_type =
        (Closeloop_Type_e)(SPEED_LOOP | CURRENT_LOOP);
    config.controller_setting_init_config.motor_reverse_flag =
        MOTOR_DIRECTION_NORMAL;
    config.controller_setting_init_config.feedback_reverse_flag =
        FEEDBACK_DIRECTION_NORMAL;
    config.motor_type = M3508;
    config.can_init_config.tx_id = CHASSIS_LEFT_MOTOR_ID;
    chassis_left_motor = DJIMotorInit(&config);
    config.can_init_config.tx_id = CHASSIS_RIGHT_MOTOR_ID;
    chassis_right_motor = DJIMotorInit(&config);
    if (chassis_left_motor == NULL || chassis_right_motor == NULL) {
        ChassisLatchFault(CHASSIS_FAULT_INIT);
        return 0u;
    }
    DJIMotorStop(chassis_left_motor);
    DJIMotorStop(chassis_right_motor);
    g_chassis_debug.initialized = 1u;
    g_chassis_debug.velocity_wheel_scale = 1.0f;
    g_chassis_debug.left_can_id = CHASSIS_LEFT_MOTOR_ID;
    g_chassis_debug.right_can_id = CHASSIS_RIGHT_MOTOR_ID;
    g_chassis_debug.state = CHASSIS_STATE_WAIT_READY;
    g_chassis_debug.state_tick = HAL_GetTick();
    chassis_last_control_tick = HAL_GetTick();
    return 1u;
}

Chassis_Command_Result_e ChassisSubmitCommand(
    const Chassis_Command_s *command)
{
    Chassis_Command_Result_e result;

    if (g_chassis_debug.initialized == 0u ||
        g_chassis_debug.state == CHASSIS_STATE_WAIT_READY) {
        result = CHASSIS_COMMAND_NOT_READY;
    } else if (g_chassis_debug.state == CHASSIS_STATE_FAULT ||
               g_chassis_debug.fault != CHASSIS_FAULT_NONE) {
        result = CHASSIS_COMMAND_FAULTED;
    } else if (command != NULL && command->command_id != 0u &&
               chassis_latest_command_id != 0u &&
               (int32_t)(command->command_id -
                         chassis_latest_command_id) <= 0) {
        result = CHASSIS_COMMAND_DUPLICATE;
    } else if (g_chassis_debug.state == CHASSIS_STATE_RUNNING ||
               g_chassis_debug.state == CHASSIS_STATE_STOPPING) {
        result = CHASSIS_COMMAND_BUSY;
    } else if (!ChassisCommandValid(command)) {
        result = CHASSIS_COMMAND_INVALID;
    } else {
        chassis_latest_command_id = command->command_id;
        g_chassis_debug.command_id = command->command_id;
        g_chassis_debug.command_type = command->type;
        g_chassis_debug.heading_mode = command->heading_mode;
        g_chassis_debug.target_distance_mm = command->distance_mm;
        g_chassis_debug.target_angle_deg = command->angle_deg;
        g_chassis_debug.tolerance_mm = command->tolerance_mm;
        g_chassis_debug.straight_target_distance_m =
            command->distance_mm * 0.001f;
        g_chassis_debug.straight_tolerance_m =
            command->tolerance_mm * 0.001f;
        g_chassis_debug.velocity_heading_hold_active = 0u;
        g_chassis_debug.velocity_command_tick = 0u;
        g_chassis_debug.velocity_target_vx_mm_s = 0.0f;
        g_chassis_debug.velocity_target_wz_rad_s = 0.0f;
        ChassisBeginCommand(HAL_GetTick());
        result = CHASSIS_COMMAND_ACCEPTED;
    }
    g_chassis_debug.last_submit_result = result;
    return result;
}

Chassis_Command_Result_e ChassisSubmitVelocityCommand(
    const Chassis_Velocity_Command_s *command)
{
    Chassis_Command_Result_e result;
    uint32_t now_ms = HAL_GetTick();

    if (g_chassis_debug.initialized == 0u ||
        g_chassis_debug.state == CHASSIS_STATE_WAIT_READY) {
        result = CHASSIS_COMMAND_NOT_READY;
    } else if (g_chassis_debug.state == CHASSIS_STATE_FAULT ||
               g_chassis_debug.fault != CHASSIS_FAULT_NONE) {
        result = CHASSIS_COMMAND_FAULTED;
    } else if (!ChassisVelocityCommandValid(command)) {
        result = CHASSIS_COMMAND_INVALID;
    } else if (chassis_latest_command_id != 0u &&
               (int32_t)(command->command_id -
                         chassis_latest_command_id) <= 0) {
        result = CHASSIS_COMMAND_DUPLICATE;
    } else if (g_chassis_debug.state == CHASSIS_STATE_STOPPING ||
               (g_chassis_debug.state == CHASSIS_STATE_RUNNING &&
                g_chassis_debug.command_type !=
                    CHASSIS_COMMAND_BODY_VELOCITY)) {
        result = CHASSIS_COMMAND_BUSY;
    } else {
        uint8_t already_running = (uint8_t)(
            g_chassis_debug.state == CHASSIS_STATE_RUNNING &&
            g_chassis_debug.command_type ==
                CHASSIS_COMMAND_BODY_VELOCITY);

        chassis_latest_command_id = command->command_id;
        g_chassis_debug.command_id = command->command_id;
        g_chassis_debug.command_type = CHASSIS_COMMAND_BODY_VELOCITY;
        g_chassis_debug.heading_mode = CHASSIS_HEADING_HOLD_ZERO_WZ;
        g_chassis_debug.target_distance_mm = 0.0f;
        g_chassis_debug.target_angle_deg = 0.0f;
        g_chassis_debug.tolerance_mm = 0.0f;
        g_chassis_debug.straight_target_distance_m = 0.0f;
        g_chassis_debug.straight_tolerance_m = 0.0f;
        if (already_running == 0u) {
            g_chassis_debug.velocity_heading_hold_active = 0u;
            ChassisBeginCommand(now_ms);
        }
        ChassisApplyVelocityTarget(command->vx_mm_s,
                                   command->wz_rad_s, now_ms);
        g_chassis_debug.velocity_refresh_count++;
        result = CHASSIS_COMMAND_ACCEPTED;
    }
    g_chassis_debug.last_submit_result = result;
    return result;
}

uint8_t ChassisGetStatus(Chassis_Status_s *status)
{
    if (status == NULL || g_chassis_debug.initialized == 0u) {
        return 0u;
    }
    status->command_id = g_chassis_debug.command_id;
    status->command_type = g_chassis_debug.command_type;
    status->state = g_chassis_debug.state;
    status->fault = g_chassis_debug.fault;
    status->target_distance_mm = g_chassis_debug.target_distance_mm;
    status->actual_distance_mm = g_chassis_debug.actual_distance_mm;
    status->target_angle_deg = g_chassis_debug.target_angle_deg;
    status->actual_angle_deg = g_chassis_debug.actual_angle_deg;
    status->target_vx_mm_s =
        g_chassis_debug.velocity_target_vx_mm_s;
    status->actual_vx_mm_s =
        g_chassis_debug.velocity_actual_vx_mm_s;
    status->target_wz_rad_s =
        g_chassis_debug.velocity_target_wz_rad_s;
    status->actual_wz_rad_s =
        g_chassis_debug.velocity_actual_wz_rad_s;
    return 1u;
}

void ChassisCancelMotion(void)
{
    if (g_chassis_debug.state == CHASSIS_STATE_RUNNING) {
        ChassisBeginStopping(HAL_GetTick(), CHASSIS_STATE_CANCELLED);
    }
}

uint8_t ChassisFaulted(void)
{
    return g_chassis_debug.fault != CHASSIS_FAULT_NONE ||
        g_chassis_debug.state == CHASSIS_STATE_FAULT;
}

void ChassisNotifyImuUpdate(uint32_t now_ms)
{
    g_chassis_debug.imu_update_tick = now_ms;
}

void ChassisEmergencyStop(void)
{
    ChassisLatchFault(CHASSIS_FAULT_EMERGENCY_STOP);
}

void ChassisTask(uint32_t now_ms)
{
    float dt_s;

    if (g_chassis_debug.initialized == 0u ||
        (uint32_t)(now_ms - chassis_last_control_tick) <
            CHASSIS_CONTROL_PERIOD_MS) {
        return;
    }
    dt_s = (float)(now_ms - chassis_last_control_tick) * 0.001f;
    chassis_last_control_tick = now_ms;
    g_chassis_debug.control_count++;
    g_chassis_debug.imu_healthy = ChassisImuFinite() &&
        (uint32_t)(now_ms - g_chassis_debug.imu_update_tick) <=
            CHASSIS_IMU_UPDATE_TIMEOUT_MS;

    switch (g_chassis_debug.state) {
    case CHASSIS_STATE_WAIT_READY:
        ChassisZeroTargets();
        if (g_chassis_debug.imu_healthy == 0u) {
            chassis_imu_stable_tick = 0u;
            break;
        }
        if (chassis_imu_stable_tick == 0u) {
            chassis_imu_stable_tick = now_ms;
        }
        if ((uint32_t)(now_ms - chassis_imu_stable_tick) >=
                CHASSIS_IMU_STABLE_MS &&
            (uint32_t)(now_ms - g_chassis_debug.state_tick) >=
                CHASSIS_TEST_START_DELAY_MS && ChassisMotorsOnline()) {
            ChassisCaptureZero(now_ms);
            ChassisSetState(CHASSIS_STATE_IDLE, now_ms);
        }
        break;

    case CHASSIS_STATE_RUNNING:
        if (!ChassisFeedbackHealthy()) break;
        ChassisUpdateOdometry();
        if (g_chassis_debug.command_type ==
            CHASSIS_COMMAND_RELATIVE_STRAIGHT) {
            ChassisRunStraight(now_ms, dt_s);
        } else if (g_chassis_debug.command_type ==
                   CHASSIS_COMMAND_RELATIVE_TURN) {
            ChassisRunTurn(now_ms, dt_s);
        } else if (g_chassis_debug.command_type ==
                   CHASSIS_COMMAND_BODY_VELOCITY) {
            ChassisRunVelocity(now_ms, dt_s);
        } else {
            ChassisLatchFault(CHASSIS_FAULT_INIT);
        }
        break;

    case CHASSIS_STATE_STOPPING:
        if (!ChassisFeedbackHealthy()) break;
        ChassisUpdateOdometry();
        ChassisUpdateCommandMeasurements();
        ChassisRunStopping(now_ms, dt_s);
        break;

    case CHASSIS_STATE_IDLE:
    case CHASSIS_STATE_COMPLETED:
    case CHASSIS_STATE_CANCELLED:
        ChassisResetMotion();
        ChassisZeroTargets();
        if (!ChassisFeedbackHealthy()) break;
        ChassisUpdateOdometry();
        break;

    case CHASSIS_STATE_FAULT:
    default:
        ChassisResetMotion();
        ChassisZeroTargets();
        break;
    }
}
