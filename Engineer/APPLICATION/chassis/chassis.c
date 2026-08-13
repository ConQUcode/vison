/**
 * @file chassis.c
 * @brief 双轮编码器里程计、IMU航向完整PID和1m/右转90度循环测试。
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

static float ChassisClamp(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
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

static void ChassisZeroTargets(void)
{
    g_chassis_debug.linear_command_m_s = 0.0f;
    g_chassis_debug.angular_command_rad_s = 0.0f;
    ChassisSetWheelTargets(0.0f, 0.0f);
}

static void ChassisResetMotionRamps(void)
{
    chassis_ramped_linear_m_s = 0.0f;
    chassis_ramped_angular_rad_s = 0.0f;
}

static void ChassisLatchFault(Chassis_Fault_e fault)
{
    if (g_chassis_debug.state != CHASSIS_TEST_FAULT) {
        if (fault == CHASSIS_FAULT_MOTOR_OFFLINE) {
            g_chassis_debug.motor_offline_count++;
        }
        if (fault == CHASSIS_FAULT_IMU_INVALID) {
            g_chassis_debug.imu_fault_count++;
        }
    }
    g_chassis_debug.fault = fault;
    g_chassis_debug.state = CHASSIS_TEST_FAULT;
    g_chassis_debug.motion_enabled = 0u;
    ChassisResetMotionRamps();
    ChassisZeroTargets();
    if (chassis_left_motor != NULL) DJIMotorStop(chassis_left_motor);
    if (chassis_right_motor != NULL) DJIMotorStop(chassis_right_motor);
}

static void ChassisSetState(Chassis_Test_State_e state, uint32_t now_ms)
{
    g_chassis_debug.state = state;
    g_chassis_debug.state_tick = now_ms;
    g_chassis_debug.wait_remaining_ms = 0u;
}

static void ChassisCaptureZero(uint32_t now_ms)
{
    /* IMU和两轮反馈稳定后，在同一时刻建立全局距离、坐标和航向零点。 */
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
    g_chassis_debug.heading_target_deg = 0.0f;
    g_chassis_debug.x_m = 0.0f;
    g_chassis_debug.y_m = 0.0f;
    g_chassis_debug.segment_distance_m = 0.0f;
    g_chassis_debug.state_tick = now_ms;
}

static void ChassisUpdateOdometry(void)
{
    /*
     * 编码器负责平移距离，IMU连续Yaw负责正式航向。轮差Yaw只用于观察
     * 打滑、有效轮距误差和IMU方向，不参与直行或转角闭环。
     */
    float left_distance;
    float right_distance;
    float dl;
    float dr;
    float ds;
    float yaw_deg;
    float yaw_mid_rad;

    g_chassis_debug.left_total_angle_deg =
        chassis_left_motor->measure.total_angle;
    g_chassis_debug.right_total_angle_deg =
        chassis_right_motor->measure.total_angle;
    left_distance = ChassisAngleToDistance(
        g_chassis_debug.left_total_angle_deg,
        chassis_left_zero_angle_deg, CHASSIS_LEFT_FEEDBACK_SIGN);
    right_distance = ChassisAngleToDistance(
        g_chassis_debug.right_total_angle_deg,
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
        chassis_left_motor->measure.speed_aps,
        CHASSIS_LEFT_FEEDBACK_SIGN);
    g_chassis_debug.right_speed_m_s = ChassisMotorDegSToWheelMS(
        chassis_right_motor->measure.speed_aps,
        CHASSIS_RIGHT_FEEDBACK_SIGN);
    g_chassis_debug.imu_gyro_z_rad_s = CHASSIS_IMU_YAW_SIGN *
        chassis_imu->Gyro[Z];
    chassis_last_left_distance_m = left_distance;
    chassis_last_right_distance_m = right_distance;
    chassis_last_imu_yaw_deg = CHASSIS_IMU_YAW_SIGN *
        chassis_imu->YawTotalAngle;
    g_chassis_debug.left_feedback_count =
        chassis_left_motor->feedback_count;
    g_chassis_debug.right_feedback_count =
        chassis_right_motor->feedback_count;
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

static void ChassisResetHeadingPid(void)
{
    g_chassis_debug.heading_pid_integral_deg_s = 0.0f;
    g_chassis_debug.heading_pid_p_rad_s = 0.0f;
    g_chassis_debug.heading_pid_i_rad_s = 0.0f;
    g_chassis_debug.heading_pid_d_rad_s = 0.0f;
    g_chassis_debug.heading_correction_rad_s = 0.0f;
}

static void ChassisResetTurnPid(void)
{
    g_chassis_debug.turn_pid_integral_deg_s = 0.0f;
    g_chassis_debug.turn_pid_p_rad_s = 0.0f;
    g_chassis_debug.turn_pid_i_rad_s = 0.0f;
    g_chassis_debug.turn_pid_d_rad_s = 0.0f;
    g_chassis_debug.turn_output_rad_s = 0.0f;
    g_chassis_debug.turn_stable_tick = 0u;
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
    /* 对测量值求导：直接使用IMU角速度，避免误差差分放大Yaw噪声。 */
    g_chassis_debug.heading_pid_d_rad_s =
        -CHASSIS_HEADING_KD * g_chassis_debug.imu_gyro_z_rad_s;
    raw_output = g_chassis_debug.heading_pid_p_rad_s +
        g_chassis_debug.heading_pid_i_rad_s +
        g_chassis_debug.heading_pid_d_rad_s;
    output = ChassisClamp(raw_output,
        -CHASSIS_HEADING_MAX_OUTPUT_RAD_S,
        CHASSIS_HEADING_MAX_OUTPUT_RAD_S);

    /* 输出沿误差方向饱和时撤销本周期积分，防止饱和后继续积分。 */
    if (raw_output != output && raw_output * error_deg > 0.0f) {
        g_chassis_debug.heading_pid_integral_deg_s = old_integral;
        g_chassis_debug.heading_pid_i_rad_s =
            CHASSIS_HEADING_KI_RAD_S_PER_DEG_S * old_integral;
        raw_output = g_chassis_debug.heading_pid_p_rad_s +
            g_chassis_debug.heading_pid_i_rad_s +
            g_chassis_debug.heading_pid_d_rad_s;
        output = ChassisClamp(raw_output,
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
    g_chassis_debug.turn_pid_i_rad_s =
        CHASSIS_TURN_KI_RAD_S_PER_DEG_S *
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
        raw_output = g_chassis_debug.turn_pid_p_rad_s +
            g_chassis_debug.turn_pid_i_rad_s +
            g_chassis_debug.turn_pid_d_rad_s;
        output = ChassisClamp(raw_output, -CHASSIS_TURN_MAX_RATE_RAD_S,
                              CHASSIS_TURN_MAX_RATE_RAD_S);
    }
    return output;
}

static void ChassisBeginStraight(Chassis_Test_State_e state,
                                 uint32_t now_ms,
                                 float heading_target_deg)
{
    ChassisResetMotionRamps();
    ChassisResetHeadingPid();
    g_chassis_debug.segment_start_left_m =
        g_chassis_debug.left_distance_m;
    g_chassis_debug.segment_start_right_m =
        g_chassis_debug.right_distance_m;
    g_chassis_debug.segment_left_distance_m = 0.0f;
    g_chassis_debug.segment_right_distance_m = 0.0f;
    g_chassis_debug.segment_distance_m = 0.0f;
    g_chassis_debug.heading_target_deg = heading_target_deg;
    g_chassis_debug.test_start_tick = now_ms;
    g_chassis_debug.left_direction_check_start_m = 0.0f;
    g_chassis_debug.right_direction_check_start_m = 0.0f;
    DJIMotorEnable(chassis_left_motor);
    DJIMotorEnable(chassis_right_motor);
    g_chassis_debug.motion_enabled = 1u;
    g_chassis_debug.straight_count++;
    ChassisSetState(state, now_ms);
}

static void ChassisBeginTurn(uint32_t now_ms)
{
    ChassisResetMotionRamps();
    ChassisResetTurnPid();
    g_chassis_debug.turn_start_yaw_deg = g_chassis_debug.yaw_deg;
    g_chassis_debug.turn_target_yaw_deg = g_chassis_debug.yaw_deg +
        CHASSIS_TURN_ANGLE_DEG;
    g_chassis_debug.turn_error_deg = CHASSIS_TURN_ANGLE_DEG;
    g_chassis_debug.test_start_tick = now_ms;
    DJIMotorEnable(chassis_left_motor);
    DJIMotorEnable(chassis_right_motor);
    g_chassis_debug.motion_enabled = 1u;
    ChassisSetState(CHASSIS_TEST_TURN_RIGHT, now_ms);
}

static void ChassisUpdateSegmentDistance(void)
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
}

static void ChassisRunStraight(uint32_t now_ms, float dt_s,
                               Chassis_Test_State_e stop_state)
{
    float remaining_m;
    float desired_linear_m_s;
    float max_linear_step;
    float angular_rad_s;
    float left_m_s;
    float right_m_s;

    ChassisUpdateSegmentDistance();
    if ((uint32_t)(now_ms - g_chassis_debug.test_start_tick) >=
            CHASSIS_DIRECTION_CHECK_MS &&
        (g_chassis_debug.segment_left_distance_m <
             CHASSIS_DIRECTION_CHECK_MIN_M ||
         g_chassis_debug.segment_right_distance_m <
             CHASSIS_DIRECTION_CHECK_MIN_M)) {
        g_chassis_debug.direction_fault_count++;
        ChassisLatchFault(CHASSIS_FAULT_DIRECTION);
        return;
    }
    if ((uint32_t)(now_ms - g_chassis_debug.test_start_tick) >=
        CHASSIS_STRAIGHT_TIMEOUT_MS) {
        ChassisLatchFault(CHASSIS_FAULT_TIMEOUT);
        return;
    }
    if (fabsf(g_chassis_debug.segment_distance_m) >
        CHASSIS_EXCESS_DISTANCE_M) {
        ChassisLatchFault(CHASSIS_FAULT_EXCESS_DISTANCE);
        return;
    }
    remaining_m = CHASSIS_TEST_DISTANCE_M -
        g_chassis_debug.segment_distance_m;
    if (remaining_m <= CHASSIS_DISTANCE_TOLERANCE_M) {
        ChassisResetMotionRamps();
        ChassisZeroTargets();
        chassis_stop_stable_tick = 0u;
        ChassisSetState(stop_state, now_ms);
        return;
    }
    desired_linear_m_s = ChassisClamp(
        CHASSIS_TEST_POSITION_KP * remaining_m,
        CHASSIS_TEST_MIN_SPEED_M_S, CHASSIS_TEST_MAX_SPEED_M_S);
    if (remaining_m < CHASSIS_TEST_DECEL_DISTANCE_M) {
        desired_linear_m_s = ChassisClamp(
            CHASSIS_TEST_MAX_SPEED_M_S * remaining_m /
                CHASSIS_TEST_DECEL_DISTANCE_M,
            CHASSIS_TEST_MIN_SPEED_M_S, CHASSIS_TEST_MAX_SPEED_M_S);
    }
    max_linear_step = CHASSIS_MAX_LINEAR_ACCEL_M_S2 * dt_s;
    if (desired_linear_m_s > chassis_ramped_linear_m_s +
        max_linear_step) {
        chassis_ramped_linear_m_s += max_linear_step;
    } else {
        chassis_ramped_linear_m_s = desired_linear_m_s;
    }
    g_chassis_debug.heading_error_deg =
        g_chassis_debug.heading_target_deg - g_chassis_debug.yaw_deg;
    angular_rad_s = ChassisHeadingPid(
        g_chassis_debug.heading_error_deg, dt_s);
    left_m_s = chassis_ramped_linear_m_s -
        angular_rad_s * CHASSIS_TRACK_WIDTH_M * 0.5f;
    right_m_s = chassis_ramped_linear_m_s +
        angular_rad_s * CHASSIS_TRACK_WIDTH_M * 0.5f;
    g_chassis_debug.linear_command_m_s = chassis_ramped_linear_m_s;
    g_chassis_debug.angular_command_rad_s = angular_rad_s;
    ChassisSetWheelTargets(left_m_s, right_m_s);
}

static void ChassisRunTurn(uint32_t now_ms, float dt_s)
{
    float desired_angular_rad_s;
    float max_angular_step;
    float yaw_travel_deg;
    float left_m_s;
    float right_m_s;

    g_chassis_debug.turn_error_deg = g_chassis_debug.turn_target_yaw_deg -
        g_chassis_debug.yaw_deg;
    yaw_travel_deg = g_chassis_debug.yaw_deg -
        g_chassis_debug.turn_start_yaw_deg;
    if ((uint32_t)(now_ms - g_chassis_debug.test_start_tick) >=
            CHASSIS_TURN_DIRECTION_CHECK_MS &&
        yaw_travel_deg > -CHASSIS_TURN_DIRECTION_CHECK_DEG) {
        g_chassis_debug.turn_direction_fault_count++;
        ChassisLatchFault(CHASSIS_FAULT_TURN_DIRECTION);
        return;
    }
    if ((uint32_t)(now_ms - g_chassis_debug.test_start_tick) >=
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
    left_m_s = -chassis_ramped_angular_rad_s *
        CHASSIS_TRACK_WIDTH_M * 0.5f;
    right_m_s = chassis_ramped_angular_rad_s *
        CHASSIS_TRACK_WIDTH_M * 0.5f;
    ChassisSetWheelTargets(left_m_s, right_m_s);

    if (fabsf(g_chassis_debug.turn_error_deg) <=
            CHASSIS_TURN_ERROR_TOLERANCE_DEG &&
        fabsf(g_chassis_debug.imu_gyro_z_rad_s) <=
            CHASSIS_TURN_GYRO_TOLERANCE_RAD_S) {
        if (g_chassis_debug.turn_stable_tick == 0u) {
            g_chassis_debug.turn_stable_tick = now_ms;
        }
        if ((uint32_t)(now_ms - g_chassis_debug.turn_stable_tick) >=
            CHASSIS_TURN_STABLE_MS) {
            ChassisResetMotionRamps();
            ChassisZeroTargets();
            chassis_stop_stable_tick = 0u;
            ChassisSetState(CHASSIS_TEST_STOP_AFTER_TURN, now_ms);
        }
    } else {
        g_chassis_debug.turn_stable_tick = 0u;
    }
}

static uint8_t ChassisStopSettled(uint32_t now_ms)
{
    float mean_abs_speed;

    ChassisZeroTargets();
    if ((uint32_t)(now_ms - g_chassis_debug.state_tick) >=
        CHASSIS_STOP_TIMEOUT_MS) {
        ChassisLatchFault(CHASSIS_FAULT_TIMEOUT);
        return 0u;
    }
    mean_abs_speed = 0.5f *
        (fabsf(g_chassis_debug.left_speed_m_s) +
         fabsf(g_chassis_debug.right_speed_m_s));
    if (mean_abs_speed > CHASSIS_STOP_SPEED_M_S) {
        chassis_stop_stable_tick = 0u;
        return 0u;
    }
    if (chassis_stop_stable_tick == 0u) {
        chassis_stop_stable_tick = now_ms;
        return 0u;
    }
    if ((uint32_t)(now_ms - chassis_stop_stable_tick) <
        CHASSIS_STOP_STABLE_MS) {
        return 0u;
    }
    DJIMotorStop(chassis_left_motor);
    DJIMotorStop(chassis_right_motor);
    g_chassis_debug.motion_enabled = 0u;
    chassis_stop_stable_tick = 0u;
    return 1u;
}

static uint8_t ChassisWaitFinished(uint32_t now_ms)
{
    uint32_t elapsed_ms = (uint32_t)(now_ms - g_chassis_debug.state_tick);

    ChassisZeroTargets();
    if (elapsed_ms >= CHASSIS_ACTION_WAIT_MS) {
        g_chassis_debug.wait_remaining_ms = 0u;
        return 1u;
    }
    g_chassis_debug.wait_remaining_ms = CHASSIS_ACTION_WAIT_MS - elapsed_ms;
    return 0u;
}

uint8_t ChassisInit(attitude_t *imu)
{
    Motor_Init_Config_s config;

    memset(&g_chassis_debug, 0, sizeof(g_chassis_debug));
    memset(&config, 0, sizeof(config));
    chassis_imu = imu;
    if (chassis_imu == NULL) {
        g_chassis_debug.fault = CHASSIS_FAULT_INIT;
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
    g_chassis_debug.left_can_id = CHASSIS_LEFT_MOTOR_ID;
    g_chassis_debug.right_can_id = CHASSIS_RIGHT_MOTOR_ID;
    g_chassis_debug.state = CHASSIS_AUTO_FORWARD_TEST_ENABLE != 0u ?
        CHASSIS_TEST_WAIT_IMU : CHASSIS_TEST_DISABLED;
    g_chassis_debug.state_tick = HAL_GetTick();
    chassis_last_control_tick = HAL_GetTick();
    return 1u;
}

void ChassisNotifyImuUpdate(uint32_t now_ms)
{
    g_chassis_debug.imu_update_tick = now_ms;
}

void ChassisEmergencyStop(void)
{
    ChassisLatchFault(CHASSIS_FAULT_INIT);
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
        case CHASSIS_TEST_WAIT_IMU:
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
                    CHASSIS_TEST_START_DELAY_MS) {
                ChassisSetState(CHASSIS_TEST_WAIT_MOTORS, now_ms);
            }
            break;

        case CHASSIS_TEST_WAIT_MOTORS:
            ChassisZeroTargets();
            if (g_chassis_debug.imu_healthy == 0u) {
                ChassisLatchFault(CHASSIS_FAULT_IMU_INVALID);
                break;
            }
            if (ChassisMotorsOnline()) {
                ChassisCaptureZero(now_ms);
                ChassisSetState(CHASSIS_TEST_SETTLE_ZERO, now_ms);
            }
            break;

        case CHASSIS_TEST_SETTLE_ZERO:
            ChassisZeroTargets();
            if (!ChassisFeedbackHealthy()) break;
            ChassisUpdateOdometry();
            if ((uint32_t)(now_ms - g_chassis_debug.state_tick) >=
                CHASSIS_ZERO_SETTLE_MS) {
                ChassisCaptureZero(now_ms);
                ChassisBeginStraight(CHASSIS_TEST_STRAIGHT_1, now_ms, 0.0f);
            }
            break;

        case CHASSIS_TEST_STRAIGHT_1:
            if (!ChassisFeedbackHealthy()) break;
            ChassisUpdateOdometry();
            ChassisRunStraight(now_ms, dt_s,
                CHASSIS_TEST_STOP_AFTER_STRAIGHT_1);
            break;

        case CHASSIS_TEST_STOP_AFTER_STRAIGHT_1:
            if (!ChassisFeedbackHealthy()) break;
            ChassisUpdateOdometry();
            if (ChassisStopSettled(now_ms)) {
                ChassisSetState(CHASSIS_TEST_WAIT_AFTER_STRAIGHT_1, now_ms);
            }
            break;

        case CHASSIS_TEST_WAIT_AFTER_STRAIGHT_1:
            if (!ChassisFeedbackHealthy()) break;
            ChassisUpdateOdometry();
            if (ChassisWaitFinished(now_ms)) {
                ChassisBeginTurn(now_ms);
            }
            break;

        case CHASSIS_TEST_TURN_RIGHT:
            if (!ChassisFeedbackHealthy()) break;
            ChassisUpdateOdometry();
            ChassisRunTurn(now_ms, dt_s);
            break;

        case CHASSIS_TEST_STOP_AFTER_TURN:
            if (!ChassisFeedbackHealthy()) break;
            ChassisUpdateOdometry();
            if (ChassisStopSettled(now_ms)) {
                ChassisSetState(CHASSIS_TEST_WAIT_AFTER_TURN, now_ms);
            }
            break;

        case CHASSIS_TEST_WAIT_AFTER_TURN:
            if (!ChassisFeedbackHealthy()) break;
            ChassisUpdateOdometry();
            if (ChassisWaitFinished(now_ms)) {
                ChassisBeginStraight(CHASSIS_TEST_STRAIGHT_2, now_ms,
                    g_chassis_debug.turn_target_yaw_deg);
            }
            break;

        case CHASSIS_TEST_STRAIGHT_2:
            if (!ChassisFeedbackHealthy()) break;
            ChassisUpdateOdometry();
            ChassisRunStraight(now_ms, dt_s,
                CHASSIS_TEST_STOP_AFTER_STRAIGHT_2);
            break;

        case CHASSIS_TEST_STOP_AFTER_STRAIGHT_2:
            if (!ChassisFeedbackHealthy()) break;
            ChassisUpdateOdometry();
            if (ChassisStopSettled(now_ms)) {
                ChassisSetState(CHASSIS_TEST_WAIT_AFTER_STRAIGHT_2, now_ms);
            }
            break;

        case CHASSIS_TEST_WAIT_AFTER_STRAIGHT_2:
            if (!ChassisFeedbackHealthy()) break;
            ChassisUpdateOdometry();
            if (ChassisWaitFinished(now_ms)) {
                g_chassis_debug.cycle_count++;
                ChassisBeginStraight(CHASSIS_TEST_STRAIGHT_1, now_ms,
                    g_chassis_debug.yaw_deg);
            }
            break;

        case CHASSIS_TEST_FAULT:
        case CHASSIS_TEST_DISABLED:
        default:
            ChassisResetMotionRamps();
            ChassisZeroTargets();
            break;
    }
}
