#include "chassis.h"

#include "chassis_config.h"
#include "DJI_motor.h"
#include "daemon.h"
#include "remote.h"
#include "stm32f4xx_hal.h"

#include <math.h>
#include <string.h>

#define CHASSIS_PI 3.14159265358979323846f
#define CHASSIS_DEG_TO_RAD (CHASSIS_PI / 180.0f)
#define CHASSIS_RAD_TO_DEG (180.0f / CHASSIS_PI)

Chassis_Debug_s g_chassis_debug;
/* catch.c仍被Keil工程编译但本轮不运行，保留空符号仅满足链接。 */
RC_ctrl_t *rc_cmd;
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
    float left_motor_deg_s = ChassisWheelMSToMotorDegS(left_m_s) *
        CHASSIS_LEFT_COMMAND_SIGN;
    float right_motor_deg_s = ChassisWheelMSToMotorDegS(right_m_s) *
        CHASSIS_RIGHT_COMMAND_SIGN;
    g_chassis_debug.left_target_m_s = left_m_s;
    g_chassis_debug.right_target_m_s = right_m_s;
    g_chassis_debug.left_target_motor_deg_s = left_motor_deg_s;
    g_chassis_debug.right_target_motor_deg_s = right_motor_deg_s;
    if (chassis_left_motor != NULL) DJIMotorSetRef(chassis_left_motor, left_motor_deg_s);
    if (chassis_right_motor != NULL) DJIMotorSetRef(chassis_right_motor, right_motor_deg_s);
}

static void ChassisStopOutputs(void)
{
    chassis_ramped_linear_m_s = 0.0f;
    g_chassis_debug.linear_command_m_s = 0.0f;
    g_chassis_debug.angular_command_rad_s = 0.0f;
    ChassisSetWheelTargets(0.0f, 0.0f);
}

static void ChassisLatchFault(Chassis_Fault_e fault)
{
    if (g_chassis_debug.state != CHASSIS_TEST_FAULT) {
        if (fault == CHASSIS_FAULT_MOTOR_OFFLINE) g_chassis_debug.motor_offline_count++;
        if (fault == CHASSIS_FAULT_IMU_INVALID) g_chassis_debug.imu_fault_count++;
    }
    g_chassis_debug.fault = fault;
    g_chassis_debug.state = CHASSIS_TEST_FAULT;
    g_chassis_debug.motion_enabled = 0u;
    ChassisStopOutputs();
    if (chassis_left_motor != NULL) DJIMotorStop(chassis_left_motor);
    if (chassis_right_motor != NULL) DJIMotorStop(chassis_right_motor);
}

static void ChassisCaptureZero(uint32_t now_ms)
{
    chassis_left_zero_angle_deg = chassis_left_motor->measure.total_angle;
    chassis_right_zero_angle_deg = chassis_right_motor->measure.total_angle;
    chassis_last_left_distance_m = 0.0f;
    chassis_last_right_distance_m = 0.0f;
    chassis_last_imu_yaw_deg = CHASSIS_IMU_YAW_SIGN * chassis_imu->YawTotalAngle;
    g_chassis_debug.yaw_zero_deg = chassis_last_imu_yaw_deg;
    g_chassis_debug.heading_target_deg = 0.0f;
    g_chassis_debug.x_m = 0.0f;
    g_chassis_debug.y_m = 0.0f;
    g_chassis_debug.travel_distance_m = 0.0f;
    g_chassis_debug.state_tick = now_ms;
}

static void ChassisUpdateOdometry(void)
{
    float left_distance, right_distance, dl, dr, ds, yaw_deg, yaw_mid_rad;
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
    g_chassis_debug.travel_distance_m = 0.5f * (left_distance + right_distance);
    g_chassis_debug.left_speed_m_s = ChassisMotorDegSToWheelMS(
        chassis_left_motor->measure.speed_aps, CHASSIS_LEFT_FEEDBACK_SIGN);
    g_chassis_debug.right_speed_m_s = ChassisMotorDegSToWheelMS(
        chassis_right_motor->measure.speed_aps, CHASSIS_RIGHT_FEEDBACK_SIGN);
    g_chassis_debug.imu_gyro_z_rad_s = CHASSIS_IMU_YAW_SIGN * chassis_imu->Gyro[Z];
    chassis_last_left_distance_m = left_distance;
    chassis_last_right_distance_m = right_distance;
    chassis_last_imu_yaw_deg = CHASSIS_IMU_YAW_SIGN * chassis_imu->YawTotalAngle;
    g_chassis_debug.left_feedback_count =
        chassis_left_motor->feedback_count;
    g_chassis_debug.right_feedback_count =
        chassis_right_motor->feedback_count;
}

uint8_t ChassisInit(attitude_t *imu)
{
    Motor_Init_Config_s config;
    memset(&g_chassis_debug, 0, sizeof(g_chassis_debug));
    memset(&config, 0, sizeof(config));
    chassis_imu = imu;
    config.can_init_config.can_handle = &hcan2;
    config.controller_param_init_config.speed_PID.Kp = CHASSIS_SPEED_PID_KP;
    config.controller_param_init_config.speed_PID.Ki = CHASSIS_SPEED_PID_KI;
    config.controller_param_init_config.speed_PID.Kd = CHASSIS_SPEED_PID_KD;
    config.controller_param_init_config.speed_PID.IntegralLimit = CHASSIS_SPEED_PID_I_LIMIT;
    config.controller_param_init_config.speed_PID.MaxOut = CHASSIS_MOTOR_CURRENT_LIMIT;
    config.controller_param_init_config.speed_PID.Improve =
        (PID_Improvement_e)(PID_Trapezoid_Intergral |
        PID_Integral_Limit | PID_Derivative_On_Measurement);
    config.controller_param_init_config.current_PID.Kp = CHASSIS_CURRENT_PID_KP;
    config.controller_param_init_config.current_PID.Ki = CHASSIS_CURRENT_PID_KI;
    config.controller_param_init_config.current_PID.IntegralLimit = CHASSIS_CURRENT_PID_I_LIMIT;
    config.controller_param_init_config.current_PID.MaxOut = CHASSIS_MOTOR_CURRENT_LIMIT;
    config.controller_param_init_config.current_PID.Improve =
        (PID_Improvement_e)(PID_Trapezoid_Intergral |
        PID_Integral_Limit | PID_Derivative_On_Measurement);
    config.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
    config.controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
    config.controller_setting_init_config.outer_loop_type = SPEED_LOOP;
    config.controller_setting_init_config.close_loop_type =
        (Closeloop_Type_e)(SPEED_LOOP | CURRENT_LOOP);
    config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    config.controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL;
    config.motor_type = M3508;
    config.can_init_config.tx_id = CHASSIS_LEFT_MOTOR_ID;
    chassis_left_motor = DJIMotorInit(&config);
    config.can_init_config.tx_id = CHASSIS_RIGHT_MOTOR_ID;
    chassis_right_motor = DJIMotorInit(&config);
    if (chassis_left_motor == NULL || chassis_right_motor == NULL || chassis_imu == NULL) {
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

void ChassisNotifyImuUpdate(uint32_t now_ms) { g_chassis_debug.imu_update_tick = now_ms; }
void ChassisEmergencyStop(void) { ChassisLatchFault(CHASSIS_FAULT_INIT); }

void ChassisTask(uint32_t now_ms)
{
    float dt_s, remaining_m, desired_linear_m_s, max_linear_step;
    float angular_rad_s, left_m_s, right_m_s, mean_abs_speed;
    if (g_chassis_debug.initialized == 0u ||
        (uint32_t)(now_ms - chassis_last_control_tick) < CHASSIS_CONTROL_PERIOD_MS) return;
    dt_s = (float)(now_ms - chassis_last_control_tick) * 0.001f;
    chassis_last_control_tick = now_ms;
    g_chassis_debug.control_count++;
    g_chassis_debug.imu_healthy = ChassisImuFinite() &&
        (uint32_t)(now_ms - g_chassis_debug.imu_update_tick) <= CHASSIS_IMU_UPDATE_TIMEOUT_MS;
    switch (g_chassis_debug.state) {
        case CHASSIS_TEST_WAIT_IMU:
            ChassisStopOutputs();
            if (g_chassis_debug.imu_healthy == 0u) { chassis_imu_stable_tick = 0u; break; }
            if (chassis_imu_stable_tick == 0u) chassis_imu_stable_tick = now_ms;
            if ((uint32_t)(now_ms - chassis_imu_stable_tick) >= CHASSIS_IMU_STABLE_MS &&
                (uint32_t)(now_ms - g_chassis_debug.state_tick) >= CHASSIS_TEST_START_DELAY_MS) {
                g_chassis_debug.state = CHASSIS_TEST_WAIT_MOTORS;
                g_chassis_debug.state_tick = now_ms;
            }
            break;
        case CHASSIS_TEST_WAIT_MOTORS:
            ChassisStopOutputs();
            if (g_chassis_debug.imu_healthy == 0u) { ChassisLatchFault(CHASSIS_FAULT_IMU_INVALID); break; }
            if (ChassisMotorsOnline()) { ChassisCaptureZero(now_ms); g_chassis_debug.state = CHASSIS_TEST_SETTLE_ZERO; }
            break;
        case CHASSIS_TEST_SETTLE_ZERO:
            ChassisStopOutputs();
            if (!ChassisMotorsOnline()) { ChassisLatchFault(CHASSIS_FAULT_MOTOR_OFFLINE); break; }
            if (g_chassis_debug.imu_healthy == 0u) { ChassisLatchFault(CHASSIS_FAULT_IMU_INVALID); break; }
            ChassisUpdateOdometry();
            if ((uint32_t)(now_ms - g_chassis_debug.state_tick) >= CHASSIS_ZERO_SETTLE_MS) {
                ChassisCaptureZero(now_ms);
                DJIMotorEnable(chassis_left_motor); DJIMotorEnable(chassis_right_motor);
                g_chassis_debug.motion_enabled = 1u;
                g_chassis_debug.test_start_tick = now_ms;
                g_chassis_debug.left_direction_check_start_m = 0.0f;
                g_chassis_debug.right_direction_check_start_m = 0.0f;
                g_chassis_debug.state = CHASSIS_TEST_RUNNING;
            }
            break;
        case CHASSIS_TEST_RUNNING:
            if (!ChassisMotorsOnline()) { ChassisLatchFault(CHASSIS_FAULT_MOTOR_OFFLINE); break; }
            if (g_chassis_debug.imu_healthy == 0u) { ChassisLatchFault(CHASSIS_FAULT_IMU_INVALID); break; }
            ChassisUpdateOdometry();
            if ((uint32_t)(now_ms - g_chassis_debug.test_start_tick) >=
                    CHASSIS_DIRECTION_CHECK_MS &&
                (g_chassis_debug.left_distance_m <
                     CHASSIS_DIRECTION_CHECK_MIN_M ||
                 g_chassis_debug.right_distance_m <
                     CHASSIS_DIRECTION_CHECK_MIN_M)) {
                g_chassis_debug.direction_fault_count++;
                ChassisLatchFault(CHASSIS_FAULT_DIRECTION);
                break;
            }
            if ((uint32_t)(now_ms - g_chassis_debug.test_start_tick) >= CHASSIS_TEST_TIMEOUT_MS) { ChassisLatchFault(CHASSIS_FAULT_TIMEOUT); break; }
            if (fabsf(g_chassis_debug.travel_distance_m) > CHASSIS_EXCESS_DISTANCE_M) { ChassisLatchFault(CHASSIS_FAULT_EXCESS_DISTANCE); break; }
            remaining_m = CHASSIS_TEST_DISTANCE_M - g_chassis_debug.travel_distance_m;
            if (remaining_m <= CHASSIS_DISTANCE_TOLERANCE_M) {
                g_chassis_debug.state = CHASSIS_TEST_STOPPING;
                g_chassis_debug.state_tick = now_ms; chassis_stop_stable_tick = 0u;
                ChassisStopOutputs(); break;
            }
            desired_linear_m_s = ChassisClamp(CHASSIS_TEST_POSITION_KP * remaining_m,
                CHASSIS_TEST_MIN_SPEED_M_S, CHASSIS_TEST_MAX_SPEED_M_S);
            if (remaining_m < CHASSIS_TEST_DECEL_DISTANCE_M) {
                desired_linear_m_s = ChassisClamp(CHASSIS_TEST_MAX_SPEED_M_S * remaining_m /
                    CHASSIS_TEST_DECEL_DISTANCE_M, CHASSIS_TEST_MIN_SPEED_M_S, CHASSIS_TEST_MAX_SPEED_M_S);
            }
            max_linear_step = CHASSIS_MAX_LINEAR_ACCEL_M_S2 * dt_s;
            if (desired_linear_m_s > chassis_ramped_linear_m_s + max_linear_step)
                chassis_ramped_linear_m_s += max_linear_step;
            else chassis_ramped_linear_m_s = desired_linear_m_s;
            g_chassis_debug.heading_error_deg = g_chassis_debug.heading_target_deg - g_chassis_debug.yaw_deg;
            angular_rad_s = CHASSIS_HEADING_KP_RAD_S_PER_DEG * g_chassis_debug.heading_error_deg -
                CHASSIS_HEADING_KD * g_chassis_debug.imu_gyro_z_rad_s;
            angular_rad_s = ChassisClamp(angular_rad_s, -CHASSIS_MAX_ANGULAR_RAD_S, CHASSIS_MAX_ANGULAR_RAD_S);
            left_m_s = chassis_ramped_linear_m_s - angular_rad_s * CHASSIS_TRACK_WIDTH_M * 0.5f;
            right_m_s = chassis_ramped_linear_m_s + angular_rad_s * CHASSIS_TRACK_WIDTH_M * 0.5f;
            g_chassis_debug.linear_command_m_s = chassis_ramped_linear_m_s;
            g_chassis_debug.angular_command_rad_s = angular_rad_s;
            g_chassis_debug.heading_correction_rad_s = angular_rad_s;
            ChassisSetWheelTargets(left_m_s, right_m_s);
            break;
        case CHASSIS_TEST_STOPPING:
            ChassisSetWheelTargets(0.0f, 0.0f);
            if (!ChassisMotorsOnline()) { ChassisLatchFault(CHASSIS_FAULT_MOTOR_OFFLINE); break; }
            if (g_chassis_debug.imu_healthy == 0u) { ChassisLatchFault(CHASSIS_FAULT_IMU_INVALID); break; }
            ChassisUpdateOdometry();
            mean_abs_speed = 0.5f * (fabsf(g_chassis_debug.left_speed_m_s) + fabsf(g_chassis_debug.right_speed_m_s));
            if (mean_abs_speed <= CHASSIS_STOP_SPEED_M_S) {
                if (chassis_stop_stable_tick == 0u) chassis_stop_stable_tick = now_ms;
                if ((uint32_t)(now_ms - chassis_stop_stable_tick) >= CHASSIS_STOP_STABLE_MS) {
                    DJIMotorStop(chassis_left_motor); DJIMotorStop(chassis_right_motor);
                    g_chassis_debug.motion_enabled = 0u;
                    g_chassis_debug.state = CHASSIS_TEST_COMPLETED;
                    g_chassis_debug.state_tick = now_ms;
                }
            } else chassis_stop_stable_tick = 0u;
            break;
        case CHASSIS_TEST_COMPLETED:
        case CHASSIS_TEST_FAULT:
        case CHASSIS_TEST_DISABLED:
        default:
            ChassisStopOutputs();
            break;
    }
}
