/**
 * @file chassis_config.h
 * @brief 双轮底盘通用相对运动的机械参数、方向、时序、PID 和安全边界。
 */

#ifndef __CHASSIS_CONFIG_H_
#define __CHASSIS_CONFIG_H_

/* CAN2主动轮：左ID1、右ID2；以下命令/反馈符号必须架空实测确认。 */
#define CHASSIS_LEFT_MOTOR_ID                  1u
#define CHASSIS_RIGHT_MOTOR_ID                 2u
#define CHASSIS_LEFT_COMMAND_SIGN            (-1.0f)
#define CHASSIS_RIGHT_COMMAND_SIGN             1.0f
#define CHASSIS_LEFT_FEEDBACK_SIGN           (-1.0f)
#define CHASSIS_RIGHT_FEEDBACK_SIGN            1.0f
/* 机械参数：95 mm轮径、19.2032减速比、320 mm轮距（轮距待实测）。 */
#define CHASSIS_WHEEL_RADIUS_M                 0.0475f
#define CHASSIS_REDUCTION_RATIO               19.2032f
#define CHASSIS_TRACK_WIDTH_M                  0.320f
/* 当前实测方向：逆时针转动车体时逻辑Yaw应增加；若相反只修改此符号。 */
#define CHASSIS_IMU_YAW_SIGN                (-1.0f)
/* 状态机时序。 */
#define CHASSIS_CONTROL_PERIOD_MS              5u
#define CHASSIS_TEST_START_DELAY_MS         3000u
#define CHASSIS_IMU_STABLE_MS               1000u
#define CHASSIS_ZERO_SETTLE_MS                300u
#define CHASSIS_STRAIGHT_TIMEOUT_MS         15000u
#define CHASSIS_TURN_TIMEOUT_MS              6000u
#define CHASSIS_STOP_STABLE_MS                300u
#define CHASSIS_STOP_TIMEOUT_MS              2000u
#define CHASSIS_IMU_UPDATE_TIMEOUT_MS          20u
/* 命令边界。应用和上位机使用mm，控制器内部换算为m。 */
#define CHASSIS_COMMAND_MAX_DISTANCE_MM       5000.0f
#define CHASSIS_COMMAND_MAX_TURN_DEG           360.0f
#define CHASSIS_COMMAND_MIN_TOLERANCE_MM         0.5f
#define CHASSIS_COMMAND_MAX_TOLERANCE_MM        50.0f
/* 连续车体速度接口：应用层使用mm/s和rad/s，内部统一换算为SI单位。 */
#define CHASSIS_VELOCITY_MAX_LINEAR_MM_S      1500.0f
#define CHASSIS_VELOCITY_MAX_ANGULAR_RAD_S       2.00f
#define CHASSIS_VELOCITY_LINEAR_ZERO_MM_S         0.5f
#define CHASSIS_VELOCITY_ANGULAR_ZERO_RAD_S       0.005f
#define CHASSIS_VELOCITY_COMMAND_TIMEOUT_MS      300u
/* 距离、速度、减速和加速度参数；首次落地不要提高最大速度。 */
#define CHASSIS_TEST_DISTANCE_M                1.000f
#define CHASSIS_TEST_MAX_SPEED_M_S             0.200f
#define CHASSIS_TEST_MIN_SPEED_M_S             0.060f
#define CHASSIS_TEST_DECEL_DISTANCE_M           0.250f
#define CHASSIS_TEST_POSITION_KP                0.80f
#define CHASSIS_MAX_LINEAR_ACCEL_M_S2           3.00f
/*
 * 直行航向完整PID。误差单位deg，积分单位deg*s，输出单位rad/s。
 * 积分只在小误差区工作并有限幅，避免启动或受阻时积累过大修正。
 */
#define CHASSIS_HEADING_KP_RAD_S_PER_DEG         0.150f
#define CHASSIS_HEADING_KI_RAD_S_PER_DEG_S       0.020f
#define CHASSIS_HEADING_KD                       0.150f
#define CHASSIS_HEADING_INTEGRAL_ZONE_DEG        8.0f
#define CHASSIS_HEADING_INTEGRAL_LIMIT_DEG_S    12.0f
#define CHASSIS_HEADING_MAX_OUTPUT_RAD_S         1.00f
/* 右转90度使用独立PID，避免直行高增益直接作用于90度大误差。 */
#define CHASSIS_TURN_ANGLE_DEG                 (-90.0f)
#define CHASSIS_TURN_KP_RAD_S_PER_DEG            0.035f
#define CHASSIS_TURN_KI_RAD_S_PER_DEG_S          0.003f
#define CHASSIS_TURN_KD                          0.180f
#define CHASSIS_TURN_INTEGRAL_ZONE_DEG          15.0f
#define CHASSIS_TURN_INTEGRAL_LIMIT_DEG_S       20.0f
#define CHASSIS_TURN_MAX_RATE_RAD_S               2.00f
#define CHASSIS_TURN_MAX_ACCEL_RAD_S2             8.00f
#define CHASSIS_TURN_ERROR_TOLERANCE_DEG           1.50f
#define CHASSIS_TURN_GYRO_TOLERANCE_RAD_S          0.0873f
#define CHASSIS_TURN_STABLE_MS                    150u
#define CHASSIS_TURN_DIRECTION_CHECK_MS           800u
#define CHASSIS_TURN_DIRECTION_CHECK_DEG            5.0f
#define CHASSIS_MAX_WHEEL_SPEED_M_S                 2.00f
#define CHASSIS_DISTANCE_TOLERANCE_M             0.010f
#define CHASSIS_STOP_SPEED_M_S                   0.020f
#define CHASSIS_EXCESS_DISTANCE_M                1.300f
#define CHASSIS_DIRECTION_CHECK_MS               800u
#define CHASSIS_DIRECTION_CHECK_MIN_M             0.005f
/* M3508速度环/电流环台架初值；先架空确认方向，再调整增益或电流上限。 */
#define CHASSIS_SPEED_PID_KP                     4.0f
#define CHASSIS_SPEED_PID_KI                     0.20f
#define CHASSIS_SPEED_PID_KD                     0.005f
#define CHASSIS_SPEED_PID_I_LIMIT             3000.0f
#define CHASSIS_CURRENT_PID_KP                   1.0f
#define CHASSIS_CURRENT_PID_KI                   0.01f
#define CHASSIS_CURRENT_PID_I_LIMIT           3000.0f
#define CHASSIS_MOTOR_CURRENT_LIMIT          10000.0f

#endif
