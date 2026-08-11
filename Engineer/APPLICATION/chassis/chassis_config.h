#ifndef __CHASSIS_CONFIG_H_
#define __CHASSIS_CONFIG_H_

/* 置1后上电自动执行一次1 m测试；完成或故障后不会自动重试。 */
#define CHASSIS_AUTO_FORWARD_TEST_ENABLE       1u
/* CAN2主动轮：左ID1、右ID2；以下命令/反馈符号必须架空实测确认。 */
#define CHASSIS_LEFT_MOTOR_ID                  1u
#define CHASSIS_RIGHT_MOTOR_ID                 2u
#define CHASSIS_LEFT_COMMAND_SIGN              1.0f
#define CHASSIS_RIGHT_COMMAND_SIGN           (-1.0f)
#define CHASSIS_LEFT_FEEDBACK_SIGN             1.0f
#define CHASSIS_RIGHT_FEEDBACK_SIGN          (-1.0f)
/* 机械参数：95 mm轮径、19.2032减速比、320 mm轮距（轮距待实测）。 */
#define CHASSIS_WHEEL_RADIUS_M                 0.0475f
#define CHASSIS_REDUCTION_RATIO               19.2032f
#define CHASSIS_TRACK_WIDTH_M                  0.320f
/* 实车确认BMI088航向正方向与底盘逻辑相反，统一翻转Yaw和Z轴角速度。 */
#define CHASSIS_IMU_YAW_SIGN                 (-1.0f)
/* 状态机与1 m测试时序。 */
#define CHASSIS_CONTROL_PERIOD_MS              5u
#define CHASSIS_TEST_START_DELAY_MS         3000u
#define CHASSIS_IMU_STABLE_MS               1000u
#define CHASSIS_ZERO_SETTLE_MS                300u
#define CHASSIS_TEST_TIMEOUT_MS             15000u
#define CHASSIS_STOP_STABLE_MS                300u
#define CHASSIS_IMU_UPDATE_TIMEOUT_MS          20u
/* 距离、速度、减速和加速度参数。 */
#define CHASSIS_TEST_DISTANCE_M                1.000f
#define CHASSIS_TEST_MAX_SPEED_M_S             0.200f
#define CHASSIS_TEST_MIN_SPEED_M_S             0.060f
#define CHASSIS_TEST_DECEL_DISTANCE_M           0.250f
#define CHASSIS_TEST_POSITION_KP                0.80f
#define CHASSIS_MAX_LINEAR_ACCEL_M_S2           0.35f
/* 1m直行测试限制修正量，即使IMU符号未标定也不让任一轮高速反转。 */
#define CHASSIS_MAX_ANGULAR_RAD_S               0.25f
#define CHASSIS_HEADING_KP_RAD_S_PER_DEG         0.025f
#define CHASSIS_HEADING_KD                      0.08f
#define CHASSIS_DISTANCE_TOLERANCE_M             0.010f
#define CHASSIS_STOP_SPEED_M_S                   0.020f
#define CHASSIS_EXCESS_DISTANCE_M                1.300f
#define CHASSIS_DIRECTION_CHECK_MS               800u
#define CHASSIS_DIRECTION_CHECK_MIN_M             0.005f
/* M3508速度环/电流环台架初值，电流输出限制为10000。 */
#define CHASSIS_SPEED_PID_KP                     4.0f
#define CHASSIS_SPEED_PID_KI                     0.20f
#define CHASSIS_SPEED_PID_KD                     0.005f
#define CHASSIS_SPEED_PID_I_LIMIT             3000.0f
#define CHASSIS_CURRENT_PID_KP                   1.0f
#define CHASSIS_CURRENT_PID_KI                   0.01f
#define CHASSIS_CURRENT_PID_I_LIMIT           3000.0f
#define CHASSIS_MOTOR_CURRENT_LIMIT          10000.0f

#endif
