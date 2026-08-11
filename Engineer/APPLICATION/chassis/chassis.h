#ifndef __CHASSIS_H_
#define __CHASSIS_H_

#include "ins_task.h"
#include "stdint.h"

typedef enum {
    CHASSIS_TEST_DISABLED = 0,
    CHASSIS_TEST_WAIT_IMU,
    CHASSIS_TEST_WAIT_MOTORS,
    CHASSIS_TEST_SETTLE_ZERO,
    CHASSIS_TEST_RUNNING,
    CHASSIS_TEST_STOPPING,
    CHASSIS_TEST_COMPLETED,
    CHASSIS_TEST_FAULT
} Chassis_Test_State_e;

typedef enum {
    CHASSIS_FAULT_NONE = 0,
    CHASSIS_FAULT_INIT,
    CHASSIS_FAULT_MOTOR_OFFLINE,
    CHASSIS_FAULT_IMU_INVALID,
    CHASSIS_FAULT_TIMEOUT,
    CHASSIS_FAULT_EXCESS_DISTANCE,
    CHASSIS_FAULT_DIRECTION
} Chassis_Fault_e;

typedef struct {
    /* 初始化、状态机、在线与故障信息。 */
    uint8_t initialized;
    Chassis_Test_State_e state;
    Chassis_Fault_e fault;
    uint8_t left_online;
    uint8_t right_online;
    uint8_t imu_healthy;
    uint8_t motion_enabled;
    uint8_t left_can_id;
    uint8_t right_can_id;
    uint32_t state_tick;
    uint32_t test_start_tick;
    uint32_t imu_update_tick;
    uint32_t control_count;
    uint32_t left_feedback_count;
    uint32_t right_feedback_count;
    uint32_t motor_offline_count;
    uint32_t imu_fault_count;
    uint32_t direction_fault_count;
    /* 编码器原始累计角、轮距和IMU/轮差里程计。 */
    float left_total_angle_deg;
    float right_total_angle_deg;
    float left_distance_m;
    float right_distance_m;
    float travel_distance_m;
    float x_m;
    float y_m;
    float yaw_deg;
    float wheel_yaw_deg;
    float imu_wheel_yaw_error_deg;
    float yaw_zero_deg;
    /* 直行航向保持和左右轮目标/反馈。 */
    float heading_target_deg;
    float heading_error_deg;
    float heading_correction_rad_s;
    float linear_command_m_s;
    float angular_command_rad_s;
    float left_target_m_s;
    float right_target_m_s;
    float left_target_motor_deg_s;
    float right_target_motor_deg_s;
    float left_speed_m_s;
    float right_speed_m_s;
    float imu_gyro_z_rad_s;
    float left_direction_check_start_m;
    float right_direction_check_start_m;
} Chassis_Debug_s;

extern Chassis_Debug_s g_chassis_debug;

/* 调度器启动前调用一次：注册CAN2 M3508 ID1/ID2并绑定INS姿态快照。 */
uint8_t ChassisInit(attitude_t *imu);
/* 允许1 kHz调用，函数内部按CHASSIS_CONTROL_PERIOD_MS执行控制和里程计。 */
void ChassisTask(uint32_t now_ms);
/* 每次INS_Task完成后调用，用于IMU新鲜度监督。 */
void ChassisNotifyImuUpdate(uint32_t now_ms);
/* 立即停两轮并锁存故障；不会自动重新启动测试。 */
void ChassisEmergencyStop(void);

#endif
