/**
 * @file chassis.h
 * @brief CAN2 双 M3508 差速底盘的直行/右转循环测试状态、故障和 Watch 快照。
 */

#ifndef __CHASSIS_H_
#define __CHASSIS_H_

#include "ins_task.h"
#include "stdint.h"

typedef enum {
    CHASSIS_TEST_DISABLED = 0,
    CHASSIS_TEST_WAIT_IMU,
    CHASSIS_TEST_WAIT_MOTORS,
    CHASSIS_TEST_SETTLE_ZERO,
    CHASSIS_TEST_STRAIGHT_1,
    CHASSIS_TEST_STOP_AFTER_STRAIGHT_1,
    CHASSIS_TEST_WAIT_AFTER_STRAIGHT_1,
    CHASSIS_TEST_TURN_RIGHT,
    CHASSIS_TEST_STOP_AFTER_TURN,
    CHASSIS_TEST_WAIT_AFTER_TURN,
    CHASSIS_TEST_STRAIGHT_2,
    CHASSIS_TEST_STOP_AFTER_STRAIGHT_2,
    CHASSIS_TEST_WAIT_AFTER_STRAIGHT_2,
    CHASSIS_TEST_FAULT
} Chassis_Test_State_e;

typedef enum {
    CHASSIS_FAULT_NONE = 0,
    CHASSIS_FAULT_INIT,
    CHASSIS_FAULT_MOTOR_OFFLINE,
    CHASSIS_FAULT_IMU_INVALID,
    CHASSIS_FAULT_TIMEOUT,
    CHASSIS_FAULT_EXCESS_DISTANCE,
    CHASSIS_FAULT_DIRECTION,
    CHASSIS_FAULT_TURN_DIRECTION
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
    uint32_t turn_direction_fault_count;
    uint32_t cycle_count;
    uint32_t straight_count;
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
    /* 当前直行段相对距离；x/y和左右总距离在循环中持续累计。 */
    float segment_start_left_m;
    float segment_start_right_m;
    float segment_left_distance_m;
    float segment_right_distance_m;
    float segment_distance_m;
    /* 直行航向完整PID及左右轮目标/反馈。 */
    float heading_target_deg;
    float heading_error_deg;
    float heading_pid_integral_deg_s;
    float heading_pid_p_rad_s;
    float heading_pid_i_rad_s;
    float heading_pid_d_rad_s;
    float heading_correction_rad_s;
    /* 右转90度完整PID、连续Yaw目标和稳定窗口。 */
    float turn_start_yaw_deg;
    float turn_target_yaw_deg;
    float turn_error_deg;
    float turn_pid_integral_deg_s;
    float turn_pid_p_rad_s;
    float turn_pid_i_rad_s;
    float turn_pid_d_rad_s;
    float turn_output_rad_s;
    uint32_t turn_stable_tick;
    uint32_t wait_remaining_ms;
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

/**
 * @brief 注册 CAN2 M3508 ID1/ID2，并绑定只读 INS 姿态快照。
 * @param imu INS_Init() 返回的共享姿态指针，不得为 NULL。
 * @return 1 初始化成功；0 参数、电机注册或配置失败。
 */
uint8_t ChassisInit(attitude_t *imu);
/** 允许 1 kHz 调用，内部按 CHASSIS_CONTROL_PERIOD_MS 更新控制和里程计。 */
void ChassisTask(uint32_t now_ms);
/** 每次 INS_Task 完成后调用，用于 IMU 数据新鲜度监督。 */
void ChassisNotifyImuUpdate(uint32_t now_ms);
/** 立即停两轮并锁存故障；循环测试不会自动恢复。 */
void ChassisEmergencyStop(void);

#endif
