/**
 * @file chassis.h
 * @brief CAN2 双 M3508 差速底盘的通用相对运动命令接口与 Watch 快照。
 */

#ifndef __CHASSIS_H_
#define __CHASSIS_H_

#include "ins_task.h"
#include "stdint.h"

typedef enum {
    CHASSIS_STATE_WAIT_READY = 0,
    CHASSIS_STATE_IDLE,
    CHASSIS_STATE_RUNNING,
    CHASSIS_STATE_STOPPING,
    CHASSIS_STATE_COMPLETED,
    CHASSIS_STATE_CANCELLED,
    CHASSIS_STATE_FAULT
} Chassis_State_e;

typedef enum {
    CHASSIS_COMMAND_NONE = 0,
    CHASSIS_COMMAND_RELATIVE_STRAIGHT,
    CHASSIS_COMMAND_RELATIVE_TURN
} Chassis_Command_Type_e;

typedef enum {
    CHASSIS_HEADING_NONE = 0,
    CHASSIS_HEADING_HOLD_START
} Chassis_Heading_Mode_e;

typedef enum {
    CHASSIS_COMMAND_ACCEPTED = 0,
    CHASSIS_COMMAND_NOT_READY,
    CHASSIS_COMMAND_BUSY,
    CHASSIS_COMMAND_DUPLICATE,
    CHASSIS_COMMAND_INVALID,
    CHASSIS_COMMAND_FAULTED
} Chassis_Command_Result_e;

typedef enum {
    CHASSIS_FAULT_NONE = 0,
    CHASSIS_FAULT_INIT,
    CHASSIS_FAULT_MOTOR_OFFLINE,
    CHASSIS_FAULT_IMU_INVALID,
    CHASSIS_FAULT_TIMEOUT,
    CHASSIS_FAULT_EXCESS_DISTANCE,
    CHASSIS_FAULT_DIRECTION,
    CHASSIS_FAULT_TURN_DIRECTION,
    CHASSIS_FAULT_EMERGENCY_STOP
} Chassis_Fault_e;

typedef struct {
    uint32_t command_id;
    Chassis_Command_Type_e type;
    /* 正距离表示沿物理车头方向前进，负距离表示后退。 */
    float distance_mm;
    /* 正角度表示逻辑 Yaw 增加，负角度表示逻辑 Yaw 减少。 */
    float angle_deg;
    float tolerance_mm;
    Chassis_Heading_Mode_e heading_mode;
} Chassis_Command_s;

typedef struct {
    uint32_t command_id;
    Chassis_Command_Type_e command_type;
    Chassis_State_e state;
    Chassis_Fault_e fault;
    float target_distance_mm;
    float actual_distance_mm;
    float target_angle_deg;
    float actual_angle_deg;
} Chassis_Status_s;

typedef struct {
    uint8_t initialized;
    Chassis_State_e state;
    Chassis_Fault_e fault;
    Chassis_Command_Result_e last_submit_result;
    uint32_t command_id;
    Chassis_Command_Type_e command_type;
    Chassis_Heading_Mode_e heading_mode;
    uint8_t left_online;
    uint8_t right_online;
    uint8_t imu_healthy;
    uint8_t motion_enabled;
    uint8_t left_can_id;
    uint8_t right_can_id;
    /* 旧OneShot Watch字段保留兼容；通用执行器中恒为0。 */
    uint8_t one_shot_straight;
    uint32_t state_tick;
    uint32_t command_start_tick;
    uint32_t test_start_tick;
    uint32_t imu_update_tick;
    uint32_t control_count;
    uint32_t left_feedback_count;
    uint32_t right_feedback_count;
    uint32_t motor_offline_count;
    uint32_t imu_fault_count;
    uint32_t direction_fault_count;
    uint32_t turn_direction_fault_count;
    uint32_t completed_count;
    uint32_t cancelled_count;
    uint32_t cycle_count;
    uint32_t straight_count;
    float target_distance_mm;
    float actual_distance_mm;
    float tolerance_mm;
    float target_angle_deg;
    float actual_angle_deg;
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
    float segment_start_left_m;
    float segment_start_right_m;
    float segment_left_distance_m;
    float segment_right_distance_m;
    float segment_distance_m;
    float straight_target_distance_m;
    float straight_tolerance_m;
    float heading_target_deg;
    float heading_error_deg;
    float heading_pid_integral_deg_s;
    float heading_pid_p_rad_s;
    float heading_pid_i_rad_s;
    float heading_pid_d_rad_s;
    float heading_correction_rad_s;
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

/** 只初始化并等待 IMU/电机就绪，不自动执行运动。 */
uint8_t ChassisInit(attitude_t *imu);
/** 提交单条相对运动命令；水果任务只使用 RELATIVE_STRAIGHT。 */
Chassis_Command_Result_e ChassisSubmitCommand(
    const Chassis_Command_s *command);
uint8_t ChassisGetStatus(Chassis_Status_s *status);
/** 正常停车并以 CANCELLED 结束当前命令。 */
void ChassisCancelMotion(void);
uint8_t ChassisFaulted(void);
/** 允许 1 kHz 调用，内部按 CHASSIS_CONTROL_PERIOD_MS 更新。 */
void ChassisTask(uint32_t now_ms);
void ChassisNotifyImuUpdate(uint32_t now_ms);
/** 立即停机并锁存 CHASSIS_FAULT_EMERGENCY_STOP。 */
void ChassisEmergencyStop(void);

#endif
