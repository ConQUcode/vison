#ifndef __ARM_HOST_H__
#define __ARM_HOST_H__

#include "stdint.h"

/*
 * 机械臂对上位机/通信层的稳定应用接口。
 * 本文件不暴露达妙电机、CAN、轨迹缓存或RTOS实现；通信层只负责填充
 * Arm_Command_s、调用ArmSubmitCommand()并周期读取ArmGetHostStatus()。
 */

typedef enum {
    ARM_CONTROL_POINT_WRIST_CENTER = 0,
    ARM_CONTROL_POINT_TOOL_TIP
} Arm_Control_Point_e;

typedef enum {
    ARM_MOVE_DIRECT = 0,
    ARM_MOVE_LINEAR
} Arm_Move_Type_e;

typedef enum {
    ARM_COMMAND_OK = 0,
    ARM_COMMAND_BUSY,
    ARM_COMMAND_NOT_READY,
    ARM_COMMAND_INVALID,
    ARM_COMMAND_UNSUPPORTED,
    ARM_COMMAND_MODE_DENIED,
    ARM_COMMAND_PREFLIGHT_FAILED,
    ARM_COMMAND_DUPLICATE,
    ARM_COMMAND_TIMEOUT
} Arm_Command_Result_e;

typedef enum {
    ARM_COMMAND_TYPE_NONE = 0,
    ARM_COMMAND_TYPE_JOINT,
    ARM_COMMAND_TYPE_CARTESIAN,
    ARM_COMMAND_TYPE_REALTIME_CARTESIAN,
    ARM_COMMAND_TYPE_TOOL,
    ARM_COMMAND_TYPE_STOP_REALTIME,
    ARM_COMMAND_TYPE_CANCEL_MOTION,
    ARM_COMMAND_TYPE_EMERGENCY_STOP,
    ARM_COMMAND_TYPE_FAULT_RESET
} Arm_Command_Type_e;

typedef enum {
    ARM_COMMAND_STATE_NONE = 0,
    ARM_COMMAND_STATE_QUEUED,
    ARM_COMMAND_STATE_RUNNING,
    ARM_COMMAND_STATE_COMPLETED,
    ARM_COMMAND_STATE_REJECTED,
    ARM_COMMAND_STATE_CANCELLED,
    ARM_COMMAND_STATE_FAULTED
} Arm_Command_State_e;

typedef enum {
    ARM_HOST_STATE_STARTING = 0,
    ARM_HOST_STATE_READY,
    ARM_HOST_STATE_MOVING,
    ARM_HOST_STATE_REALTIME,
    ARM_HOST_STATE_FAULT,
    ARM_HOST_STATE_ESTOP
} Arm_Host_State_e;

typedef struct {
    float x_mm;
    float y_mm;
    float z_mm;
} Arm_Position_s;

typedef struct {
    Arm_Move_Type_e move_type;
    float q_deg[3];
} Arm_Command_Joint_s;

typedef struct {
    Arm_Control_Point_e control_point;
    Arm_Move_Type_e move_type;
    Arm_Position_s target_mm;
    float max_speed_mm_s;
    uint8_t tool_pitch_valid;
    float tool_pitch_deg;
    uint8_t tool_yaw_valid;
    float tool_yaw_deg;
} Arm_Command_Cartesian_s;

typedef struct {
    Arm_Control_Point_e control_point;
    Arm_Position_s target_mm;
    float max_speed_mm_s;
    float max_acceleration_mm_s2;
    uint8_t tool_pitch_valid;
    float tool_pitch_deg;
    uint8_t tool_yaw_valid;
    float tool_yaw_deg;
} Arm_Command_Realtime_s;

typedef enum {
    ARM_TOOL_ACTION_NONE = 0,
    ARM_TOOL_ACTION_MAGNET_ON,
    ARM_TOOL_ACTION_MAGNET_OFF,
    ARM_TOOL_ACTION_SERVO1_ANGLE,
    ARM_TOOL_ACTION_SERVO2_ANGLE,
    ARM_TOOL_ACTION_SERVO2_WORLD_YAW,
    ARM_TOOL_ACTION_RESET_DEFAULT
} Arm_Tool_Action_e;

typedef struct {
    Arm_Tool_Action_e action;
    float servo1_deg;
    float servo2_deg;
} Arm_Command_Tool_s;

typedef union {
    Arm_Command_Joint_s joint;
    Arm_Command_Cartesian_s cartesian;
    Arm_Command_Realtime_s realtime;
    Arm_Command_Tool_s tool;
} Arm_Command_Payload_u;

typedef struct {
    uint32_t command_id; /* 由上位机单调递增；0保留为无命令。 */
    Arm_Command_Type_e type;
    Arm_Command_Payload_u payload;
} Arm_Command_s;

typedef struct {
    uint32_t update_count;
    Arm_Host_State_e state;
    uint8_t ready;
    uint8_t busy;
    uint8_t realtime_active;
    uint8_t realtime_timed_out;
    uint8_t command_pending;
    uint8_t motor_online[3];
    uint8_t motor_enabled[3];
    uint32_t fault_code;
    uint32_t fault_reset_result;

    uint32_t pending_command_id;
    Arm_Command_Type_e pending_command_type;
    uint32_t active_command_id;
    Arm_Command_Type_e active_command_type;
    Arm_Command_State_e active_command_state;
    uint32_t last_command_id;
    Arm_Command_Type_e last_command_type;
    Arm_Command_State_e last_command_state;
    Arm_Command_Result_e last_command_result;
    uint32_t interrupted_command_id;
    Arm_Command_Type_e interrupted_command_type;
    Arm_Command_State_e interrupted_command_state;

    float q_feedback_deg[3];
    float q_target_deg[3];
    Arm_Position_s position_mm;
    Arm_Position_s target_position_mm;
    uint8_t tool_ready;
    uint8_t magnet_on;
    uint8_t servo_online[2];
    float servo_target_deg[2];
    uint16_t servo_target_pos[2];
    uint8_t tool_yaw_active;
    float tool_yaw_target_deg;
    Arm_Position_s wrist_center_mm;
    Arm_Position_s tool_tip_mm;
    uint8_t tool_vertical_down_enabled;
    uint32_t tool_error_code;
    float trajectory_progress;
    float mos_temperature_c[3];
    float rotor_temperature_c[3];
} Arm_Host_Status_s;

/*
 * 非阻塞提交：只做基础格式/模式检查并写入单槽邮箱。
 * ARM_COMMAND_OK表示已受理，不表示轨迹已经完成；执行结果读取状态快照。
 * command_id必须由上位机递增；已受理过的相同或旧ID会返回DUPLICATE。
 * 该结构是固件内API，通信层必须逐字段序列化，禁止直接memcpy为线协议。
 */
Arm_Command_Result_e ArmSubmitCommand(const Arm_Command_s *command);

/* 将当前上位机状态复制到调用者缓冲区；返回0表示参数无效。 */
uint8_t ArmGetHostStatus(Arm_Host_Status_s *status);

/* 仅供调试器Watch直接观察；通信代码优先使用ArmGetHostStatus()复制快照。 */
extern Arm_Host_Status_s g_arm_host_status;

#endif
