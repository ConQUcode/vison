/**
 * @file arm_host.h
 * @brief 机械臂与上位机桥之间的稳定命令和状态数据结构。
 */

#ifndef __ARM_HOST_H__
#define __ARM_HOST_H__

#include "stdint.h"

/*
 * 机械臂对上位机/通信层的稳定应用接口。
 * 本文件不暴露达妙电机、CAN、轨迹缓存或RTOS实现；通信层只负责填充
 * Arm_Command_s、调用ArmSubmitCommand()并周期读取ArmGetHostStatus()。
 */

typedef enum {
    ARM_CONTROL_POINT_WRIST_CENTER = 0, /* ID1俯仰舵机输出轴中心。 */
    ARM_CONTROL_POINT_TOOL_CENTER,      /* 夹爪中心，正式业务控制点。 */
    /* 兼容旧源码名称；线值与TOOL_CENTER相同，不新增协议语义。 */
    ARM_CONTROL_POINT_TOOL_TIP = ARM_CONTROL_POINT_TOOL_CENTER
} Arm_Control_Point_e;

typedef enum {
    ARM_MOVE_DIRECT = 0,
    ARM_MOVE_LINEAR
} Arm_Move_Type_e;

typedef enum {
    ARM_CARTESIAN_SAFETY_NORMAL = 0,
    /* AC地面侧抓：低位允许底座进入侧后方，但仍限制在+/-115deg内。 */
    ARM_CARTESIAN_SAFETY_AC_SIDE_PICK
} Arm_Cartesian_Safety_Profile_e;

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
    /* 可选路径引导点；只改变经过方向，不在该点停车或重新提交命令。 */
    uint8_t waypoint_valid;
    float waypoint_q_deg[3];
    /* 非0时，关节轨迹同步把ID1移动到相对小臂的指定角度。 */
    uint8_t tool_relative_pitch_valid;
    float tool_relative_pitch_deg;
} Arm_Command_Joint_s;

typedef struct {
    Arm_Control_Point_e control_point;
    Arm_Move_Type_e move_type;
    /* 目标坐标单位mm；具体是ID1轴心或夹爪中心由control_point决定。 */
    Arm_Position_s target_mm;
    float max_speed_mm_s;
    uint8_t tool_pitch_valid; /* 0：锁存并保持当前绝对俯仰；非0：使用下字段。 */
    float tool_pitch_deg;     /* 夹爪中心线的世界绝对俯仰角，单位deg。 */
    uint8_t tool_yaw_valid;   /* 兼容保留；当前必须为0，否则返回UNSUPPORTED。 */
    float tool_yaw_deg;       /* 兼容保留，不再控制ID2。 */
    Arm_Cartesian_Safety_Profile_e safety_profile;
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
    ARM_TOOL_ACTION_NONE = 0,       /* 不执行工具动作。 */
    ARM_TOOL_ACTION_SET_PITCH,      /* 只调整ID1绝对俯仰。 */
    ARM_TOOL_ACTION_GRIPPER_READY,  /* ID2回到配置的默认位置。 */
    ARM_TOOL_ACTION_GRIPPER_OPEN,   /* ID2回到配置的默认张开位置。 */
    ARM_TOOL_ACTION_GRIPPER_CLOSE,  /* ID2到660并启用堵转及分级卸力。 */
    ARM_TOOL_ACTION_RESET_SAFE      /* 清除可恢复工具故障并回安全状态。 */
} Arm_Tool_Action_e;

typedef struct {
    Arm_Tool_Action_e action;
    float pitch_deg;
    uint8_t gripper_command;
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
    Arm_Position_s position_mm;        /* 当前ID1舵机轴心坐标。 */
    /* 当前运动目标；坐标语义由提交命令的control_point决定。 */
    Arm_Position_s target_position_mm;
    uint8_t tool_ready;
    uint8_t servo_online[2];
    uint16_t servo_target_pos[2];
    float tool_pitch_target_deg;
    float tool_pitch_feedback_deg;
    uint16_t tool_pitch_servo_pos;
    uint8_t gripper_state;
    uint8_t gripper_target_state;
    uint16_t gripper_target_pos;
    uint16_t gripper_feedback_pos;
    int16_t gripper_position_error;
    uint8_t gripper_stall_candidate;
    uint8_t gripper_stall_latched;
    Arm_Position_s wrist_center_mm; /* ID1俯仰舵机输出轴中心。 */
    Arm_Position_s tool_tip_mm;     /* 按配置工具长度计算的夹爪中心。 */
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
