#ifndef __ARM_TOOL_H__
#define __ARM_TOOL_H__

#include "arm_host.h"
#include <stdint.h>

typedef enum {
    ARM_TOOL_ERROR_NONE = 0,          /* 无工具层故障。 */
    ARM_TOOL_ERROR_INVALID_ARGUMENT,  /* 参数为空、非有限数或命令枚举无效。 */
    ARM_TOOL_ERROR_SERVO_RANGE,       /* ID1/ID2目标超出软件限位。 */
    ARM_TOOL_ERROR_SERVO_TX,          /* USART6控制板发送失败。 */
    ARM_TOOL_ERROR_SERVO_FEEDBACK,    /* 位置反馈无效或离线。 */
    ARM_TOOL_ERROR_SERVO_TIMEOUT,     /* 动作在截止时间内未完成。 */
    ARM_TOOL_ERROR_GRIPPER_STALL,     /* 夹爪卡死或抓取闭合受阻。 */
    ARM_TOOL_ERROR_GEOMETRY           /* 30 mm工具中心换算参数无效。 */
} Arm_Tool_Error_e;

typedef enum {
    ARM_TOOL_INIT_DISABLED = 0,
    ARM_TOOL_INIT_WAIT_FEEDBACK,
    ARM_TOOL_INIT_COMMAND_BOOT,
    ARM_TOOL_INIT_WAIT_BOOT,
    ARM_TOOL_INIT_RELIEVING,
    ARM_TOOL_INIT_DONE,
    ARM_TOOL_INIT_ERROR
} Arm_Tool_Init_State_e;

typedef enum {
    ARM_GRIPPER_UNKNOWN = 0,       /* 尚无有效反馈或状态未建立。 */
    ARM_GRIPPER_BOOTING,           /* 上电移动到默认位置550。 */
    ARM_GRIPPER_READYING,          /* 移动到等待抓取位置550。 */
    ARM_GRIPPER_READY,             /* 已在默认位置550。 */
    ARM_GRIPPER_OPENING,           /* 正在打开到默认位置550。 */
    ARM_GRIPPER_OPEN,              /* 已在默认位置550。 */
    ARM_GRIPPER_CLOSING,           /* 正在闭合到抓取位置630。 */
    ARM_GRIPPER_CONTACT_SUSPECTED, /* 已检测到位置停滞，尚未完成卸力。 */
    ARM_GRIPPER_RELIEVING,         /* 从停滞位置回退10个控制值。 */
    ARM_GRIPPER_HELD_CONTACT,      /* 受阻后卸力完成，不代表检测到夹持力。 */
    ARM_GRIPPER_CLOSED_EMPTY,      /* 正常到达630，未检测到提前接触。 */
    ARM_GRIPPER_JAMMED,            /* 行程不足20即停滞，判定机构异常。 */
    ARM_GRIPPER_FAULT              /* 反馈、发送、超时或初始化故障。 */
} Arm_Gripper_State_e;

typedef enum {
    ARM_GRIPPER_COMMAND_HOLD = 0, /* 保持当前目标，不发送新动作。 */
    ARM_GRIPPER_COMMAND_READY,    /* 到550，等待抓取。 */
    ARM_GRIPPER_COMMAND_OPEN,     /* 到550，张开/释放。 */
    ARM_GRIPPER_COMMAND_CLOSE     /* 到630，启用接触/卡死检测。 */
} Arm_Gripper_Command_e;

/* 两台240deg舵机的紧凑Watch变量，角度按控制值0~1000映射到0~240deg。 */
typedef struct {
    uint8_t servo1_communication_ok; /* ID1反馈有效且100ms内有更新。 */
    uint8_t servo2_communication_ok; /* ID2反馈有效且100ms内有更新。 */
    float servo1_current_deg;        /* ID1当前舵机轴角度。 */
    float servo1_target_deg;         /* ID1目标舵机轴角度。 */
    float servo2_current_deg;        /* ID2当前舵机轴角度。 */
    float servo2_target_deg;         /* ID2目标舵机轴角度。 */
} Arm_Servo_Angle_Debug_s;

typedef struct {
    /* 工具初始化、双舵机在线状态和底层反馈。 */
    uint8_t initialized;
    uint8_t self_test_passed;
    uint32_t self_test_fail_mask;
    uint8_t tool_ready;
    Arm_Tool_Init_State_e init_state;
    Arm_Command_Result_e init_result[2];
    uint8_t servo_online[2];
    uint8_t servo_feedback_valid[2];
    uint8_t servo_arrived[2];
    uint8_t servo_motion_timeout[2];
    uint16_t servo_target_pos[2];
    uint16_t servo_feedback_pos[2];
    int16_t servo_position_error[2];
    float servo_feedback_velocity_pos_s[2];
    uint32_t servo_last_feedback_tick[2];
    uint32_t servo_feedback_sequence[2];

    /* ID1绝对俯仰：世界绝对俯仰 = 小臂绝对俯仰 + ID1相对角。 */
    uint8_t tool_pitch_target_valid;
    float tool_pitch_target_deg;
    float tool_pitch_feedback_deg;
    float small_link_pitch_deg;
    uint16_t tool_pitch_servo_pos;
    uint32_t tool_pitch_update_tick;

    /* ID2闭环动作和堵转检测状态。 */
    Arm_Gripper_State_e gripper_state;
    Arm_Gripper_State_e gripper_target_state;
    Arm_Gripper_Command_e gripper_command;
    uint16_t gripper_target_pos;
    uint16_t gripper_feedback_pos;
    int16_t gripper_position_error;
    uint8_t gripper_stall_candidate;
    uint8_t gripper_stall_latched;
    uint8_t gripper_fault_latched;
    uint8_t gripper_boot_stall;
    uint16_t gripper_close_start_pos;
    uint16_t gripper_relief_target_pos;
    uint16_t gripper_stall_window_min_pos;
    uint16_t gripper_stall_window_max_pos;
    uint32_t gripper_action_start_tick;
    uint32_t gripper_stall_window_start_tick;
    uint32_t gripper_settle_start_tick;
    uint32_t gripper_last_feedback_sequence;
    uint32_t gripper_contact_count;
    uint32_t gripper_jam_count;
    uint32_t gripper_timeout_count;

    /* 非阻塞发送统计及30 mm工具中心调试值。 */
    uint32_t error_code;
    uint32_t tx_count[2];
    uint32_t tx_fail_count[2];
    uint8_t tx_pending[2];
    uint16_t tx_pending_pos[2];
    uint16_t tx_pending_time_ms[2];
    uint32_t tx_pending_overwrite_count[2];
    uint32_t tx_single_frame_count;
    uint32_t tx_dual_frame_count;
    uint32_t last_update_tick;
    Arm_Position_s wrist_center_mm;
    Arm_Position_s tool_tip_mm;

} Arm_Tool_State_s;

extern Arm_Tool_State_s g_arm_tool_debug;
extern Arm_Servo_Angle_Debug_s g_arm_servo_angle_debug;

void ArmToolInit(void);
/* 1 ms周期服务：刷新反馈、发送一次性目标并推进夹爪状态机。 */
void ArmToolTask(uint32_t now_ms);
void ArmToolUpdateSmallLinkPitch(float small_link_pitch_deg);
void ArmToolClearPendingCommands(void);
uint8_t ArmToolTxIdle(void);

/* 设置夹爪中心线的绝对俯仰角，越界时返回失败且不发送。 */
Arm_Command_Result_e ArmToolSetPitchDeg(float tool_pitch_deg);
/* 根据ID1新鲜反馈锁存当前绝对俯仰，供后续主臂轨迹保持。 */
Arm_Command_Result_e ArmToolHoldCurrentPitch(float small_link_pitch_deg);
/* 轨迹期间依据当前小臂参考角更新ID1，调用周期为20 ms。 */
Arm_Command_Result_e ArmToolTrackPitch(float tool_pitch_deg,
                                        float small_link_pitch_deg,
                                        uint32_t now_ms);
/* 夹爪业务入口，只接受HOLD/READY/OPEN/CLOSE，不暴露任意原始位置。 */
Arm_Command_Result_e ArmToolSetGripper(Arm_Gripper_Command_e command);
uint8_t ArmToolGripperActionComplete(void);
uint8_t ArmToolGripperFaulted(void);

/* 计算指定绝对俯仰和小臂姿态所需的ID1控制值，不写硬件。 */
uint8_t ArmToolPitchPositionForPose(float tool_pitch_deg,
                                     float small_link_pitch_deg,
                                     uint16_t *position);
uint8_t ArmToolPitchValidForPose(float tool_pitch_deg,
                                  const float q_deg[3]);
float ArmToolSmallLinkPitchFromJoint(const float q_deg[3]);
float ArmToolPitchFromFeedback(float small_link_pitch_deg,
                               uint16_t position);

/* 腕部轴心与夹爪中心之间的30 mm正向/反向坐标换算。 */
uint8_t ArmToolGetCenterFromWrist(const Arm_Position_s *wrist,
                                  float base_yaw_deg,
                                  float tool_pitch_deg,
                                  Arm_Position_s *center);
uint8_t ArmToolGetWristFromCenter(const Arm_Position_s *center,
                                  float base_yaw_deg,
                                  float tool_pitch_deg,
                                  Arm_Position_s *wrist);
const Arm_Tool_State_s *ArmToolGetState(void);

void ArmToolStopServo1Tracking(void);

#endif
