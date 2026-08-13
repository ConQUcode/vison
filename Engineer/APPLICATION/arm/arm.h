/**
 * @file arm.h
 * @brief 三自由度达妙机械臂的启动、运动、故障和 Watch 调试接口。
 */

#ifndef __ARM_H__
#define __ARM_H__

#include "stdint.h"
#include "arm_host.h"

/* 本头文件包含机械臂内部状态与调试接口；通信层只包含arm_host.h。 */

typedef enum {
    ARM_JOINT_BASE_YAW = 0,
    ARM_JOINT_SHOULDER,
    ARM_JOINT_ELBOW,
    ARM_JOINT_WRIST,
    ARM_JOINT_COUNT
} Arm_Joint_e;

typedef enum {
    ARM_MODE_SAFE = 0,
    ARM_MODE_READY,
    ARM_MODE_DM_SINGLE_AXIS_TEST,
    ARM_MODE_TEACH_POINT,
    ARM_MODE_FAULT,
    ARM_MODE_ESTOP,
    ARM_MODE_DM_ENABLE_ONLY
} Arm_Mode_e;

typedef enum {
    ARM_START_REGISTERED = 0,
    ARM_START_WAIT_PASSIVE_FEEDBACK,
    ARM_START_ENTER_ALL_MODES,
    ARM_START_WAIT_ENABLE_CONFIRM,
    ARM_START_SYNC_TARGETS,
    ARM_START_VALIDATE_POSITION,
    ARM_START_ESCAPE_ELBOW,
    ARM_START_ESCAPE_SHOULDER,
    ARM_START_ESCAPE_BASE,
    ARM_START_RETURN_ELBOW,
    ARM_START_RETURN_SHOULDER,
    ARM_START_RETURN_BASE,
    ARM_START_READY,
    ARM_START_FAULT,
    ARM_START_ESTOP
} Arm_Start_State_e;

typedef enum {
    ARM_FAULT_NONE = 0,
    ARM_FAULT_CONFIG,
    ARM_FAULT_FEEDBACK_TIMEOUT,
    ARM_FAULT_CAN_TX,
    ARM_FAULT_MOTOR_STATE,
    ARM_FAULT_SOFT_LIMIT,
    ARM_FAULT_HARD_BOUNDARY,
    ARM_FAULT_ESCAPE_DIRECTION,
    ARM_FAULT_ESCAPE_TIMEOUT,
    ARM_FAULT_RETURN_TIMEOUT,
    ARM_FAULT_OVER_TEMPERATURE,
    ARM_FAULT_EMERGENCY_STOP,
    ARM_FAULT_MOTOR_ENABLE_TIMEOUT
} Arm_Fault_e;

typedef enum {
    ARM_FAULT_RESET_NONE = 0,
    ARM_FAULT_RESET_PENDING,
    ARM_FAULT_RESET_OK,
    ARM_FAULT_RESET_NOT_FAULTED,
    ARM_FAULT_RESET_FEEDBACK_INVALID,
    ARM_FAULT_RESET_TEMPERATURE_HIGH,
    ARM_FAULT_RESET_MOTOR_STATE_ACTIVE,
    ARM_FAULT_RESET_OUTSIDE_HARD_BOUNDARY,
    ARM_FAULT_RESET_CAN_UNAVAILABLE,
    ARM_FAULT_RESET_ESTOP_ACTIVE
} Arm_Fault_Reset_Result_e;

typedef enum {
    ARM_LIMIT_INVALID = 0,
    ARM_LIMIT_INSIDE_SOFT,
    ARM_LIMIT_ESCAPE_ALLOWED,
    ARM_LIMIT_OUTSIDE_HARD
} Arm_Limit_Result_e;

typedef enum {
    ARM_IK_OK = 0,
    ARM_IK_BASE_NOT_CALIBRATED,
    ARM_IK_JOINT_NOT_CALIBRATED,
    ARM_IK_INVALID_ARGUMENT,
    ARM_IK_OUT_OF_REACH,
    ARM_IK_NO_LIMITED_SOLUTION,
    ARM_IK_COLLISION_RISK,
    ARM_IK_NUMERICAL_ERROR
} Arm_IK_Status_e;

typedef enum {
    ARM_REALTIME_REJECT_NONE = 0,
    ARM_REALTIME_REJECT_IK,
    ARM_REALTIME_REJECT_FK_ERROR,
    ARM_REALTIME_REJECT_SOFT_LIMIT,
    ARM_REALTIME_REJECT_AUTO_REGION,
    ARM_REALTIME_REJECT_CONTINUITY,
    ARM_REALTIME_REJECT_WORKSPACE_SAFETY
} Arm_Realtime_Reject_Reason_e;

/** 夹爪中心工作区检查结果，Watch可据此直接定位拒绝原因。 */
typedef enum {
    ARM_WORKSPACE_SAFETY_OK = 0,
    ARM_WORKSPACE_SAFETY_TARGET_REAR_TOO_LOW,
    ARM_WORKSPACE_SAFETY_PATH_REAR_TOO_LOW,
    ARM_WORKSPACE_SAFETY_CROSSING_TOO_LOW,
    ARM_WORKSPACE_SAFETY_ESCAPE_ONLY,
    ARM_WORKSPACE_SAFETY_PREFLIGHT_IK,
    ARM_WORKSPACE_SAFETY_SAMPLE_CAPACITY,
    ARM_WORKSPACE_SAFETY_RUNTIME_HEIGHT,
    ARM_WORKSPACE_SAFETY_FRONT_SHOULDER_LIMIT
} Arm_Workspace_Safety_Result_e;

typedef enum {
    ARM_MOTION_IDLE = 0,
    ARM_MOTION_BOOT_DELAY,
    ARM_MOTION_STAGING,
    ARM_MOTION_PREFLIGHT,
    ARM_MOTION_RUNNING,
    ARM_MOTION_SETTLING,
    ARM_MOTION_HOLDING,
    ARM_MOTION_COMPLETE,
    ARM_MOTION_ERROR_IK,
    ARM_MOTION_ERROR_LIMIT,
    ARM_MOTION_ERROR_COLLISION,
    ARM_MOTION_ERROR_OFFLINE,
    ARM_MOTION_ERROR_OVERCURRENT,
    ARM_MOTION_ERROR_TIMEOUT,
    ARM_MOTION_ABORTED
} Arm_Motion_State_e;

typedef enum {
    ARM_MOTION_FAULT_NONE = 0,
    ARM_MOTION_FAULT_SELF_TEST,
    ARM_MOTION_FAULT_IK,
    ARM_MOTION_FAULT_LIMIT,
    ARM_MOTION_FAULT_COLLISION,
    ARM_MOTION_FAULT_OFFLINE,
    ARM_MOTION_FAULT_OVERCURRENT,
    ARM_MOTION_FAULT_TIMEOUT,
    ARM_MOTION_FAULT_ABORT
} Arm_Motion_Fault_e;

typedef enum {
    ARM_MOTION_RESULT_OK = 0,
    ARM_MOTION_RESULT_BUSY,
    ARM_MOTION_RESULT_NOT_READY,
    ARM_MOTION_RESULT_INVALID,
    ARM_MOTION_RESULT_PREFLIGHT_FAILED
} Arm_Motion_Result_e;

typedef enum {
    ARM_DM_AUTO_INIT_IDLE = 0,
    ARM_DM_AUTO_INIT_WAIT_READY,
    ARM_DM_AUTO_INIT_MOVE_AXIS,
    ARM_DM_AUTO_INIT_WAIT_AXIS,
    ARM_DM_AUTO_INIT_MOVE_SAFE,
    ARM_DM_AUTO_INIT_WAIT_SAFE,
    ARM_DM_AUTO_INIT_DONE,
    ARM_DM_AUTO_INIT_FAULT
} Arm_DM_Auto_Init_State_e;

typedef enum {
    ARM_BOOT_WAIT_MOTORS = 0,
    ARM_BOOT_AUTO_INIT,
    ARM_BOOT_WAIT_TOOL,
    ARM_BOOT_STABILIZE,
    ARM_BOOT_GRIPPER_READY_COMMAND,
    ARM_BOOT_GRIPPER_READY_WAIT,
    ARM_BOOT_READY,
    ARM_BOOT_FAULT
} Arm_Boot_State_e;

typedef struct {
    Arm_IK_Status_e status;
    uint8_t candidate_count;
    float q_deg[3];
    Arm_Position_s fk_position;
    float position_error_mm;
} Arm_IK_Result_s;

/** 夹爪中心逆解结果；同时保留最终轴心、工具中心和往返误差。 */
typedef struct {
    Arm_IK_Status_e status;
    uint8_t candidate_count;
    float q_deg[3];
    Arm_Position_s wrist_center_mm;
    Arm_Position_s tool_center_mm;
    float position_error_mm;
} Arm_Tool_Center_IK_Result_s;

/* 单个夹爪中心逆解候选；同一点最多有两种径向方向和两种肘部构型。 */
#define ARM_TOOL_CENTER_IK_MAX_CANDIDATES 4u
typedef struct {
    float q_deg[3];
    Arm_Position_s wrist_center_mm;
    Arm_Position_s tool_center_mm;
    float position_error_mm;
} Arm_Tool_Center_IK_Candidate_s;

typedef struct {
    uint32_t command_id;
    Arm_Control_Point_e control_point;
    Arm_Move_Type_e move_type;
    Arm_Position_s target_mm;
    float max_speed_mm_s;
    uint8_t tool_pitch_valid;
    float tool_pitch_deg;
    uint8_t tool_yaw_valid;
    float tool_yaw_deg;
} Arm_Cartesian_Command_s;

/** 夹爪中心稳定命令；调用方无需再填写容易混淆的control_point。 */
typedef struct {
    uint32_t command_id;
    Arm_Move_Type_e move_type;
    Arm_Position_s target_center_mm;
    float max_speed_mm_s;
    uint8_t tool_pitch_valid;
    float tool_pitch_deg;
} Arm_Tool_Center_Command_s;

typedef struct {
    uint32_t command_id;
    Arm_Move_Type_e move_type;
    float q_deg[3];
    /* 可选连续路径引导点，轨迹经过该点但不会在该点进入到位等待。 */
    uint8_t waypoint_valid;
    float waypoint_q_deg[3];
    /* 可选ID1相对俯仰，单位deg；用于与三台达妙同步完成关节动作。 */
    uint8_t tool_relative_pitch_valid;
    float tool_relative_pitch_deg;
} Arm_Joint_Command_s;

typedef struct {
    uint32_t command_id;
    Arm_Control_Point_e control_point;
    Arm_Position_s target_mm;
    float max_speed_mm_s;
    float max_acceleration_mm_s2;
    uint8_t tool_pitch_valid;
    float tool_pitch_deg;
    uint8_t tool_yaw_valid;
    float tool_yaw_deg;
} Arm_Realtime_Cartesian_Target_s;

typedef struct {
    uint8_t ready;
    Arm_Control_Point_e point_type;
    uint8_t kinematics_valid;
    uint8_t motor_online[3];
    uint8_t motor_enabled[3];
    float q_deg[ARM_JOINT_COUNT];
    float motor_position_rad[3];
    Arm_Position_s wrist_center_mm;
    Arm_Position_s tool_tip_mm;
    float small_link_pitch_deg;
    uint16_t wrist_pwm_us;
    uint8_t wrist_configured;
    uint8_t tool_ready;
    uint16_t tool_servo_target_pos[2];
    uint32_t tool_error_code;
    uint32_t update_count;
} Arm_Teach_Point_s;

typedef struct {
    Arm_Mode_e mode;
    Arm_Start_State_e start_state;
    Arm_Fault_e fault_latched;
    uint8_t config_valid;
    uint8_t kinematics_valid;
    uint8_t all_targets_synced;
    uint8_t active_axis;
    uint8_t motor_online[3];
    uint8_t motor_enabled[3];
    uint8_t target_synced[3];
    uint8_t soft_limit_ok[3];
    uint8_t hard_boundary_ok[3];
    uint8_t motor_state[3];
    float q_feedback_deg[ARM_JOINT_COUNT];
    float q_target_deg[ARM_JOINT_COUNT];
    float motor_position_rad[3];
    float motor_velocity_rad_s[3];
    float motor_speed_dps[3];
    float motor_current[3]; /* 达妙版兼容轨迹调试字段，单位为N*m。 */
    float motor_torque_nm[3];
    float motor_command_rad[3];
    float motor_velocity_limit_rad_s[3];
    float mos_temperature_c[3];
    float rotor_temperature_c[3];
    Arm_Position_s wrist_center;
    Arm_Position_s tool_tip;
    float small_link_pitch_deg;
    float end_pitch_deg;
    uint8_t tool_ready;
    uint16_t tool_servo_target_pos[2];
    uint32_t tool_error_code;
    uint16_t wrist_pwm_us;
    uint32_t state_elapsed_ms;
    uint32_t fault_reset_request;
    uint32_t fault_reset_applied;
    Arm_Fault_Reset_Result_e fault_reset_result;
} Arm_State_s;

typedef struct {
    uint16_t motor_id;
    uint16_t master_id;
    uint16_t command_id;
    float raw_position_rad;
    float logical_joint_deg;
    float logical_target_deg;
    float command_position_rad;
    float velocity_limit_rad_s;
    float velocity_limit_deg_s;
    float velocity_rad_s;
    float velocity_deg_s;
    float torque_nm;
    float mos_temperature_c;
    float rotor_temperature_c;
    uint8_t state;
    uint8_t feedback_valid;
    uint8_t online;
    uint8_t target_synced;
    uint8_t enabled;
    uint8_t mode_request_pending;
    uint8_t mode_confirmed;
    Arm_Limit_Result_e limit_result;
    uint8_t hard_boundary_ok;
    uint32_t rx_count;
    uint32_t feedback_age_ms; /* 距最新CAN反馈的时间；ISR反馈晚于快照时记0。 */
    uint32_t tx_count;
    uint32_t tx_fail_count;
    uint32_t mode_command_count;
    uint16_t last_mode_tx_id;
} Arm_DM_Axis_Debug_s;

typedef struct {
    uint8_t enable;
    Arm_DM_Auto_Init_State_e state;
    uint8_t axis;
    uint8_t step;
    uint8_t done;
    Arm_Command_Result_e result;
    float speed_deg_s;
    float start_deg;
    float target_deg;
    /* 兼容字段名；当前记录ID1俯仰舵机轴心HOME目标。 */
    Arm_Position_s target_tool_tip_mm;
    Arm_Position_s target_wrist_mm;
    Arm_IK_Status_e ik_status;
    float target_q_deg[3];
    uint32_t elapsed_ms;
    uint32_t cycle_count;
} Arm_DM_Auto_Init_Debug_s;

typedef struct {
    Arm_Boot_State_e state;
    Arm_Motion_Result_e motion_result;
    Arm_IK_Status_e ik_status;
    uint32_t state_tick;
    uint32_t elapsed_ms;
} Arm_Boot_Debug_s;

typedef struct {
    Arm_Mode_e mode;
    Arm_Start_State_e start_state;
    Arm_Fault_e fault;
    uint8_t elbow_coupling_active;
    uint8_t passive_feedback_observed[3];
    uint8_t enter_mode_sent[3];
    uint8_t power_on_delay_active;
    uint8_t power_on_delay_done;
    uint32_t power_on_delay_elapsed_ms;
    Arm_DM_Auto_Init_Debug_s auto_init;
    uint32_t fault_reset_request;
    uint32_t fault_reset_applied;
    Arm_Fault_Reset_Result_e fault_reset_result;
    Arm_DM_Axis_Debug_s axis[3];
} Arm_DM_Debug_s;

/*
 * 大臂/小臂归正观察量：仅保留台架测试最需要的四个角度。
 * 大臂使用物理安装角；小臂使用其与大臂之间的物理夹角。
 */
typedef struct {
    float shoulder_current_deg;       /* 大臂当前物理角，HOME约-90deg。 */
    float shoulder_target_deg;        /* 大臂目标物理角，HOME为-90deg。 */
    float elbow_included_current_deg; /* 当前两杆夹角，HOME约60deg。 */
    float elbow_included_target_deg;  /* 目标两杆夹角，HOME为60deg。 */
} Arm_Home_Joint_Debug_s;

typedef struct {
    uint8_t kinematics_valid;
    uint8_t motor_online[3];
    uint8_t motor_enabled[3];
    float q_feedback_deg[3];
    float q_target_deg[3];
    Arm_Position_s wrist_center_mm; /* 主臂FK输出：ID1舵机输出轴中心。 */
    Arm_Position_s tool_center_mm;  /* 工具FK输出：当前夹爪中心。 */
    float tool_axis_to_center_mm;   /* ID1轴心到夹爪中心的配置长度。 */
    float tool_pitch_feedback_deg;  /* 夹爪中心线世界绝对俯仰角。 */
    float horizontal_radius_mm;
    float planar_reach_from_shoulder_mm;
    float wrist_height_from_shoulder_mm;
    float small_link_pitch_deg;
    float base_height_mm;
    float link_1_mm;
    float link_2_mm;
    float shoulder_offset_forward_mm;
    float shoulder_offset_left_mm;
    float soft_limit_deg[3][2];
    float reference_q_deg[3];
    Arm_Position_s reference_wrist_center_mm;
} Arm_Kinematics_Debug_s;

typedef struct {
    Arm_Motion_State_e motion_state;
    Arm_Motion_Fault_e fault_code;
    uint8_t sequence_index;
    float current_q_deg[3];
    float trajectory_q_deg[3];
    float target_q_deg[3];
    float joint_error_deg[3];
    uint8_t motor_online[3];
    uint8_t motor_enabled[3];
    Arm_Position_s current_position_mm;
    Arm_Position_s target_position_mm;
    Arm_IK_Status_e ik_status;
    uint8_t path_preflight_passed;
    uint8_t command_accepted;
    Arm_Realtime_Reject_Reason_e realtime_reject_reason;
    Arm_Realtime_Reject_Reason_e last_realtime_reject_reason;
    uint32_t command_reject_count;
    float trajectory_progress;
    uint32_t trajectory_duration_ms;
    uint32_t trajectory_elapsed_ms;
    uint16_t path_sample_count;
    uint32_t preflight_duration_ms; /* 最近一次完整路径预检耗时。 */
    /* 兼容旧Watch；独立MotorControlTask启用后该值应保持0。 */
    uint32_t preflight_motor_service_count;
    /* 同步预检期间推进末端舵机非阻塞通信的次数。 */
    uint32_t preflight_tool_service_count;
    float position_error_mm;
    uint8_t safety_route_enabled;
    uint8_t safety_route_segment_count;
    uint8_t safety_route_active_segment;
    Arm_Workspace_Safety_Result_e workspace_safety_result;
    /* 路径预检失败定位：段号、段内采样号、工具中心和候选关节角。 */
    uint8_t preflight_failed_segment;
    uint16_t preflight_failed_sample;
    uint32_t preflight_failed_check_mask;
    Arm_Position_s preflight_failed_center_mm;
    float preflight_failed_q_deg[3];
} Arm_Motion_Debug_s;

typedef struct {
    float current_q_deg[3];
    float reference_q_deg[3];
    float target_q_deg[3];
    float tracking_error_deg[3];
    float peak_tracking_error_deg[3];
    float overshoot_deg[3];
    float motor_speed_deg_s[3];
    float motor_current[3]; /* 达妙扭矩N*m。 */
    uint8_t tracking_error_warning[3];
    uint32_t tracking_error_duration_ms[3];
    float trajectory_speed_mm_s;
    float trajectory_acceleration_mm_s2;
    uint32_t trajectory_elapsed_ms;
    uint32_t trajectory_duration_ms;
    uint32_t settling_time_ms;
    uint32_t arrival_stable_ms;
    uint32_t settling_timeout_ms;
    uint32_t command_age_ms;
    uint8_t arrival_within_tolerance;
    uint8_t realtime_active;
    uint8_t realtime_timed_out;
    uint32_t realtime_command_id;
} Arm_Control_Debug_s;

extern Arm_State_s g_arm_state;
extern Arm_DM_Debug_s g_arm_dm_debug;
extern Arm_Home_Joint_Debug_s g_arm_home_joint_debug;
extern Arm_Kinematics_Debug_s g_arm_kinematics_debug;
extern Arm_Motion_Debug_s g_arm_motion_debug;
extern Arm_Control_Debug_s g_arm_control_debug;
extern Arm_Teach_Point_s g_arm_teach_point;
extern Arm_Boot_Debug_s g_arm_boot_debug;

/** 注册三台 CAN1 达妙电机和末端舵机，建立启动状态机。 */
void ArmInit(void);
/** 机械臂 1 kHz 非阻塞任务；仅由 ArmControlTask 调用。 */
void ArmTask(void);
/** 取消运动并停止当前控制输出。 */
void ArmStop(void);
/** 返回内部状态只读指针，调用方不得修改。 */
const Arm_State_s *ArmGetState(void);
const Arm_Motion_Debug_s *ArmGetMotionState(void);
const Arm_Teach_Point_s *ArmGetTeachPoint(void);
/**
 * 打点模式达妙反馈轮询；周期重发失能命令触发反馈回复，无力矩影响。
 * 由高优先级电机任务调用；非打点模式编译为空操作。
 */
void ArmTeachPointFeedbackPoll(uint32_t now_ms);
Arm_Command_Result_e ArmSubmitCartesianCommand(
    const Arm_Cartesian_Command_s *command);
/** 提交以夹爪中心为目标的运动，内部固定选择TOOL_CENTER。 */
Arm_Command_Result_e ArmSubmitToolCenterCommand(
    const Arm_Tool_Center_Command_s *command);
Arm_Command_Result_e ArmSubmitJointCommand(
    const Arm_Joint_Command_s *command);
Arm_Command_Result_e ArmSubmitRealtimeCartesianTarget(
    const Arm_Realtime_Cartesian_Target_s *target);
void ArmStopRealtimeTracking(void);
void ArmCancelMotion(void);
void ArmEmergencyStop(void);
Arm_Fault_Reset_Result_e ArmRequestFaultReset(void);

float ArmMotorRadToJointDeg(Arm_Joint_e axis, float motor_rad);
float ArmJointDegToMotorRad(Arm_Joint_e axis, float joint_deg);
float ArmMotorVelocityToJointDegS(Arm_Joint_e axis, float motor_rad_s);

void ArmForwardKinematics3DOF(float q1_deg,
                              float q2_deg,
                              float q3_deg,
                              Arm_Position_s *position);
Arm_IK_Status_e ArmInverseKinematics3DOF(const Arm_Position_s *target,
                                         const float current_q_deg[3],
                                         Arm_IK_Result_s *result);
/** 由三关节和绝对俯仰计算夹爪中心，成功返回1。 */
uint8_t ArmForwardKinematicsToolCenter(
    const float q_deg[3], float tool_pitch_deg,
    Arm_Position_s *tool_center_mm);
/** 对夹爪中心目标检查正/负径向候选并返回与seed最近的合法解。 */
Arm_IK_Status_e ArmInverseKinematicsToolCenter(
    const Arm_Position_s *target_center_mm,
    float tool_pitch_deg,
    const float seed_q_deg[3],
    Arm_Tool_Center_IK_Result_s *result);
Arm_IK_Status_e ArmInverseKinematicsToolCenterAll(
    const Arm_Position_s *target_center_mm,
    float tool_pitch_deg,
    const float seed_q_deg[3],
    Arm_Tool_Center_IK_Candidate_s candidates[
        ARM_TOOL_CENTER_IK_MAX_CANDIDATES],
    uint8_t *candidate_count);

#endif
