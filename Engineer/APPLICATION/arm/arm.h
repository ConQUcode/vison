#ifndef __ARM_H__
#define __ARM_H__

#include "stdint.h"

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
    ARM_REALTIME_REJECT_CONTINUITY
} Arm_Realtime_Reject_Reason_e;

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
    ARM_COMMAND_PREFLIGHT_FAILED
} Arm_Command_Result_e;

typedef enum {
    ARM_DM_AUTO_INIT_IDLE = 0,
    ARM_DM_AUTO_INIT_WAIT_READY,
    ARM_DM_AUTO_INIT_MOVE_AXIS,
    ARM_DM_AUTO_INIT_WAIT_AXIS,
    ARM_DM_AUTO_INIT_DONE,
    ARM_DM_AUTO_INIT_FAULT
} Arm_DM_Auto_Init_State_e;

typedef enum {
    ARM_DM_AUTO_POINT_IDLE = 0,
    ARM_DM_AUTO_POINT_WAIT_INIT,
    ARM_DM_AUTO_POINT_SOLVE_IK,
    ARM_DM_AUTO_POINT_MOVE_AXIS,
    ARM_DM_AUTO_POINT_WAIT_AXIS,
    ARM_DM_AUTO_POINT_DONE,
    ARM_DM_AUTO_POINT_FAULT
} Arm_DM_Auto_Point_State_e;

typedef struct {
    float x_mm;
    float y_mm;
    float z_mm;
} Arm_Position_s;

typedef struct {
    Arm_IK_Status_e status;
    uint8_t candidate_count;
    float q_deg[3];
    Arm_Position_s fk_position;
    float position_error_mm;
} Arm_IK_Result_s;

typedef struct {
    uint32_t command_id;
    Arm_Control_Point_e control_point;
    Arm_Move_Type_e move_type;
    Arm_Position_s target_mm;
    float max_speed_mm_s;
    uint8_t tool_pitch_valid;
    float tool_pitch_deg;
} Arm_Cartesian_Command_s;

typedef struct {
    uint32_t command_id;
    Arm_Move_Type_e move_type;
    float q_deg[3];
} Arm_Joint_Command_s;

typedef struct {
    uint32_t command_id;
    Arm_Position_s target_mm;
    float max_speed_mm_s;
    float max_acceleration_mm_s2;
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
    float small_link_pitch_deg;
    uint16_t wrist_pwm_us;
    uint8_t wrist_configured;
    uint8_t tool_model_valid;
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
    float small_link_pitch_deg;
    float end_pitch_deg;
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
    float target_q_deg[3];
    uint32_t elapsed_ms;
    uint32_t cycle_count;
} Arm_DM_Auto_Init_Debug_s;

typedef struct {
    uint8_t enable;
    Arm_DM_Auto_Point_State_e state;
    uint8_t axis;
    uint8_t step;
    uint8_t done;
    Arm_Command_Result_e result;
    Arm_IK_Status_e ik_status;
    Arm_Position_s target_mm;
    float target_q_deg[3];
    float speed_deg_s;
    float start_deg;
    float target_deg;
    uint32_t elapsed_ms;
    uint32_t cycle_count;
} Arm_DM_Auto_Point_Debug_s;

typedef struct {
    Arm_Mode_e mode;
    Arm_Start_State_e start_state;
    Arm_Fault_e fault;
    uint8_t elbow_coupling_active;
    uint8_t passive_feedback_observed[3];
    uint8_t enter_mode_sent[3];
    Arm_DM_Auto_Init_Debug_s auto_init;
    Arm_DM_Auto_Point_Debug_s auto_point;
    uint32_t fault_reset_request;
    uint32_t fault_reset_applied;
    Arm_Fault_Reset_Result_e fault_reset_result;
    Arm_DM_Axis_Debug_s axis[3];
} Arm_DM_Debug_s;

typedef struct {
    uint8_t kinematics_valid;
    uint8_t motor_online[3];
    uint8_t motor_enabled[3];
    float q_feedback_deg[3];
    float q_target_deg[3];
    Arm_Position_s wrist_center_mm;
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
    float position_error_mm;
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
    uint32_t command_age_ms;
    uint8_t realtime_active;
    uint8_t realtime_timed_out;
    uint32_t realtime_command_id;
} Arm_Control_Debug_s;

extern Arm_State_s g_arm_state;
extern Arm_DM_Debug_s g_arm_dm_debug;
extern Arm_Kinematics_Debug_s g_arm_kinematics_debug;
extern Arm_Motion_Debug_s g_arm_motion_debug;
extern Arm_Control_Debug_s g_arm_control_debug;
extern Arm_Teach_Point_s g_arm_teach_point;

void ArmInit(void);
void ArmTask(void);
void ArmStop(void);
const Arm_State_s *ArmGetState(void);
const Arm_Motion_Debug_s *ArmGetMotionState(void);
const Arm_Teach_Point_s *ArmGetTeachPoint(void);
Arm_Command_Result_e ArmSubmitCartesianCommand(
    const Arm_Cartesian_Command_s *command);
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

#endif
