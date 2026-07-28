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
    ARM_MODE_CALIBRATION,
    ARM_MODE_READY,
    ARM_MODE_SOFT_LIMIT
} Arm_Mode_e;

typedef enum {
    ARM_SOFT_LIMIT_AXIS_NONE = 0,
    ARM_SOFT_LIMIT_AXIS_BASE,
    ARM_SOFT_LIMIT_AXIS_SHOULDER,
    ARM_SOFT_LIMIT_AXIS_ELBOW
} Arm_Soft_Limit_Axis_e;

typedef enum {
    ARM_SOFT_LIMIT_DISABLED = 0,
    ARM_SOFT_LIMIT_WAIT_ONLINE,
    ARM_SOFT_LIMIT_BASE_MOVING,
    ARM_SOFT_LIMIT_BASE_SETTLE,
    ARM_SOFT_LIMIT_WAIT_CALIBRATION,
    ARM_SOFT_LIMIT_COMPLETE,
    ARM_SOFT_LIMIT_ERROR_OFFLINE,
    ARM_SOFT_LIMIT_ERROR_LIMIT,
    ARM_SOFT_LIMIT_ERROR_OVERCURRENT,
    ARM_SOFT_LIMIT_ERROR_TIMEOUT,
    ARM_SOFT_LIMIT_ABORTED
} Arm_Soft_Limit_State_e;

typedef enum {
    ARM_CAL_IDLE = 0,
    ARM_CAL_ELBOW_FIND_REFERENCE,
    ARM_CAL_ELBOW_SETTLE_REFERENCE,
    ARM_CAL_ELBOW_RELEASE_REFERENCE,
    ARM_CAL_ELBOW_FIND_OPPOSITE,
    ARM_CAL_ELBOW_SETTLE_OPPOSITE,
    ARM_CAL_ELBOW_DONE,
    ARM_CAL_SHOULDER_FIND_REFERENCE,
    ARM_CAL_SHOULDER_SETTLE_REFERENCE,
    ARM_CAL_SHOULDER_RELEASE_REFERENCE,
    ARM_CAL_SHOULDER_FIND_OPPOSITE,
    ARM_CAL_SHOULDER_SETTLE_OPPOSITE,
    ARM_CAL_SHOULDER_DONE,
    ARM_CAL_VALID,
    ARM_CAL_ERROR_ABORT,
    ARM_CAL_ERROR_OFFLINE,
    ARM_CAL_ERROR_TIMEOUT,
    ARM_CAL_ERROR_TRAVEL
} Arm_Calibration_State_e;

typedef enum {
    ARM_HOMING_NONE = 0,
    ARM_HOMING_SINGLE_REFERENCE,
    ARM_HOMING_FULL_SCAN
} Arm_Homing_Mode_e;

typedef enum {
    ARM_RATIO_TEST_DISABLED = 0,
    ARM_RATIO_TEST_WAIT_ONLINE,
    ARM_RATIO_TEST_SETTLE_ZERO,
    ARM_RATIO_TEST_RUNNING,
    ARM_RATIO_TEST_COMPLETE,
    ARM_RATIO_TEST_ERROR_OFFLINE,
    ARM_RATIO_TEST_ERROR_STALL,
    ARM_RATIO_TEST_ERROR_TIMEOUT,
    ARM_RATIO_TEST_ABORTED
} Arm_Ratio_Test_State_e;

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
    float shoulder_hard_min_deg;
    float shoulder_hard_max_deg;
    float shoulder_soft_min_deg;
    float shoulder_soft_max_deg;
    float elbow_hard_min_deg;
    float elbow_hard_max_deg;
    float elbow_soft_min_deg;
    float elbow_soft_max_deg;
    float shoulder_motor_span_deg;
    float elbow_motor_span_deg;
    float shoulder_deg_per_motor_deg;
    float elbow_deg_per_motor_deg;
    float shoulder_direction;
    float elbow_direction;
    float shoulder_teach_motor_deg;
    float elbow_teach_motor_deg;
    uint8_t shoulder_reference_found;
    uint8_t shoulder_opposite_found;
    uint8_t elbow_reference_found;
    uint8_t elbow_opposite_found;
    uint8_t base_calibrated;
    uint8_t joint_calibrated;
    uint8_t calibration_valid;
} Arm_Calibration_s;

typedef struct {
    Arm_Mode_e mode;
    Arm_Homing_Mode_e homing_mode;
    Arm_Ratio_Test_State_e ratio_test_state;
    Arm_Soft_Limit_Axis_e soft_limit_axis;
    Arm_Soft_Limit_State_e soft_limit_state;
    Arm_Calibration_State_e calibration_state;
    float q_feedback_deg[ARM_JOINT_COUNT];
    float motor_total_angle_deg[3];
    float motor_current[3];
    float motor_speed_dps[3];
    uint8_t motor_online[3];
    uint32_t calibration_stage_elapsed_ms;
    uint32_t calibration_stall_elapsed_ms;
    float calibration_stage_travel_motor_deg;
    uint8_t calibration_stall_condition;
    int16_t calibration_peak_abs_current[2];
    int16_t calibration_trigger_current[2];
    float calibration_trigger_speed_dps[2];
    float ratio_test_target_motor_deg;
    float ratio_test_motor_angle_deg;
    float ratio_test_expected_output_deg;
    uint32_t ratio_test_elapsed_ms;
    float base_raw_deg;
    float shoulder_deg_per_motor_deg;
    float elbow_deg_per_motor_deg;
    uint8_t base_calibrated;
    uint8_t joint_calibrated;
    uint8_t kinematics_valid;
    uint8_t motor_enabled[3];
    float q_target_deg[ARM_JOINT_COUNT];
    float soft_limit_target_joint_deg[2];
    float soft_limit_target_motor_deg[2];
    float base_init_target_raw_deg;
    float base_init_target_motor_deg;
    uint32_t soft_limit_elapsed_ms;
    Arm_Position_s wrist_center;
    float small_link_pitch_deg;
    float end_pitch_deg;
    uint16_t wrist_pwm_us;
} Arm_State_s;

/*
 * Watch-only snapshot for boot initialization. target_joint_deg records the
 * 5-degree software-limit boundary for later commands; normal boot does not
 * move M3508 or M2006 to that target.
 */
typedef struct {
    Arm_Mode_e arm_mode;
    Arm_Calibration_State_e calibration_state;
    Arm_Soft_Limit_Axis_e active_axis;
    Arm_Soft_Limit_State_e state;
    uint8_t homing_abort;
    uint8_t base_calibrated;
    uint8_t joint_calibrated;
    uint8_t kinematics_valid;
    uint8_t motor_online[3];
    uint8_t motor_enabled[3];
    float q_feedback_deg[3];
    float motor_total_angle_deg[3];
    float motor_current[3];
    float motor_speed_dps[3];
    uint8_t small_angle_test_active;
    float command_target_joint_deg[3];
    float command_target_motor_deg[3];
    float target_joint_deg[2];
    float target_motor_deg[2];
    float base_target_raw_deg;
    float base_target_motor_deg;
    float base_angle_error_deg;
    float base_angle_pid_output_dps;
    float base_speed_pid_output;
    float final_joint_deg[2];
    float final_motor_deg[2];
    float active_angle_ref_deg;
    float active_angle_pid_output_dps;
    float active_speed_pid_output;
    float angle_ref_deg[2];
    float angle_pid_output_dps[2];
    float speed_pid_output[2];
    float current_pid_output[3];
    uint8_t motion_test_pose_index;
    uint8_t motion_test_target_reached;
    uint32_t motion_test_hold_elapsed_ms;
    uint32_t state_elapsed_ms;
} Arm_Soft_Limit_Debug_s;

/* Watch-only snapshot for FK validation. All distances are in millimeters
 * and all joint angles are in degrees. */
typedef struct {
    uint8_t kinematics_valid;
    uint8_t base_calibrated;
    uint8_t joint_calibrated;
    uint8_t motor_online[3];
    uint8_t motor_enabled[3];
    float base_raw_deg;
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
    float shoulder_soft_limit_deg[2];
    float elbow_soft_limit_deg[2];
    float reference_q_deg[3];
    Arm_Position_s reference_wrist_center_mm;
    float reference_horizontal_radius_mm;
    float reference_planar_reach_from_shoulder_mm;
    float reference_height_from_shoulder_mm;
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
    uint32_t command_reject_count;
    float trajectory_progress;
    uint32_t trajectory_duration_ms;
    uint32_t trajectory_elapsed_ms;
    uint16_t path_sample_count;
    float position_error_mm;
} Arm_Motion_Debug_s;

extern Arm_State_s g_arm_state;
extern Arm_Calibration_s g_arm_calibration;
extern Arm_Soft_Limit_Debug_s g_arm_soft_limit_debug;
extern Arm_Kinematics_Debug_s g_arm_kinematics_debug;
extern Arm_Motion_Debug_s g_arm_motion_debug;
extern volatile uint8_t g_arm_homing_abort;

void ArmInit(void);
void ArmTask(void);
void ArmStop(void);
const Arm_State_s *ArmGetState(void);

/* Normal boot homing: find only the two measured reference stops. */
void ArmHomingStart(void);
/* Maintenance calibration: scan both stops and refresh RAM scale values. */
void ArmCalibrationStart(void);
void ArmCalibrationAbort(void);
uint8_t ArmBaseTeachFront(void);
uint8_t ArmSetJointTargetDeg(float q1_deg, float q2_deg, float q3_deg);
uint8_t ArmBeginJointMove(const float target_q_deg[3]);
uint8_t ArmUpdateJointReference(const float reference_q_deg[3]);
void ArmMotionStopMotors(void);
void ArmAbortMotion(Arm_Motion_Fault_e reason);
Arm_Motion_Result_e ArmSetCartesianTarget(const Arm_Position_s *target,
                                          Arm_IK_Result_s *result);
Arm_Motion_Result_e ArmMoveLinear(const Arm_Position_s *target,
                                  float max_speed_mm_s);

void ArmForwardKinematics3DOF(float q1_deg,
                              float q2_deg,
                              float q3_deg,
                              Arm_Position_s *position);
Arm_IK_Status_e ArmInverseKinematics3DOF(const Arm_Position_s *target,
                                         const float current_q_deg[3],
                                         Arm_IK_Result_s *result);

void ArmWristPWMInit(void);
void ArmWristPWMSetUs(uint16_t pulse_us);
void ArmWristPWMSetAngle(float angle_deg);

#endif
