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
    ARM_MODE_READY
} Arm_Mode_e;

typedef enum {
    ARM_CAL_IDLE = 0,
    ARM_CAL_ELBOW_FIND_REFERENCE,
    ARM_CAL_ELBOW_SETTLE_REFERENCE,
    ARM_CAL_ELBOW_RELEASE_REFERENCE,
    ARM_CAL_ELBOW_FIND_OPPOSITE,
    ARM_CAL_ELBOW_DONE,
    ARM_CAL_SHOULDER_FIND_REFERENCE,
    ARM_CAL_SHOULDER_SETTLE_REFERENCE,
    ARM_CAL_SHOULDER_RELEASE_REFERENCE,
    ARM_CAL_SHOULDER_FIND_OPPOSITE,
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
    ARM_IK_NOT_CALIBRATED,
    ARM_IK_INVALID_ARGUMENT,
    ARM_IK_OUT_OF_REACH,
    ARM_IK_NO_LIMITED_SOLUTION
} Arm_IK_Status_e;

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
    Arm_Position_s wrist_center;
    float end_pitch_deg;
    uint16_t wrist_pwm_us;
} Arm_State_s;

extern Arm_State_s g_arm_state;
extern Arm_Calibration_s g_arm_calibration;
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
