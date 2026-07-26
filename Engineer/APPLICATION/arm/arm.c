#include "arm.h"

#include "DJI_motor.h"
#include "can.h"
#include "daemon.h"
#include "math.h"
#include "string.h"
#include "tim.h"

#define ARM_PI                         3.14159265358979323846f
#define ARM_DEG_TO_RAD                 (ARM_PI / 180.0f)
#define ARM_RAD_TO_DEG                 (180.0f / ARM_PI)
#define ARM_BASE_HEIGHT_MM             80.0f
#define ARM_LINK_1_MM                  150.0f
#define ARM_LINK_2_MM                  179.0f
#define ARM_WRIST_PWM_MIN_US           1000u
#define ARM_WRIST_PWM_MID_US           1500u
#define ARM_WRIST_PWM_MAX_US           2000u
#define ARM_SHOULDER_SPEED_OUTPUT_MAX  3500.0f
#define ARM_ELBOW_SPEED_OUTPUT_MAX     2200.0f
#define ARM_FLOAT_EPSILON              0.0001f
#define ARM_RANGE_EPSILON_DEG          0.5f
#define ARM_AUTO_START_ONLINE_MS        500u
#define ARM_REFERENCE_SETTLE_MS        2000u
#define ARM_SHOULDER_MIN_DIRECTION      1.0f
#define ARM_ELBOW_MIN_DIRECTION         1.0f

/*
 * Joint angles measured at the two mechanical stops. The reference stop is
 * the first stop reached during homing. A maintenance full scan re-measures
 * the motor span and updates the motor-to-joint scale from these endpoints.
 */
#define ARM_SHOULDER_REFERENCE_DEG      (-9.0f)
#define ARM_SHOULDER_OPPOSITE_DEG       188.0f
#define ARM_ELBOW_REFERENCE_DEG         132.0f
#define ARM_ELBOW_OPPOSITE_DEG          (-101.0f)
#define ARM_SHOULDER_MEASURED_SPAN_DEG  (-5493.33936f)
#define ARM_ELBOW_MEASURED_SPAN_DEG     (-8517.4375f)

/*
 * 0: normal boot, find only the two reference stops.
 * 1: maintenance boot, scan both stops and refresh mapping values in RAM.
 * Return this switch to 0 after maintenance; full-scan results are visible in
 * g_arm_calibration and can be copied back to the measured span constants.
 */
#define ARM_BOOT_FULL_SCAN              0u

/*
 * Temporary M3508 gearbox test. While enabled, normal arm homing is bypassed,
 * GM6020/M2006 remain disabled, and M3508 rotates 19 * 90 = 1710 motor degrees.
 * Set this back to 0 after checking the output-shaft angle.
 */
#define ARM_3508_RATIO_TEST_RATIO        19.0f
#define ARM_3508_RATIO_TEST_OUTPUT_DEG   90.0f
#define ARM_3508_RATIO_TEST_DIRECTION    1.0f
#define ARM_3508_RATIO_TEST_SPEED_DPS    342.0f
#define ARM_3508_RATIO_TEST_SETTLE_MS    2000u
#define ARM_3508_RATIO_TEST_TIMEOUT_MS   30000u
#define ARM_3508_RATIO_TEST_STALL_MS     200u
#define ARM_3508_RATIO_TEST_OVERRUN_DEG  90.0f

typedef enum {
    ARM_CAL_MOTOR_SHOULDER = 0,
    ARM_CAL_MOTOR_ELBOW,
    ARM_CAL_MOTOR_COUNT
} Arm_Calibration_Motor_e;

typedef struct {
    uint32_t stage_start_tick;
    uint32_t stall_start_tick;
    float stage_start_motor_angle_deg;
} Arm_Calibration_Runtime_s;

static DJIMotor_Instance *arm_base_motor;
static DJIMotor_Instance *arm_shoulder_motor;
static DJIMotor_Instance *arm_elbow_motor;
static DJIMotor_Instance *arm_motors[3];
static Arm_Calibration_Runtime_s arm_cal_runtime;
static uint8_t arm_initialized;
static uint8_t arm_auto_calibration_attempted;
static uint8_t arm_auto_online_waiting;
static uint32_t arm_auto_online_start_tick;
static float arm_base_front_raw_deg;
static float arm_base_direction = 1.0f;
static uint32_t arm_ratio_test_state_tick;
static uint32_t arm_ratio_test_stall_tick;
/* Temporary internal switch: set to 0 after the gearbox test. */
static volatile uint8_t arm_3508_ratio_test_enable = 1u;

static void ArmCalibrationFail(Arm_Calibration_State_e error_state);

Arm_State_s g_arm_state;
Arm_Calibration_s g_arm_calibration;

/* Bench-tuned calibration constants. Change these in source when required. */
static const float g_arm_shoulder_homing_speed_dps = 450.0f;
static const float g_arm_shoulder_stall_current = 1300.0f;
static const float g_arm_elbow_homing_speed_dps = 800.0f;
static const float g_arm_elbow_stall_current = 1400.0f;
static const float g_arm_homing_stall_speed_dps = 20.0f;
static const uint32_t g_arm_homing_spinup_ms = 500u;
static const uint32_t g_arm_homing_stall_confirm_ms = 8u;
static const float g_arm_shoulder_motor_to_joint_ratio = 27.88497f;
static const float g_arm_elbow_motor_to_joint_ratio = 36.55553f;
static const float g_arm_soft_limit_margin_deg = 3.0f;
static const float g_arm_release_joint_deg = 1.0f;
static const uint32_t g_arm_release_min_ms = 500u;
static const uint32_t g_arm_shoulder_stage_timeout_ms = 45000u;
static const uint32_t g_arm_elbow_stage_timeout_ms = 80000u;
static const float g_arm_shoulder_max_joint_travel_deg = 360.0f;
static const float g_arm_elbow_max_joint_travel_deg = 360.0f;

volatile uint8_t g_arm_homing_abort;

static float ArmClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float ArmWrapTo180(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg <= -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float ArmAngleDifference(float angle_deg, float reference_deg)
{
    return ArmWrapTo180(angle_deg - reference_deg);
}

static float ArmSign(float value)
{
    return value >= 0.0f ? 1.0f : -1.0f;
}

static Motor_Init_Config_s ArmBaseMotorConfig(void)
{
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 12.0f,
                .Ki = 0.2f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 1000.0f,
                .MaxOut = 12000.0f,
            },
            .speed_PID = {
                .Kp = 30.0f,
                .Ki = 1.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 3000.0f,
                .MaxOut = 12000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = (Closeloop_Type_e)(ANGLE_LOOP | SPEED_LOOP),
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .feedforward_flag = FEEDFORWARD_NONE,
        },
        .motor_type = GM6020,
    };
    return config;
}

static Motor_Init_Config_s ArmShoulderMotorConfig(void)
{
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 12.0f,
                .Ki = 0.2f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 1000.0f,
                .MaxOut = ARM_SHOULDER_SPEED_OUTPUT_MAX,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .feedforward_flag = FEEDFORWARD_NONE,
        },
        .motor_type = M3508,
    };
    return config;
}

static Motor_Init_Config_s ArmElbowMotorConfig(void)
{
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 3,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10.0f,
                .Ki = 0.1f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 700.0f,
                .MaxOut = ARM_ELBOW_SPEED_OUTPUT_MAX,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .feedforward_flag = FEEDFORWARD_NONE,
        },
        .motor_type = M2006,
    };
    return config;
}

static void ArmClearPidRuntime(PID_Instance *pid)
{
    pid->Measure = 0.0f;
    pid->Last_Measure = 0.0f;
    pid->Err = 0.0f;
    pid->Last_Err = 0.0f;
    pid->Last_ITerm = 0.0f;
    pid->Pout = 0.0f;
    pid->Iout = 0.0f;
    pid->Dout = 0.0f;
    pid->ITerm = 0.0f;
    pid->Output = 0.0f;
    pid->Last_Output = 0.0f;
    pid->Last_Dout = 0.0f;
    pid->Ref = 0.0f;
    pid->ERRORHandler.ERRORCount = 0u;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;
    DWT_GetDeltaT(&pid->DWT_CNT);
}

static void ArmClearMotorController(DJIMotor_Instance *motor)
{
    if (motor == NULL) {
        return;
    }
    motor->motor_controller.pid_ref = 0.0f;
    ArmClearPidRuntime(&motor->motor_controller.angle_PID);
    ArmClearPidRuntime(&motor->motor_controller.speed_PID);
    ArmClearPidRuntime(&motor->motor_controller.current_PID);
}

static void ArmStopMotor(DJIMotor_Instance *motor)
{
    if (motor == NULL) {
        return;
    }
    DJIMotorSetRef(motor, 0.0f);
    DJIMotorStop(motor);
}

static uint8_t ArmMotorFeedbackReady(const DJIMotor_Instance *motor)
{
    return motor != NULL && motor->daemon != NULL && motor->feed_cnt != 0u &&
           DaemonIsOnline(motor->daemon);
}

static void ArmUpdateCalibrationValid(void)
{
    g_arm_calibration.calibration_valid =
        g_arm_calibration.base_calibrated && g_arm_calibration.joint_calibrated;
}

static float ArmBaseJointAngle(void)
{
    float direction;

    if (!g_arm_calibration.base_calibrated || arm_base_motor == NULL) {
        return 0.0f;
    }
    direction = ArmSign(arm_base_direction);
    return ArmWrapTo180(direction *
                        (arm_base_motor->measure.angle_single_round -
                         arm_base_front_raw_deg));
}

static float ArmMotorToCalibratedJoint(float motor_angle_deg,
                                       float reference_joint_deg,
                                       float deg_per_motor_deg)
{
    if (!isfinite(deg_per_motor_deg) ||
        fabsf(deg_per_motor_deg) <= ARM_FLOAT_EPSILON) {
        return 0.0f;
    }
    return reference_joint_deg + motor_angle_deg * deg_per_motor_deg;
}

static void ArmUpdateFeedback(void)
{
    for (uint8_t i = 0u; i < 3u; ++i) {
        if (arm_motors[i] != NULL) {
            g_arm_state.motor_total_angle_deg[i] = arm_motors[i]->measure.total_angle;
            g_arm_state.motor_current[i] = (float)arm_motors[i]->measure.real_current;
            g_arm_state.motor_speed_dps[i] = arm_motors[i]->measure.speed_aps;
            g_arm_state.motor_online[i] = ArmMotorFeedbackReady(arm_motors[i]);
        } else {
            g_arm_state.motor_total_angle_deg[i] = 0.0f;
            g_arm_state.motor_current[i] = 0.0f;
            g_arm_state.motor_speed_dps[i] = 0.0f;
            g_arm_state.motor_online[i] = 0u;
        }
    }

    g_arm_state.q_feedback_deg[ARM_JOINT_BASE_YAW] = ArmBaseJointAngle();
    if (g_arm_calibration.joint_calibrated) {
        g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER] =
            ArmMotorToCalibratedJoint(arm_shoulder_motor->measure.total_angle,
                                      ARM_SHOULDER_REFERENCE_DEG,
                                      g_arm_calibration.shoulder_deg_per_motor_deg);
        g_arm_state.q_feedback_deg[ARM_JOINT_ELBOW] =
            ArmMotorToCalibratedJoint(arm_elbow_motor->measure.total_angle,
                                      ARM_ELBOW_REFERENCE_DEG,
                                      g_arm_calibration.elbow_deg_per_motor_deg);
    } else {
        g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER] = 0.0f;
        g_arm_state.q_feedback_deg[ARM_JOINT_ELBOW] = 0.0f;
    }
    g_arm_state.q_feedback_deg[ARM_JOINT_WRIST] = 0.0f;
}

void ArmForwardKinematics3DOF(float q1_deg,
                              float q2_deg,
                              float q3_deg,
                              Arm_Position_s *position)
{
    float q1;
    float q2;
    float q23;
    float radial;

    if (position == NULL) {
        return;
    }
    q1 = q1_deg * ARM_DEG_TO_RAD;
    q2 = q2_deg * ARM_DEG_TO_RAD;
    q23 = (q2_deg + q3_deg) * ARM_DEG_TO_RAD;
    radial = ARM_LINK_1_MM * cosf(q2) + ARM_LINK_2_MM * cosf(q23);
    position->x_mm = radial * cosf(q1);
    position->y_mm = radial * sinf(q1);
    position->z_mm = ARM_BASE_HEIGHT_MM +
                     ARM_LINK_1_MM * sinf(q2) +
                     ARM_LINK_2_MM * sinf(q23);
}

static void ArmUpdateForwardKinematics(void)
{
    ArmForwardKinematics3DOF(g_arm_state.q_feedback_deg[ARM_JOINT_BASE_YAW],
                             g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER],
                             g_arm_state.q_feedback_deg[ARM_JOINT_ELBOW],
                             &g_arm_state.wrist_center);
    g_arm_state.end_pitch_deg =
        g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER] +
        g_arm_state.q_feedback_deg[ARM_JOINT_ELBOW];
}

static uint8_t ArmCalibrationStateIsActive(Arm_Calibration_State_e state)
{
    return state >= ARM_CAL_ELBOW_FIND_REFERENCE &&
           state <= ARM_CAL_SHOULDER_DONE;
}

static void ArmLoadMeasuredJointMapping(void)
{
    g_arm_calibration.shoulder_hard_min_deg = ARM_SHOULDER_REFERENCE_DEG;
    g_arm_calibration.shoulder_hard_max_deg = ARM_SHOULDER_OPPOSITE_DEG;
    g_arm_calibration.elbow_hard_min_deg = ARM_ELBOW_OPPOSITE_DEG;
    g_arm_calibration.elbow_hard_max_deg = ARM_ELBOW_REFERENCE_DEG;
    g_arm_calibration.shoulder_soft_min_deg =
        g_arm_calibration.shoulder_hard_min_deg + g_arm_soft_limit_margin_deg;
    g_arm_calibration.shoulder_soft_max_deg =
        g_arm_calibration.shoulder_hard_max_deg - g_arm_soft_limit_margin_deg;
    g_arm_calibration.elbow_soft_min_deg =
        g_arm_calibration.elbow_hard_min_deg + g_arm_soft_limit_margin_deg;
    g_arm_calibration.elbow_soft_max_deg =
        g_arm_calibration.elbow_hard_max_deg - g_arm_soft_limit_margin_deg;
    g_arm_calibration.shoulder_motor_span_deg = ARM_SHOULDER_MEASURED_SPAN_DEG;
    g_arm_calibration.elbow_motor_span_deg = ARM_ELBOW_MEASURED_SPAN_DEG;
    g_arm_calibration.shoulder_deg_per_motor_deg =
        (ARM_SHOULDER_OPPOSITE_DEG - ARM_SHOULDER_REFERENCE_DEG) /
        ARM_SHOULDER_MEASURED_SPAN_DEG;
    g_arm_calibration.elbow_deg_per_motor_deg =
        (ARM_ELBOW_OPPOSITE_DEG - ARM_ELBOW_REFERENCE_DEG) /
        ARM_ELBOW_MEASURED_SPAN_DEG;
    g_arm_calibration.shoulder_direction =
        ArmSign(g_arm_calibration.shoulder_deg_per_motor_deg);
    g_arm_calibration.elbow_direction =
        ArmSign(g_arm_calibration.elbow_deg_per_motor_deg);
}

static uint8_t ArmUpdateMappingFromFullScan(void)
{
    if (fabsf(g_arm_calibration.shoulder_motor_span_deg) <= ARM_FLOAT_EPSILON ||
        fabsf(g_arm_calibration.elbow_motor_span_deg) <= ARM_FLOAT_EPSILON) {
        return 0u;
    }
    g_arm_calibration.shoulder_deg_per_motor_deg =
        (ARM_SHOULDER_OPPOSITE_DEG - ARM_SHOULDER_REFERENCE_DEG) /
        g_arm_calibration.shoulder_motor_span_deg;
    g_arm_calibration.elbow_deg_per_motor_deg =
        (ARM_ELBOW_OPPOSITE_DEG - ARM_ELBOW_REFERENCE_DEG) /
        g_arm_calibration.elbow_motor_span_deg;
    g_arm_calibration.shoulder_direction =
        ArmSign(g_arm_calibration.shoulder_deg_per_motor_deg);
    g_arm_calibration.elbow_direction =
        ArmSign(g_arm_calibration.elbow_deg_per_motor_deg);
    return isfinite(g_arm_calibration.shoulder_deg_per_motor_deg) &&
           isfinite(g_arm_calibration.elbow_deg_per_motor_deg);
}

static void ArmFinishJointHoming(void)
{
    ArmStopMotor(arm_shoulder_motor);
    ArmStopMotor(arm_elbow_motor);
    if (!g_arm_calibration.shoulder_reference_found ||
        !g_arm_calibration.elbow_reference_found ||
        (g_arm_state.homing_mode == ARM_HOMING_FULL_SCAN &&
         (!g_arm_calibration.shoulder_opposite_found ||
          !g_arm_calibration.elbow_opposite_found ||
          !ArmUpdateMappingFromFullScan()))) {
        ArmCalibrationFail(ARM_CAL_ERROR_TRAVEL);
        return;
    }
    g_arm_calibration.joint_calibrated = 1u;
    ArmUpdateCalibrationValid();
    g_arm_state.calibration_state = ARM_CAL_VALID;
    g_arm_state.mode = ARM_MODE_READY;
}

static void ArmEnterCalibrationState(Arm_Calibration_State_e state,
                                     DJIMotor_Instance *stage_motor,
                                     uint32_t now)
{
    g_arm_state.calibration_state = state;
    g_arm_state.calibration_stage_elapsed_ms = 0u;
    g_arm_state.calibration_stall_elapsed_ms = 0u;
    g_arm_state.calibration_stage_travel_motor_deg = 0.0f;
    g_arm_state.calibration_stall_condition = 0u;
    arm_cal_runtime.stage_start_tick = now;
    arm_cal_runtime.stall_start_tick = 0u;
    arm_cal_runtime.stage_start_motor_angle_deg =
        stage_motor != NULL ? stage_motor->measure.total_angle : 0.0f;
}

static void ArmCalibrationFail(Arm_Calibration_State_e error_state)
{
    ArmStopMotor(arm_shoulder_motor);
    ArmStopMotor(arm_elbow_motor);
    ArmClearMotorController(arm_shoulder_motor);
    ArmClearMotorController(arm_elbow_motor);
    g_arm_calibration.joint_calibrated = 0u;
    ArmUpdateCalibrationValid();
    g_arm_state.calibration_stall_condition = 0u;
    g_arm_state.calibration_state = error_state;
    g_arm_state.mode = ARM_MODE_SAFE;
}

static uint8_t ArmStallCondition(const DJIMotor_Instance *motor,
                                 float current_threshold)
{
    return fabsf(motor->measure.speed_aps) <= g_arm_homing_stall_speed_dps &&
           fabsf((float)motor->measure.real_current) >= current_threshold;
}

static void ArmUpdatePeakCurrent(Arm_Calibration_Motor_e channel,
                                 const DJIMotor_Instance *motor)
{
    int32_t current = (int32_t)motor->measure.real_current;
    if (current < 0) {
        current = -current;
    }
    if (current > 32767) {
        current = 32767;
    }
    if (current > g_arm_state.calibration_peak_abs_current[channel]) {
        g_arm_state.calibration_peak_abs_current[channel] = (int16_t)current;
    }
}

static uint8_t ArmStageProtectionTriggered(DJIMotor_Instance *motor,
                                           float ratio,
                                           float max_joint_travel_deg,
                                           uint32_t timeout_ms,
                                           uint32_t now)
{
    float max_motor_travel;

    g_arm_state.calibration_stage_elapsed_ms =
        (uint32_t)(now - arm_cal_runtime.stage_start_tick);
    g_arm_state.calibration_stage_travel_motor_deg =
        fabsf(motor->measure.total_angle -
              arm_cal_runtime.stage_start_motor_angle_deg);

    if (timeout_ms > 0u &&
        g_arm_state.calibration_stage_elapsed_ms >= timeout_ms) {
        ArmCalibrationFail(ARM_CAL_ERROR_TIMEOUT);
        return 1u;
    }

    max_motor_travel = ratio * max_joint_travel_deg;
    if (!isfinite(ratio) || !isfinite(max_joint_travel_deg) ||
        ratio <= ARM_FLOAT_EPSILON || max_joint_travel_deg <= 0.0f ||
        !isfinite(max_motor_travel) ||
        g_arm_state.calibration_stage_travel_motor_deg >= max_motor_travel) {
        ArmCalibrationFail(ARM_CAL_ERROR_TRAVEL);
        return 1u;
    }
    return 0u;
}

static uint8_t ArmConfirmedStall(DJIMotor_Instance *motor,
                                 Arm_Calibration_Motor_e channel,
                                 float current_threshold,
                                 uint32_t now)
{
    if (g_arm_state.calibration_stage_elapsed_ms < g_arm_homing_spinup_ms) {
        g_arm_state.calibration_stall_condition = 0u;
        arm_cal_runtime.stall_start_tick = 0u;
        g_arm_state.calibration_stall_elapsed_ms = 0u;
        return 0u;
    }

    g_arm_state.calibration_stall_condition =
        ArmStallCondition(motor, current_threshold);
    if (!g_arm_state.calibration_stall_condition) {
        arm_cal_runtime.stall_start_tick = 0u;
        g_arm_state.calibration_stall_elapsed_ms = 0u;
        return 0u;
    }

    if (arm_cal_runtime.stall_start_tick == 0u) {
        arm_cal_runtime.stall_start_tick = now;
    }
    g_arm_state.calibration_stall_elapsed_ms =
        (uint32_t)(now - arm_cal_runtime.stall_start_tick);
    if (g_arm_state.calibration_stall_elapsed_ms <
        g_arm_homing_stall_confirm_ms) {
        return 0u;
    }

    g_arm_state.calibration_trigger_current[channel] =
        motor->measure.real_current;
    g_arm_state.calibration_trigger_speed_dps[channel] =
        motor->measure.speed_aps;
    return 1u;
}

static void ArmResetMotorAtReference(DJIMotor_Instance *motor)
{
    uint32_t primask;

    ArmStopMotor(motor);
    primask = __get_PRIMASK();
    __disable_irq();
    DJIMotorReset(motor);
    if (primask == 0u) {
        __enable_irq();
    }
    ArmClearMotorController(motor);
}

static void ArmRunFindReference(DJIMotor_Instance *motor,
                                DJIMotor_Instance *inactive_motor,
                                Arm_Calibration_Motor_e channel,
                                float speed_dps,
                                float reference_direction,
                                float current_threshold,
                                float ratio,
                                float max_joint_travel_deg,
                                uint32_t timeout_ms,
                                Arm_Calibration_State_e settle_state,
                                uint32_t now)
{
    ArmStopMotor(inactive_motor);
    if (!isfinite(speed_dps) || !isfinite(current_threshold) ||
        fabsf(speed_dps) <= ARM_FLOAT_EPSILON || current_threshold < 0.0f) {
        ArmCalibrationFail(ARM_CAL_ERROR_TRAVEL);
        return;
    }
    ArmUpdatePeakCurrent(channel, motor);
    if (ArmStageProtectionTriggered(motor, ratio, max_joint_travel_deg,
                                    timeout_ms, now)) {
        return;
    }
    DJIMotorSetRef(motor, ArmSign(reference_direction) * fabsf(speed_dps));
    DJIMotorEnable(motor);
    if (!ArmConfirmedStall(motor, channel, current_threshold, now)) {
        return;
    }

    /* Remove torque immediately. The encoder zero is set after a 2 s settle. */
    ArmStopMotor(motor);
    ArmClearMotorController(motor);
    ArmEnterCalibrationState(settle_state, motor, now);
}

static uint8_t ArmRunSettleReference(DJIMotor_Instance *motor,
                                     DJIMotor_Instance *inactive_motor,
                                     Arm_Calibration_Motor_e channel,
                                     uint32_t now)
{
    ArmStopMotor(motor);
    ArmStopMotor(inactive_motor);
    g_arm_state.calibration_stage_elapsed_ms =
        (uint32_t)(now - arm_cal_runtime.stage_start_tick);
    if (g_arm_state.calibration_stage_elapsed_ms < ARM_REFERENCE_SETTLE_MS) {
        return 0u;
    }

    ArmResetMotorAtReference(motor);
    if (channel == ARM_CAL_MOTOR_SHOULDER) {
        g_arm_calibration.shoulder_reference_found = 1u;
    } else {
        g_arm_calibration.elbow_reference_found = 1u;
    }
    return 1u;
}

static void ArmRunReleaseReference(DJIMotor_Instance *motor,
                                   DJIMotor_Instance *inactive_motor,
                                   float speed_dps,
                                   float reference_direction,
                                   float ratio,
                                   float max_joint_travel_deg,
                                   uint32_t timeout_ms,
                                   Arm_Calibration_State_e find_opposite_state,
                                   uint32_t now)
{
    float released_joint_deg;

    ArmStopMotor(inactive_motor);
    if (!isfinite(speed_dps) || fabsf(speed_dps) <= ARM_FLOAT_EPSILON ||
        !isfinite(g_arm_release_joint_deg) || g_arm_release_joint_deg < 0.0f) {
        ArmCalibrationFail(ARM_CAL_ERROR_TRAVEL);
        return;
    }
    if (ArmStageProtectionTriggered(motor, ratio, max_joint_travel_deg,
                                    timeout_ms, now)) {
        return;
    }
    DJIMotorSetRef(motor, -ArmSign(reference_direction) * fabsf(speed_dps));
    DJIMotorEnable(motor);
    released_joint_deg = ratio > ARM_FLOAT_EPSILON ?
                         fabsf(motor->measure.total_angle) / ratio : 0.0f;
    if (released_joint_deg >= g_arm_release_joint_deg ||
        g_arm_state.calibration_stage_elapsed_ms >= g_arm_release_min_ms) {
        ArmEnterCalibrationState(find_opposite_state, motor, now);
    }
}

static void ArmRunFindOpposite(DJIMotor_Instance *motor,
                               DJIMotor_Instance *inactive_motor,
                               Arm_Calibration_Motor_e channel,
                               float speed_dps,
                               float reference_direction,
                               float current_threshold,
                               float ratio,
                               float max_joint_travel_deg,
                               uint32_t timeout_ms,
                               Arm_Calibration_State_e done_state,
                               uint32_t now)
{
    float max_motor_travel;
    float motor_span;

    ArmStopMotor(inactive_motor);
    if (!isfinite(speed_dps) || !isfinite(current_threshold) ||
        fabsf(speed_dps) <= ARM_FLOAT_EPSILON || current_threshold < 0.0f) {
        ArmCalibrationFail(ARM_CAL_ERROR_TRAVEL);
        return;
    }
    ArmUpdatePeakCurrent(channel, motor);
    g_arm_state.calibration_stage_elapsed_ms =
        (uint32_t)(now - arm_cal_runtime.stage_start_tick);
    g_arm_state.calibration_stage_travel_motor_deg =
        fabsf(motor->measure.total_angle);
    if (timeout_ms > 0u &&
        g_arm_state.calibration_stage_elapsed_ms >= timeout_ms) {
        ArmCalibrationFail(ARM_CAL_ERROR_TIMEOUT);
        return;
    }
    if (!isfinite(ratio) || !isfinite(max_joint_travel_deg) ||
        ratio <= ARM_FLOAT_EPSILON || max_joint_travel_deg <= 0.0f) {
        ArmCalibrationFail(ARM_CAL_ERROR_TRAVEL);
        return;
    }
    max_motor_travel = ratio * max_joint_travel_deg;
    if (!isfinite(max_motor_travel) ||
        g_arm_state.calibration_stage_travel_motor_deg >= max_motor_travel) {
        ArmCalibrationFail(ARM_CAL_ERROR_TRAVEL);
        return;
    }
    DJIMotorSetRef(motor, -ArmSign(reference_direction) * fabsf(speed_dps));
    DJIMotorEnable(motor);
    if (!ArmConfirmedStall(motor, channel, current_threshold, now)) {
        return;
    }

    motor_span = motor->measure.total_angle;
    ArmStopMotor(motor);
    ArmClearMotorController(motor);
    if (fabsf(motor_span) / ratio <=
        2.0f * g_arm_soft_limit_margin_deg + ARM_RANGE_EPSILON_DEG) {
        ArmCalibrationFail(ARM_CAL_ERROR_TRAVEL);
        return;
    }

    if (channel == ARM_CAL_MOTOR_SHOULDER) {
        g_arm_calibration.shoulder_motor_span_deg = motor_span;
        g_arm_calibration.shoulder_opposite_found = 1u;
    } else {
        g_arm_calibration.elbow_motor_span_deg = motor_span;
        g_arm_calibration.elbow_opposite_found = 1u;
    }
    ArmEnterCalibrationState(done_state, NULL, now);
}

static void ArmStartHomingMode(Arm_Homing_Mode_e mode)
{
    uint32_t now = HAL_GetTick();

    arm_auto_calibration_attempted = 1u;
    ArmStop();
    ArmLoadMeasuredJointMapping();
    g_arm_state.homing_mode = mode;
    g_arm_calibration.joint_calibrated = 0u;
    g_arm_calibration.shoulder_reference_found = 0u;
    g_arm_calibration.shoulder_opposite_found = 0u;
    g_arm_calibration.elbow_reference_found = 0u;
    g_arm_calibration.elbow_opposite_found = 0u;
    if (mode == ARM_HOMING_FULL_SCAN) {
        g_arm_calibration.shoulder_motor_span_deg = 0.0f;
        g_arm_calibration.elbow_motor_span_deg = 0.0f;
    }
    g_arm_calibration.shoulder_teach_motor_deg = 0.0f;
    g_arm_calibration.elbow_teach_motor_deg = 0.0f;
    memset(g_arm_state.calibration_peak_abs_current, 0,
           sizeof(g_arm_state.calibration_peak_abs_current));
    memset(g_arm_state.calibration_trigger_current, 0,
           sizeof(g_arm_state.calibration_trigger_current));
    memset(g_arm_state.calibration_trigger_speed_dps, 0,
           sizeof(g_arm_state.calibration_trigger_speed_dps));
    ArmUpdateCalibrationValid();

    if (!ArmMotorFeedbackReady(arm_shoulder_motor) ||
        !ArmMotorFeedbackReady(arm_elbow_motor)) {
        ArmCalibrationFail(ARM_CAL_ERROR_OFFLINE);
        return;
    }
    if (!isfinite(g_arm_shoulder_motor_to_joint_ratio) ||
        !isfinite(g_arm_elbow_motor_to_joint_ratio) ||
        g_arm_shoulder_motor_to_joint_ratio <= ARM_FLOAT_EPSILON ||
        g_arm_elbow_motor_to_joint_ratio <= ARM_FLOAT_EPSILON) {
        ArmCalibrationFail(ARM_CAL_ERROR_TRAVEL);
        return;
    }

    g_arm_state.mode = ARM_MODE_CALIBRATION;
    ArmClearMotorController(arm_shoulder_motor);
    ArmClearMotorController(arm_elbow_motor);
    ArmEnterCalibrationState(ARM_CAL_ELBOW_FIND_REFERENCE,
                             arm_elbow_motor, now);
}

void ArmHomingStart(void)
{
    ArmStartHomingMode(ARM_HOMING_SINGLE_REFERENCE);
}

void ArmCalibrationStart(void)
{
    ArmStartHomingMode(ARM_HOMING_FULL_SCAN);
}

void ArmCalibrationAbort(void)
{
    ArmCalibrationFail(ARM_CAL_ERROR_ABORT);
}

static void ArmCalibrationTask(uint32_t now)
{
    Arm_Calibration_State_e state = g_arm_state.calibration_state;

    if (!ArmCalibrationStateIsActive(state)) {
        ArmStopMotor(arm_shoulder_motor);
        ArmStopMotor(arm_elbow_motor);
        return;
    }
    if (!ArmMotorFeedbackReady(arm_shoulder_motor) ||
        !ArmMotorFeedbackReady(arm_elbow_motor)) {
        ArmCalibrationFail(ARM_CAL_ERROR_OFFLINE);
        return;
    }

    switch (state) {
        case ARM_CAL_ELBOW_FIND_REFERENCE:
            ArmRunFindReference(arm_elbow_motor, arm_shoulder_motor,
                                ARM_CAL_MOTOR_ELBOW,
                                g_arm_elbow_homing_speed_dps,
                                ARM_ELBOW_MIN_DIRECTION,
                                g_arm_elbow_stall_current,
                                g_arm_elbow_motor_to_joint_ratio,
                                g_arm_elbow_max_joint_travel_deg,
                                g_arm_elbow_stage_timeout_ms,
                                ARM_CAL_ELBOW_SETTLE_REFERENCE, now);
            break;
        case ARM_CAL_ELBOW_SETTLE_REFERENCE:
            if (ArmRunSettleReference(arm_elbow_motor, arm_shoulder_motor,
                                      ARM_CAL_MOTOR_ELBOW, now)) {
                if (g_arm_state.homing_mode == ARM_HOMING_FULL_SCAN) {
                    ArmEnterCalibrationState(ARM_CAL_ELBOW_RELEASE_REFERENCE,
                                             arm_elbow_motor, now);
                } else {
                    ArmEnterCalibrationState(ARM_CAL_ELBOW_DONE, NULL, now);
                }
            }
            break;
        case ARM_CAL_ELBOW_RELEASE_REFERENCE:
            ArmRunReleaseReference(arm_elbow_motor, arm_shoulder_motor,
                                   g_arm_elbow_homing_speed_dps,
                                   ARM_ELBOW_MIN_DIRECTION,
                                   g_arm_elbow_motor_to_joint_ratio,
                                   g_arm_elbow_max_joint_travel_deg,
                                   g_arm_elbow_stage_timeout_ms,
                                   ARM_CAL_ELBOW_FIND_OPPOSITE, now);
            break;
        case ARM_CAL_ELBOW_FIND_OPPOSITE:
            ArmRunFindOpposite(arm_elbow_motor, arm_shoulder_motor,
                               ARM_CAL_MOTOR_ELBOW,
                               g_arm_elbow_homing_speed_dps,
                               ARM_ELBOW_MIN_DIRECTION,
                               g_arm_elbow_stall_current,
                               g_arm_elbow_motor_to_joint_ratio,
                               g_arm_elbow_max_joint_travel_deg,
                               g_arm_elbow_stage_timeout_ms,
                               ARM_CAL_ELBOW_DONE, now);
            break;
        case ARM_CAL_ELBOW_DONE:
            ArmStopMotor(arm_elbow_motor);
            ArmEnterCalibrationState(ARM_CAL_SHOULDER_FIND_REFERENCE,
                                     arm_shoulder_motor, now);
            break;
        case ARM_CAL_SHOULDER_FIND_REFERENCE:
            ArmRunFindReference(arm_shoulder_motor, arm_elbow_motor,
                              ARM_CAL_MOTOR_SHOULDER,
                              g_arm_shoulder_homing_speed_dps,
                              ARM_SHOULDER_MIN_DIRECTION,
                              g_arm_shoulder_stall_current,
                              g_arm_shoulder_motor_to_joint_ratio,
                              g_arm_shoulder_max_joint_travel_deg,
                              g_arm_shoulder_stage_timeout_ms,
                              ARM_CAL_SHOULDER_SETTLE_REFERENCE, now);
            break;
        case ARM_CAL_SHOULDER_SETTLE_REFERENCE:
            if (ArmRunSettleReference(arm_shoulder_motor, arm_elbow_motor,
                                      ARM_CAL_MOTOR_SHOULDER, now)) {
                if (g_arm_state.homing_mode == ARM_HOMING_FULL_SCAN) {
                    ArmEnterCalibrationState(ARM_CAL_SHOULDER_RELEASE_REFERENCE,
                                             arm_shoulder_motor, now);
                } else {
                    ArmEnterCalibrationState(ARM_CAL_SHOULDER_DONE, NULL, now);
                }
            }
            break;
        case ARM_CAL_SHOULDER_RELEASE_REFERENCE:
            ArmRunReleaseReference(arm_shoulder_motor, arm_elbow_motor,
                                   g_arm_shoulder_homing_speed_dps,
                                   ARM_SHOULDER_MIN_DIRECTION,
                                   g_arm_shoulder_motor_to_joint_ratio,
                                   g_arm_shoulder_max_joint_travel_deg,
                                   g_arm_shoulder_stage_timeout_ms,
                                   ARM_CAL_SHOULDER_FIND_OPPOSITE, now);
            break;
        case ARM_CAL_SHOULDER_FIND_OPPOSITE:
            ArmRunFindOpposite(arm_shoulder_motor, arm_elbow_motor,
                               ARM_CAL_MOTOR_SHOULDER,
                               g_arm_shoulder_homing_speed_dps,
                               ARM_SHOULDER_MIN_DIRECTION,
                               g_arm_shoulder_stall_current,
                               g_arm_shoulder_motor_to_joint_ratio,
                               g_arm_shoulder_max_joint_travel_deg,
                               g_arm_shoulder_stage_timeout_ms,
                               ARM_CAL_SHOULDER_DONE, now);
            break;
        case ARM_CAL_SHOULDER_DONE:
            ArmStopMotor(arm_shoulder_motor);
            ArmFinishJointHoming();
            break;
        default:
            ArmCalibrationFail(ARM_CAL_ERROR_ABORT);
            break;
    }
}

static uint8_t ArmCandidateWithinLimits(float q2_deg, float q3_deg)
{
    return q2_deg >= g_arm_calibration.shoulder_soft_min_deg &&
           q2_deg <= g_arm_calibration.shoulder_soft_max_deg &&
           q3_deg >= g_arm_calibration.elbow_soft_min_deg &&
           q3_deg <= g_arm_calibration.elbow_soft_max_deg;
}

static float ArmCandidateScore(const float q_deg[3],
                               const float current_q_deg[3])
{
    float shoulder_span = g_arm_calibration.shoulder_soft_max_deg -
                          g_arm_calibration.shoulder_soft_min_deg;
    float elbow_span = g_arm_calibration.elbow_soft_max_deg -
                       g_arm_calibration.elbow_soft_min_deg;
    float dq1 = ArmAngleDifference(q_deg[0], current_q_deg[0]) / 360.0f;
    float dq2 = (q_deg[1] - current_q_deg[1]) / shoulder_span;
    float dq3 = (q_deg[2] - current_q_deg[2]) / elbow_span;
    return dq1 * dq1 + dq2 * dq2 + dq3 * dq3;
}

Arm_IK_Status_e ArmInverseKinematics3DOF(const Arm_Position_s *target,
                                         const float current_q_deg[3],
                                         Arm_IK_Result_s *result)
{
    float rho;
    float z_planar;
    float cos_q3;
    float base_yaw;
    float best_score = 0.0f;
    uint8_t found = 0u;

    if (result == NULL) {
        return ARM_IK_INVALID_ARGUMENT;
    }
    memset(result, 0, sizeof(*result));
    if (target == NULL || current_q_deg == NULL ||
        !isfinite(target->x_mm) || !isfinite(target->y_mm) ||
        !isfinite(target->z_mm) ||
        !isfinite(current_q_deg[0]) || !isfinite(current_q_deg[1]) ||
        !isfinite(current_q_deg[2])) {
        result->status = ARM_IK_INVALID_ARGUMENT;
        return result->status;
    }
    if (!g_arm_calibration.calibration_valid) {
        result->status = ARM_IK_NOT_CALIBRATED;
        return result->status;
    }

    rho = sqrtf(target->x_mm * target->x_mm +
                target->y_mm * target->y_mm);
    z_planar = target->z_mm - ARM_BASE_HEIGHT_MM;
    cos_q3 = (rho * rho + z_planar * z_planar -
              ARM_LINK_1_MM * ARM_LINK_1_MM -
              ARM_LINK_2_MM * ARM_LINK_2_MM) /
             (2.0f * ARM_LINK_1_MM * ARM_LINK_2_MM);
    if (cos_q3 < -1.0f - ARM_FLOAT_EPSILON ||
        cos_q3 > 1.0f + ARM_FLOAT_EPSILON) {
        result->status = ARM_IK_OUT_OF_REACH;
        return result->status;
    }
    cos_q3 = ArmClampFloat(cos_q3, -1.0f, 1.0f);
    base_yaw = rho > ARM_FLOAT_EPSILON ?
               atan2f(target->y_mm, target->x_mm) * ARM_RAD_TO_DEG :
               current_q_deg[0];

    for (uint8_t radial_index = 0u; radial_index < 2u; ++radial_index) {
        float signed_radius;
        float q1_deg;

        if (rho <= ARM_FLOAT_EPSILON && radial_index == 1u) {
            continue;
        }
        signed_radius = radial_index == 0u ? rho : -rho;
        q1_deg = radial_index == 0u ? base_yaw : base_yaw + 180.0f;
        q1_deg = ArmWrapTo180(q1_deg);

        for (uint8_t elbow_index = 0u; elbow_index < 2u; ++elbow_index) {
            float q3_rad = acosf(cos_q3);
            float q2_rad;
            float candidate[3];
            float score;

            if (elbow_index != 0u) {
                q3_rad = -q3_rad;
            }
            q2_rad = atan2f(z_planar, signed_radius) -
                     atan2f(ARM_LINK_2_MM * sinf(q3_rad),
                            ARM_LINK_1_MM +
                            ARM_LINK_2_MM * cosf(q3_rad));
            candidate[0] = q1_deg;
            candidate[1] = q2_rad * ARM_RAD_TO_DEG;
            candidate[2] = q3_rad * ARM_RAD_TO_DEG;
            if (!ArmCandidateWithinLimits(candidate[1], candidate[2])) {
                continue;
            }

            result->candidate_count++;
            score = ArmCandidateScore(candidate, current_q_deg);
            if (!found || score < best_score) {
                found = 1u;
                best_score = score;
                result->q_deg[0] = candidate[0];
                result->q_deg[1] = candidate[1];
                result->q_deg[2] = candidate[2];
            }
        }
    }

    if (!found) {
        result->status = ARM_IK_NO_LIMITED_SOLUTION;
        return result->status;
    }

    ArmForwardKinematics3DOF(result->q_deg[0], result->q_deg[1],
                             result->q_deg[2], &result->fk_position);
    result->position_error_mm =
        sqrtf((result->fk_position.x_mm - target->x_mm) *
              (result->fk_position.x_mm - target->x_mm) +
              (result->fk_position.y_mm - target->y_mm) *
              (result->fk_position.y_mm - target->y_mm) +
              (result->fk_position.z_mm - target->z_mm) *
              (result->fk_position.z_mm - target->z_mm));
    result->status = ARM_IK_OK;
    return result->status;
}

uint8_t ArmBaseTeachFront(void)
{
    if (!ArmMotorFeedbackReady(arm_base_motor)) {
        g_arm_calibration.base_calibrated = 0u;
        ArmUpdateCalibrationValid();
        return 0u;
    }
    arm_base_front_raw_deg = arm_base_motor->measure.angle_single_round;
    g_arm_calibration.base_calibrated = 1u;
    ArmUpdateCalibrationValid();
    return 1u;
}

static void ArmAutoCalibrationTask(uint32_t now)
{
    if (arm_auto_calibration_attempted ||
        g_arm_state.calibration_state != ARM_CAL_IDLE) {
        return;
    }
    if (!ArmMotorFeedbackReady(arm_shoulder_motor) ||
        !ArmMotorFeedbackReady(arm_elbow_motor)) {
        arm_auto_online_waiting = 0u;
        return;
    }
    if (!arm_auto_online_waiting) {
        arm_auto_online_waiting = 1u;
        arm_auto_online_start_tick = now;
        return;
    }
    if ((uint32_t)(now - arm_auto_online_start_tick) >=
        ARM_AUTO_START_ONLINE_MS) {
        arm_auto_calibration_attempted = 1u;
        if (ARM_BOOT_FULL_SCAN != 0u) {
            ArmCalibrationStart();
        } else {
            ArmHomingStart();
        }
    }
}

static void ArmRatioTestEnterState(Arm_Ratio_Test_State_e state,
                                   uint32_t now)
{
    g_arm_state.ratio_test_state = state;
    g_arm_state.ratio_test_elapsed_ms = 0u;
    arm_ratio_test_state_tick = now;
    arm_ratio_test_stall_tick = 0u;
}

static void ArmRatioTestFail(Arm_Ratio_Test_State_e state, uint32_t now)
{
    ArmStopMotor(arm_base_motor);
    ArmStopMotor(arm_shoulder_motor);
    ArmStopMotor(arm_elbow_motor);
    ArmClearMotorController(arm_shoulder_motor);
    ArmRatioTestEnterState(state, now);
    g_arm_state.mode = ARM_MODE_SAFE;
}

static void Arm3508RatioTestTask(uint32_t now)
{
    float signed_progress;
    float target_motor_deg = ARM_3508_RATIO_TEST_RATIO *
                             ARM_3508_RATIO_TEST_OUTPUT_DEG;
    uint8_t stalled;

    ArmStopMotor(arm_base_motor);
    ArmStopMotor(arm_elbow_motor);
    g_arm_state.homing_mode = ARM_HOMING_NONE;
    g_arm_state.ratio_test_target_motor_deg = target_motor_deg;
    g_arm_state.ratio_test_expected_output_deg =
        ARM_3508_RATIO_TEST_OUTPUT_DEG;
    g_arm_state.ratio_test_motor_angle_deg =
        arm_shoulder_motor != NULL ?
        arm_shoulder_motor->measure.total_angle : 0.0f;
    g_arm_state.ratio_test_elapsed_ms =
        (uint32_t)(now - arm_ratio_test_state_tick);

    switch (g_arm_state.ratio_test_state) {
        case ARM_RATIO_TEST_WAIT_ONLINE:
            ArmStopMotor(arm_shoulder_motor);
            if (!ArmMotorFeedbackReady(arm_shoulder_motor)) {
                arm_auto_online_waiting = 0u;
                return;
            }
            if (!arm_auto_online_waiting) {
                arm_auto_online_waiting = 1u;
                arm_auto_online_start_tick = now;
                return;
            }
            if ((uint32_t)(now - arm_auto_online_start_tick) >=
                ARM_AUTO_START_ONLINE_MS) {
                ArmRatioTestEnterState(ARM_RATIO_TEST_SETTLE_ZERO, now);
            }
            break;

        case ARM_RATIO_TEST_SETTLE_ZERO:
            ArmStopMotor(arm_shoulder_motor);
            if (!ArmMotorFeedbackReady(arm_shoulder_motor)) {
                ArmRatioTestFail(ARM_RATIO_TEST_ERROR_OFFLINE, now);
                return;
            }
            if (g_arm_state.ratio_test_elapsed_ms >=
                ARM_3508_RATIO_TEST_SETTLE_MS) {
                ArmResetMotorAtReference(arm_shoulder_motor);
                ArmRatioTestEnterState(ARM_RATIO_TEST_RUNNING, now);
            }
            break;

        case ARM_RATIO_TEST_RUNNING:
            if (!ArmMotorFeedbackReady(arm_shoulder_motor)) {
                ArmRatioTestFail(ARM_RATIO_TEST_ERROR_OFFLINE, now);
                return;
            }
            signed_progress = ARM_3508_RATIO_TEST_DIRECTION *
                              arm_shoulder_motor->measure.total_angle;
            g_arm_state.ratio_test_motor_angle_deg =
                arm_shoulder_motor->measure.total_angle;
            if (signed_progress >= target_motor_deg) {
                ArmStopMotor(arm_shoulder_motor);
                ArmClearMotorController(arm_shoulder_motor);
                ArmRatioTestEnterState(ARM_RATIO_TEST_COMPLETE, now);
                return;
            }
            if (signed_progress < -ARM_3508_RATIO_TEST_OVERRUN_DEG ||
                signed_progress >
                    target_motor_deg + ARM_3508_RATIO_TEST_OVERRUN_DEG) {
                ArmRatioTestFail(ARM_RATIO_TEST_ERROR_TIMEOUT, now);
                return;
            }
            if (g_arm_state.ratio_test_elapsed_ms >=
                ARM_3508_RATIO_TEST_TIMEOUT_MS) {
                ArmRatioTestFail(ARM_RATIO_TEST_ERROR_TIMEOUT, now);
                return;
            }

            stalled =
                fabsf((float)arm_shoulder_motor->measure.real_current) >=
                    g_arm_shoulder_stall_current &&
                fabsf(arm_shoulder_motor->measure.speed_aps) <=
                    g_arm_homing_stall_speed_dps;
            if (g_arm_state.ratio_test_elapsed_ms < g_arm_homing_spinup_ms ||
                !stalled) {
                arm_ratio_test_stall_tick = 0u;
            } else if (arm_ratio_test_stall_tick == 0u) {
                arm_ratio_test_stall_tick = now;
            } else if ((uint32_t)(now - arm_ratio_test_stall_tick) >=
                       ARM_3508_RATIO_TEST_STALL_MS) {
                ArmRatioTestFail(ARM_RATIO_TEST_ERROR_STALL, now);
                return;
            }

            DJIMotorSetRef(arm_shoulder_motor,
                           ARM_3508_RATIO_TEST_DIRECTION *
                           ARM_3508_RATIO_TEST_SPEED_DPS);
            DJIMotorEnable(arm_shoulder_motor);
            g_arm_state.mode = ARM_MODE_CALIBRATION;
            break;

        case ARM_RATIO_TEST_COMPLETE:
        case ARM_RATIO_TEST_ERROR_OFFLINE:
        case ARM_RATIO_TEST_ERROR_STALL:
        case ARM_RATIO_TEST_ERROR_TIMEOUT:
        case ARM_RATIO_TEST_ABORTED:
            ArmStopMotor(arm_shoulder_motor);
            g_arm_state.ratio_test_motor_angle_deg =
                arm_shoulder_motor != NULL ?
                arm_shoulder_motor->measure.total_angle : 0.0f;
            g_arm_state.mode = ARM_MODE_SAFE;
            break;

        default:
            ArmRatioTestFail(ARM_RATIO_TEST_ERROR_TIMEOUT, now);
            break;
    }
}

void ArmWristPWMInit(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    ArmWristPWMSetUs(ARM_WRIST_PWM_MID_US);
}

void ArmWristPWMSetUs(uint16_t pulse_us)
{
    pulse_us = (uint16_t)ArmClampFloat((float)pulse_us,
                                      (float)ARM_WRIST_PWM_MIN_US,
                                      (float)ARM_WRIST_PWM_MAX_US);
    g_arm_state.wrist_pwm_us = pulse_us;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
}

void ArmWristPWMSetAngle(float angle_deg)
{
    float clamped = ArmClampFloat(angle_deg, -90.0f, 90.0f);
    float pulse = (float)ARM_WRIST_PWM_MIN_US +
                  (clamped + 90.0f) *
                  ((float)(ARM_WRIST_PWM_MAX_US - ARM_WRIST_PWM_MIN_US) /
                   180.0f);
    ArmWristPWMSetUs((uint16_t)(pulse + 0.5f));
}

void ArmInit(void)
{
    Motor_Init_Config_s base_config = ArmBaseMotorConfig();
    Motor_Init_Config_s shoulder_config = ArmShoulderMotorConfig();
    Motor_Init_Config_s elbow_config = ArmElbowMotorConfig();

    memset(&g_arm_state, 0, sizeof(g_arm_state));
    memset(&g_arm_calibration, 0, sizeof(g_arm_calibration));
    arm_base_motor = DJIMotorInit(&base_config);
    arm_shoulder_motor = DJIMotorInit(&shoulder_config);
    arm_elbow_motor = DJIMotorInit(&elbow_config);
    arm_motors[0] = arm_base_motor;
    arm_motors[1] = arm_shoulder_motor;
    arm_motors[2] = arm_elbow_motor;
    g_arm_state.mode = ARM_MODE_SAFE;
    g_arm_state.calibration_state = ARM_CAL_IDLE;
    g_arm_state.wrist_pwm_us = 0u;
    g_arm_homing_abort = 0u;
    arm_auto_calibration_attempted = 0u;
    arm_auto_online_waiting = 0u;
    arm_auto_online_start_tick = 0u;
    arm_base_front_raw_deg = 0.0f;
    arm_base_direction = 1.0f;
    arm_ratio_test_state_tick = HAL_GetTick();
    arm_ratio_test_stall_tick = 0u;
    g_arm_state.ratio_test_state = arm_3508_ratio_test_enable != 0u ?
        ARM_RATIO_TEST_WAIT_ONLINE : ARM_RATIO_TEST_DISABLED;
    ArmStop();
    arm_initialized = 1u;
}

void ArmStop(void)
{
    for (uint8_t i = 0u; i < 3u; ++i) {
        ArmStopMotor(arm_motors[i]);
    }
    g_arm_state.mode = ARM_MODE_SAFE;
}

void ArmTask(void)
{
    uint32_t now = HAL_GetTick();

    if (!arm_initialized) {
        return;
    }
    ArmStopMotor(arm_base_motor);
    ArmUpdateFeedback();

    if (g_arm_homing_abort) {
        arm_auto_calibration_attempted = 1u;
        if (arm_3508_ratio_test_enable != 0u) {
            ArmRatioTestFail(ARM_RATIO_TEST_ABORTED, now);
        } else {
            ArmCalibrationAbort();
        }
        ArmUpdateFeedback();
        ArmUpdateForwardKinematics();
        return;
    }

    if (arm_3508_ratio_test_enable != 0u) {
        Arm3508RatioTestTask(now);
        ArmUpdateFeedback();
        ArmUpdateForwardKinematics();
        return;
    }

    ArmAutoCalibrationTask(now);
    ArmCalibrationTask(now);
    ArmUpdateFeedback();
    ArmUpdateForwardKinematics();

    if (g_arm_calibration.joint_calibrated &&
        g_arm_state.calibration_state == ARM_CAL_VALID) {
        g_arm_state.mode = ARM_MODE_READY;
    } else if (ArmCalibrationStateIsActive(g_arm_state.calibration_state)) {
        g_arm_state.mode = ARM_MODE_CALIBRATION;
    } else {
        g_arm_state.mode = ARM_MODE_SAFE;
    }
}

const Arm_State_s *ArmGetState(void)
{
    return &g_arm_state;
}
