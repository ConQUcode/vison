#include "arm.h"
#include "arm_config.h"
#include "arm_internal.h"
#include "arm_kinematics.h"
#include "arm_trajectory.h"
#include "arm_wrist.h"

#include "DJI_motor.h"
#include "can.h"
#include "daemon.h"
#include "math.h"
#include "string.h"

#define ARM_PI                         3.14159265358979323846f
#define ARM_DEG_TO_RAD                 (ARM_PI / 180.0f)
#define ARM_RAD_TO_DEG                 (180.0f / ARM_PI)
#define ARM_FLOAT_EPSILON              0.0001f
#define ARM_RANGE_EPSILON_DEG          0.5f

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
static float arm_base_front_raw_deg = ARM_BASE_FRONT_RAW_DEG;
static float arm_base_direction = ARM_BASE_DIRECTION;
static uint32_t arm_ratio_test_state_tick;
static uint32_t arm_ratio_test_stall_tick;
static uint32_t arm_soft_limit_state_tick;
static uint32_t arm_soft_limit_overcurrent_tick;
static float arm_joint_hold_target_motor_deg[2];
static float arm_base_last_target_error_deg;
static uint8_t arm_base_target_error_valid;
static uint8_t arm_teach_mode_entered;
static uint8_t arm_homing_speed_pid_loaded;
static PID_Init_Config_s arm_shoulder_normal_speed_pid;
static PID_Init_Config_s arm_elbow_normal_speed_pid;

/*
 * M3508肩关节前馈：Iff = K1*cos(q2) + K2*cos(q2+q3) + bias。
 * K1补偿大臂及安装在大臂上的质量，K2预留给小臂/腕部重量传到肩关节的
 * 力矩；当前K2为0，因此没有启用小臂负载补偿，也没有给M2006增加前馈。
 * q2<=30度关闭，30度后平滑介入，接近90度平滑退出，q2>=90度关闭。
 * 这样只在用户确认的30~90度区间辅助，并避免开关边界出现电流阶跃。
 */
Arm_Shoulder_Feedforward_s g_arm_shoulder_feedforward = {
    .enabled = 1u,
    .active = 0u,
    .shoulder_gain_current = -2000.0f,
    .link_load_gain_current = 0.0f,
    .bias_current = 0.0f,
    .max_current = 3000.0f,
    .start_above_deg = 30.0f,
    .off_above_deg = 90.0f,
    .q2_deg = 0.0f,
    .q2_plus_q3_deg = 0.0f,
    .angle_window_scale = 0.0f,
    .cos_q2 = 0.0f,
    .cos_q2_plus_q3 = 0.0f,
    .shoulder_term_current = 0.0f,
    .link_load_term_current = 0.0f,
    .raw_current = 0.0f,
    .output_current = 0.0f,
};

/*
 * 堵转初始化专用速度PID。
 * 当前数值复制自拆分前的3508/2006注册速度环，后续调节正常三环时不会影响
 * 初始化碰限位力度；若只想改变堵转手感，应只修改这里。
 */
static PID_Init_Config_s arm_shoulder_homing_speed_pid = {
    .Kp = 3.0f,
    .Ki = 0.2f,
    .Kd = 0.0f,
    .Improve = PID_Integral_Limit,
    .IntegralLimit = 3000.0f,
    .MaxOut = 5500.0f,
};

static PID_Init_Config_s arm_elbow_homing_speed_pid = {
    .Kp = 2.3f,
    .Ki = 0.1f,
    .Kd = 0.0f,
    .Improve = PID_Integral_Limit,
    .IntegralLimit = 5000.0f,
    .MaxOut = 8200.0f,
};

static void ArmCalibrationFail(Arm_Calibration_State_e error_state);
static void ArmSoftLimitEnterState(Arm_Soft_Limit_State_e state,
                                   Arm_Soft_Limit_Axis_e axis,
                                   uint32_t now);
static void ArmUpdateSoftLimitTargets(void);
static void ArmBaseInitHold(void);
static void ArmSetAngleLoop(DJIMotor_Instance *motor);
static float ArmSoftLimitJointToMotor(float joint_deg,
                                      float reference_joint_deg,
                                      float scale);
static float ArmNearestBaseTotalTarget(float target_raw_deg);
static float ArmWrapTo360(float angle_deg);
static void ArmUpdateTeachPoint(void);
static void ArmRestoreNormalSpeedPids(void);
static void ArmUpdateShoulderGravityFeedforward(void);

Arm_State_s g_arm_state;
Arm_Calibration_s g_arm_calibration;
Arm_Soft_Limit_Debug_s g_arm_soft_limit_debug;
Arm_Kinematics_Debug_s g_arm_kinematics_debug;
Arm_Teach_Point_s g_arm_teach_point;

/* 下列别名只为缩短标定状态机代码，实际调参统一在 arm_config.h。 */
static const float g_arm_shoulder_homing_speed_dps =
    ARM_SHOULDER_HOMING_SPEED_DPS;
static const float g_arm_shoulder_stall_current =
    ARM_SHOULDER_STALL_CURRENT;
static const float g_arm_elbow_homing_speed_dps =
    ARM_ELBOW_HOMING_SPEED_DPS;
static const float g_arm_elbow_stall_current = ARM_ELBOW_STALL_CURRENT;
static const float g_arm_homing_stall_speed_dps =
    ARM_HOMING_STALL_SPEED_DPS;
static const uint32_t g_arm_homing_spinup_ms = ARM_HOMING_SPINUP_MS;
static const uint32_t g_arm_homing_stall_confirm_ms =
    ARM_HOMING_STALL_CONFIRM_MS;
static const float g_arm_shoulder_motor_to_joint_ratio =
    ARM_SHOULDER_EFFECTIVE_RATIO;
static const float g_arm_elbow_motor_to_joint_ratio =
    ARM_ELBOW_EFFECTIVE_RATIO;
static const float g_arm_soft_limit_margin_deg = ARM_SOFT_LIMIT_MARGIN_DEG;
static const float g_arm_release_joint_deg = ARM_RELEASE_JOINT_DEG;
static const uint32_t g_arm_release_min_ms = ARM_RELEASE_MIN_MS;
static const uint32_t g_arm_shoulder_stage_timeout_ms =
    ARM_SHOULDER_STAGE_TIMEOUT_MS;
static const uint32_t g_arm_elbow_stage_timeout_ms =
    ARM_ELBOW_STAGE_TIMEOUT_MS;
static const float g_arm_shoulder_max_joint_travel_deg =
    ARM_SHOULDER_MAX_TRAVEL_DEG;
static const float g_arm_elbow_max_joint_travel_deg =
    ARM_ELBOW_MAX_TRAVEL_DEG;

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

static float ArmSign(float value)
{
    return value >= 0.0f ? 1.0f : -1.0f;
}

static Motor_Init_Config_s ArmBaseMotorConfig(void)
{
    /*
     * GM6020 底座：角度环输出速度目标，速度环输出电流目标，电流环输出CAN值。
     * 调参顺序建议从内到外：电流环 -> 速度环 -> 角度环。
     * MaxOut 是每层最大输出；增大可提高响应，但过大容易超调和振动。
     */
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = ARM_BASE_MOTOR_CAN_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10.8f,
                .Ki = 2.8f,
                .DeadBand = 1.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 3500.0f,
                .MaxOut = 3000.0f,
            },
            .speed_PID = {
                .Kp = 10.0f,
                .Ki = 1.0f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 3000.0f,
                .MaxOut = 18000.0f,
            },
            .current_PID = {
                .Kp = 1.4f,
                .Ki = 0.01f,
                .Kd = 0.0f,
                .Improve = (PID_Improvement_e)(
                    PID_Trapezoid_Intergral | PID_Integral_Limit),
                .IntegralLimit = 3000.0f,
                .MaxOut = 30000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type =
                (Closeloop_Type_e)(ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP),
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
    /*
     * M3508 大臂：注册时保留同一套角度/速度/电流环参数。
     * 堵转寻零阶段临时切为速度环；初始化结束后恢复三级角度闭环。
     * 角度PID的输入是电机侧total_angle，因此DeadBand也为电机侧角度。
     */
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = ARM_SHOULDER_MOTOR_CAN_ID,
        },
        .controller_param_init_config = {
            .current_feedforward_ptr =
                &g_arm_shoulder_feedforward.output_current,
            .angle_PID = {
                .Kp = 10.0f,
                .Ki = 0.01f,
                .Kd = 0.0f,
                .DeadBand = 19.22925f,
                .MaxOut = 5800.0f,
            },
            .speed_PID = {
                .Kp = 7.5f,
                .Ki = 0.2f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 3000.0f,
                .MaxOut = 7500.0f,
            },
            .current_PID = {
                .Kp = 1.2f,
                .Ki = 0.01f,
                .Kd = 0.0f,
                .Improve = (PID_Improvement_e)(
                    PID_Trapezoid_Intergral | PID_Integral_Limit),
                .IntegralLimit = 3000.0f,
                .MaxOut = 16000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .feedforward_flag = CURRENT_FEEDFORWARD,
        },
        .motor_type = M3508,
    };
    return config;
}

static Motor_Init_Config_s ArmElbowMotorConfig(void)
{
    /*
     * M2006 小臂：与3508使用相同的三级闭环结构，参数独立调节。
     * 若回正无力，先观察速度环/电流环是否触及MaxOut，再决定提高限幅或Kp；
     * 若出现抖动和超调，优先降低角度Kp/积分或轨迹关节速度。
     */
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = ARM_ELBOW_MOTOR_CAN_ID,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 5.5f,
                .Ki = 0.5f,
                .Kd = 0.0f,
                .DeadBand = 100.0f,
                .MaxOut = 2800.0f,
            },
            .speed_PID = {
                .Kp = 2.3f,
                .Ki = 0.1f,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000.0f,
                .MaxOut = 8200.0f,
            },
            .current_PID = {
                .Kp = 1.2f,
                .Ki = 0.01f,
                .Kd = 0.0f,
                .Improve = (PID_Improvement_e)(
                    PID_Trapezoid_Intergral | PID_Integral_Limit),
                .IntegralLimit = 5000.0f,
                .MaxOut = 20000.0f,
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

static void ArmCopyPidConfig(const PID_Instance *pid,
                             PID_Init_Config_s *config)
{
    if (pid == NULL || config == NULL) {
        return;
    }
    config->Kp = pid->Kp;
    config->Ki = pid->Ki;
    config->Kd = pid->Kd;
    config->MaxOut = pid->MaxOut;
    config->DeadBand = pid->DeadBand;
    config->Improve = pid->Improve;
    config->IntegralLimit = pid->IntegralLimit;
    config->CoefA = pid->CoefA;
    config->CoefB = pid->CoefB;
    config->Output_LPF_RC = pid->Output_LPF_RC;
    config->Derivative_LPF_RC = pid->Derivative_LPF_RC;
}

/*
 * 进入堵转初始化时保存正常三环中的速度PID，然后只替换速度环参数。
 * 角度环和电流环不参与寻零，且其参数完全不受此切换影响。
 */
static void ArmLoadHomingSpeedPids(void)
{
    if (arm_homing_speed_pid_loaded || arm_shoulder_motor == NULL ||
        arm_elbow_motor == NULL) {
        return;
    }
    ArmCopyPidConfig(&arm_shoulder_motor->motor_controller.speed_PID,
                     &arm_shoulder_normal_speed_pid);
    ArmCopyPidConfig(&arm_elbow_motor->motor_controller.speed_PID,
                     &arm_elbow_normal_speed_pid);
    PIDInit(&arm_shoulder_motor->motor_controller.speed_PID,
            &arm_shoulder_homing_speed_pid);
    PIDInit(&arm_elbow_motor->motor_controller.speed_PID,
            &arm_elbow_homing_speed_pid);
    arm_homing_speed_pid_loaded = 1u;
}

/* 成功、失败和人工中止都必须恢复注册结构体中的正常速度PID。 */
static void ArmRestoreNormalSpeedPids(void)
{
    if (!arm_homing_speed_pid_loaded || arm_shoulder_motor == NULL ||
        arm_elbow_motor == NULL) {
        return;
    }
    PIDInit(&arm_shoulder_motor->motor_controller.speed_PID,
            &arm_shoulder_normal_speed_pid);
    PIDInit(&arm_elbow_motor->motor_controller.speed_PID,
            &arm_elbow_normal_speed_pid);
    arm_homing_speed_pid_loaded = 0u;
}

static void ArmStopMotor(DJIMotor_Instance *motor)
{
    if (motor == NULL) {
        return;
    }
    DJIMotorSetRef(motor, 0.0f);
    DJIMotorStop(motor);
}

static void ArmSetSpeedLoop(DJIMotor_Instance *motor)
{
    if (motor == NULL) {
        return;
    }
    motor->motor_settings.outer_loop_type = SPEED_LOOP;
    motor->motor_settings.close_loop_type = SPEED_LOOP;
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
    g_arm_state.base_calibrated = g_arm_calibration.base_calibrated;
    g_arm_state.joint_calibrated = g_arm_calibration.joint_calibrated;
    g_arm_state.kinematics_valid = g_arm_calibration.calibration_valid;
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
            g_arm_state.motor_enabled[i] =
                arm_motors[i]->stop_flag == MOTOR_ENALBED;
        } else {
            g_arm_state.motor_total_angle_deg[i] = 0.0f;
            g_arm_state.motor_current[i] = 0.0f;
            g_arm_state.motor_speed_dps[i] = 0.0f;
            g_arm_state.motor_online[i] = 0u;
            g_arm_state.motor_enabled[i] = 0u;
        }
    }

    g_arm_state.base_raw_deg = arm_base_motor != NULL ?
        arm_base_motor->measure.angle_single_round : 0.0f;
    g_arm_state.shoulder_deg_per_motor_deg =
        g_arm_calibration.shoulder_deg_per_motor_deg;
    g_arm_state.elbow_deg_per_motor_deg =
        g_arm_calibration.elbow_deg_per_motor_deg;
    g_arm_state.base_calibrated = g_arm_calibration.base_calibrated;
    g_arm_state.joint_calibrated = g_arm_calibration.joint_calibrated;
    g_arm_state.kinematics_valid = g_arm_calibration.calibration_valid;
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

/*
 * 每个控制周期根据当前q2更新M3508电流前馈。
 * 仅在正常初始化完成、反馈在线且电机已使能时生效；堵转寻零、打点、
 * 维护扫描、失能及急停状态统一输出0，避免前馈干扰初始化或手动拖动。
 */
static void ArmUpdateShoulderGravityFeedforward(void)
{
    float q2_deg = g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER];
    float q3_deg = g_arm_state.q_feedback_deg[ARM_JOINT_ELBOW];
    float limit = fabsf(g_arm_shoulder_feedforward.max_current);
    float start_deg = g_arm_shoulder_feedforward.start_above_deg;
    float off_deg = g_arm_shoulder_feedforward.off_above_deg;
    float window_scale = 0.0f;
    uint8_t can_apply =
        g_arm_shoulder_feedforward.enabled != 0u &&
        arm_shoulder_motor != NULL &&
        g_arm_state.motor_online[ARM_JOINT_SHOULDER] != 0u &&
        g_arm_state.motor_enabled[ARM_JOINT_SHOULDER] != 0u &&
        g_arm_calibration.joint_calibrated != 0u &&
        g_arm_state.calibration_state == ARM_CAL_VALID &&
        g_arm_state.soft_limit_state == ARM_SOFT_LIMIT_COMPLETE &&
        q2_deg >= g_arm_calibration.shoulder_soft_min_deg &&
        q2_deg <= g_arm_calibration.shoulder_soft_max_deg &&
        arm_homing_speed_pid_loaded == 0u &&
        g_arm_homing_abort == 0u &&
        ARM_BOOT_MODE != ARM_BOOT_MODE_TEACH_POINT &&
        ARM_BOOT_MODE != ARM_BOOT_MODE_FULL_CALIBRATION &&
        ARM_BOOT_MODE != ARM_BOOT_MODE_SHOULDER_RATIO_TEST;

    g_arm_shoulder_feedforward.q2_deg = q2_deg;
    g_arm_shoulder_feedforward.q2_plus_q3_deg = q2_deg + q3_deg;
    if (!isfinite(q2_deg) || !isfinite(q3_deg)) {
        g_arm_shoulder_feedforward.angle_window_scale = 0.0f;
        g_arm_shoulder_feedforward.cos_q2 = 0.0f;
        g_arm_shoulder_feedforward.cos_q2_plus_q3 = 0.0f;
        g_arm_shoulder_feedforward.shoulder_term_current = 0.0f;
        g_arm_shoulder_feedforward.link_load_term_current = 0.0f;
        g_arm_shoulder_feedforward.raw_current = 0.0f;
        g_arm_shoulder_feedforward.output_current = 0.0f;
        g_arm_shoulder_feedforward.active = 0u;
        return;
    }

    if (isfinite(start_deg) && isfinite(off_deg) &&
        off_deg > start_deg + 15.0f && q2_deg > start_deg &&
        q2_deg < off_deg) {
        float ramp_in = ArmClampFloat((q2_deg - start_deg) / 5.0f,
                                      0.0f, 1.0f);
        float ramp_out = ArmClampFloat((off_deg - q2_deg) / 10.0f,
                                       0.0f, 1.0f);

        /* 30~35度渐入，35~80度完整，80~90度渐出；两端斜率均为0。 */
        ramp_in = ramp_in * ramp_in * (3.0f - 2.0f * ramp_in);
        ramp_out = ramp_out * ramp_out * (3.0f - 2.0f * ramp_out);
        window_scale = fminf(ramp_in, ramp_out);
    }
    g_arm_shoulder_feedforward.angle_window_scale = window_scale;

    g_arm_shoulder_feedforward.cos_q2 = cosf(q2_deg * ARM_DEG_TO_RAD);
    g_arm_shoulder_feedforward.cos_q2_plus_q3 = cosf(
        g_arm_shoulder_feedforward.q2_plus_q3_deg * ARM_DEG_TO_RAD);
    g_arm_shoulder_feedforward.shoulder_term_current =
        g_arm_shoulder_feedforward.shoulder_gain_current *
        g_arm_shoulder_feedforward.cos_q2;
    g_arm_shoulder_feedforward.link_load_term_current =
        g_arm_shoulder_feedforward.link_load_gain_current *
        g_arm_shoulder_feedforward.cos_q2_plus_q3;
    g_arm_shoulder_feedforward.raw_current =
        (g_arm_shoulder_feedforward.shoulder_term_current +
         g_arm_shoulder_feedforward.link_load_term_current +
         g_arm_shoulder_feedforward.bias_current) * window_scale;

    if (!can_apply || window_scale <= ARM_FLOAT_EPSILON ||
        !isfinite(g_arm_shoulder_feedforward.raw_current) ||
        limit <= ARM_FLOAT_EPSILON) {
        g_arm_shoulder_feedforward.output_current = 0.0f;
        g_arm_shoulder_feedforward.active = 0u;
        return;
    }

    g_arm_shoulder_feedforward.output_current = ArmClampFloat(
        g_arm_shoulder_feedforward.raw_current, -limit, limit);
    g_arm_shoulder_feedforward.active = 1u;
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
    g_arm_state.small_link_pitch_deg = g_arm_state.end_pitch_deg;
}

/*
 * 打点模式唯一Watch快照。三台电机失能后仍持续接收CAN反馈，因此用手拖动
 * 机械臂时，q1/q2/q3和腕部轴心FK坐标会实时变化。q4尚未接入，固定为0。
 */
static void ArmUpdateTeachPoint(void)
{
    const Arm_Wrist_State_s *wrist_state = ArmWristGetState();

    g_arm_teach_point.ready = arm_teach_mode_entered &&
        !g_arm_state.motor_enabled[0] &&
        !g_arm_state.motor_enabled[1] &&
        !g_arm_state.motor_enabled[2];
    g_arm_teach_point.point_type = ARM_CONTROL_POINT_WRIST_CENTER;
    g_arm_teach_point.kinematics_valid = g_arm_state.kinematics_valid;
    memcpy(g_arm_teach_point.motor_online, g_arm_state.motor_online,
           sizeof(g_arm_teach_point.motor_online));
    memcpy(g_arm_teach_point.motor_enabled, g_arm_state.motor_enabled,
           sizeof(g_arm_teach_point.motor_enabled));
    memcpy(g_arm_teach_point.q_deg, g_arm_state.q_feedback_deg,
           sizeof(g_arm_teach_point.q_deg));
    g_arm_teach_point.q_deg[ARM_JOINT_WRIST] = 0.0f;
    g_arm_teach_point.base_raw_deg = g_arm_state.base_raw_deg;
    memcpy(g_arm_teach_point.motor_total_angle_deg,
           g_arm_state.motor_total_angle_deg,
           sizeof(g_arm_teach_point.motor_total_angle_deg));
    g_arm_teach_point.wrist_center_mm = g_arm_state.wrist_center;
    g_arm_teach_point.small_link_pitch_deg =
        g_arm_state.small_link_pitch_deg;
    g_arm_teach_point.wrist_pwm_us = wrist_state->pulse_us;
    g_arm_teach_point.wrist_configured = wrist_state->configured;
    g_arm_teach_point.tool_model_valid =
        ARM_TOOL_MODEL_ENABLE != 0u && wrist_state->configured;
    if (arm_teach_mode_entered) {
        g_arm_teach_point.update_count++;
    }
}

static void ArmUpdateKinematicsDebug(void)
{
    g_arm_kinematics_debug.kinematics_valid =
        g_arm_state.kinematics_valid;
    g_arm_kinematics_debug.base_calibrated =
        g_arm_state.base_calibrated;
    g_arm_kinematics_debug.joint_calibrated =
        g_arm_state.joint_calibrated;
    memcpy(g_arm_kinematics_debug.motor_online,
           g_arm_state.motor_online,
           sizeof(g_arm_kinematics_debug.motor_online));
    memcpy(g_arm_kinematics_debug.motor_enabled,
           g_arm_state.motor_enabled,
           sizeof(g_arm_kinematics_debug.motor_enabled));
    g_arm_kinematics_debug.base_raw_deg = g_arm_state.base_raw_deg;
    memcpy(g_arm_kinematics_debug.q_feedback_deg,
           g_arm_state.q_feedback_deg,
           sizeof(g_arm_kinematics_debug.q_feedback_deg));
    memcpy(g_arm_kinematics_debug.q_target_deg,
           g_arm_state.q_target_deg,
           sizeof(g_arm_kinematics_debug.q_target_deg));
    g_arm_kinematics_debug.wrist_center_mm = g_arm_state.wrist_center;
    g_arm_kinematics_debug.horizontal_radius_mm = sqrtf(
        g_arm_state.wrist_center.x_mm * g_arm_state.wrist_center.x_mm +
        g_arm_state.wrist_center.y_mm * g_arm_state.wrist_center.y_mm);
    g_arm_kinematics_debug.planar_reach_from_shoulder_mm =
        ARM_LINK_1_MM * cosf(
            g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER] *
            ARM_DEG_TO_RAD) +
        ARM_LINK_2_MM * cosf(
            (g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER] +
             g_arm_state.q_feedback_deg[ARM_JOINT_ELBOW]) *
            ARM_DEG_TO_RAD);
    g_arm_kinematics_debug.wrist_height_from_shoulder_mm =
        g_arm_state.wrist_center.z_mm - ARM_BASE_HEIGHT_MM;
    g_arm_kinematics_debug.small_link_pitch_deg =
        g_arm_state.small_link_pitch_deg;
    g_arm_kinematics_debug.base_height_mm = ARM_BASE_HEIGHT_MM;
    g_arm_kinematics_debug.link_1_mm = ARM_LINK_1_MM;
    g_arm_kinematics_debug.link_2_mm = ARM_LINK_2_MM;
    g_arm_kinematics_debug.shoulder_offset_forward_mm =
        ARM_SHOULDER_OFFSET_FORWARD_MM;
    g_arm_kinematics_debug.shoulder_offset_left_mm =
        ARM_SHOULDER_OFFSET_LEFT_MM;
    g_arm_kinematics_debug.shoulder_soft_limit_deg[0] =
        g_arm_calibration.shoulder_soft_min_deg;
    g_arm_kinematics_debug.shoulder_soft_limit_deg[1] =
        g_arm_calibration.shoulder_soft_max_deg;
    g_arm_kinematics_debug.elbow_soft_limit_deg[0] =
        g_arm_calibration.elbow_soft_min_deg;
    g_arm_kinematics_debug.elbow_soft_limit_deg[1] =
        g_arm_calibration.elbow_soft_max_deg;
    g_arm_kinematics_debug.reference_q_deg[0] = 0.0f;
    g_arm_kinematics_debug.reference_q_deg[1] =
        ARM_SHOULDER_REFERENCE_DEG;
    g_arm_kinematics_debug.reference_q_deg[2] = ARM_ELBOW_REFERENCE_DEG;
    ArmForwardKinematics3DOF(
        g_arm_kinematics_debug.reference_q_deg[0],
        g_arm_kinematics_debug.reference_q_deg[1],
        g_arm_kinematics_debug.reference_q_deg[2],
        &g_arm_kinematics_debug.reference_wrist_center_mm);
    g_arm_kinematics_debug.reference_horizontal_radius_mm = sqrtf(
        g_arm_kinematics_debug.reference_wrist_center_mm.x_mm *
            g_arm_kinematics_debug.reference_wrist_center_mm.x_mm +
        g_arm_kinematics_debug.reference_wrist_center_mm.y_mm *
            g_arm_kinematics_debug.reference_wrist_center_mm.y_mm);
    g_arm_kinematics_debug.reference_height_from_shoulder_mm =
        g_arm_kinematics_debug.reference_wrist_center_mm.z_mm -
        ARM_BASE_HEIGHT_MM;
    g_arm_kinematics_debug.reference_planar_reach_from_shoulder_mm =
        ARM_LINK_1_MM * cosf(ARM_SHOULDER_REFERENCE_DEG *
                            ARM_DEG_TO_RAD) +
        ARM_LINK_2_MM * cosf(
            (ARM_SHOULDER_REFERENCE_DEG + ARM_ELBOW_REFERENCE_DEG) *
            ARM_DEG_TO_RAD);
}

static uint8_t ArmCalibrationStateIsActive(Arm_Calibration_State_e state)
{
    return state >= ARM_CAL_ELBOW_FIND_REFERENCE &&
           state <= ARM_CAL_SHOULDER_DONE;
}

static void ArmLoadMeasuredJointMapping(void)
{
    g_arm_calibration.shoulder_hard_min_deg =
        fminf(ARM_SHOULDER_REFERENCE_DEG, ARM_SHOULDER_OPPOSITE_DEG);
    g_arm_calibration.shoulder_hard_max_deg =
        fmaxf(ARM_SHOULDER_REFERENCE_DEG, ARM_SHOULDER_OPPOSITE_DEG);
    g_arm_calibration.elbow_hard_min_deg =
        fminf(ARM_ELBOW_REFERENCE_DEG, ARM_ELBOW_OPPOSITE_DEG);
    g_arm_calibration.elbow_hard_max_deg =
        fmaxf(ARM_ELBOW_REFERENCE_DEG, ARM_ELBOW_OPPOSITE_DEG);
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
    if (!g_arm_calibration.shoulder_reference_found ||
        !g_arm_calibration.elbow_reference_found ||
        (g_arm_state.homing_mode == ARM_HOMING_FULL_SCAN &&
         (!g_arm_calibration.shoulder_opposite_found ||
          !g_arm_calibration.elbow_opposite_found ||
          !ArmUpdateMappingFromFullScan()))) {
        ArmCalibrationFail(ARM_CAL_ERROR_TRAVEL);
        return;
    }
    ArmRestoreNormalSpeedPids();
    arm_joint_hold_target_motor_deg[0] =
        arm_shoulder_motor->measure.total_angle;
    arm_joint_hold_target_motor_deg[1] =
        arm_elbow_motor->measure.total_angle;
    ArmClearMotorController(arm_shoulder_motor);
    ArmClearMotorController(arm_elbow_motor);
    if (ARM_BOOT_MODE == ARM_BOOT_MODE_TEACH_POINT ||
        ARM_BOOT_MODE == ARM_BOOT_MODE_FULL_CALIBRATION) {
        /* 打点和维护扫描完成后立即保持零输出，不出现短暂重新使能。 */
        ArmStopMotor(arm_shoulder_motor);
        ArmStopMotor(arm_elbow_motor);
    } else {
        ArmSetAngleLoop(arm_shoulder_motor);
        ArmSetAngleLoop(arm_elbow_motor);
        DJIMotorSetRef(arm_shoulder_motor,
                       arm_joint_hold_target_motor_deg[0]);
        DJIMotorSetRef(arm_elbow_motor,
                       arm_joint_hold_target_motor_deg[1]);
        DJIMotorEnable(arm_shoulder_motor);
        DJIMotorEnable(arm_elbow_motor);
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
    Arm_Soft_Limit_State_e init_error_state = ARM_SOFT_LIMIT_ERROR_LIMIT;

    ArmStopMotor(arm_base_motor);
    ArmStopMotor(arm_shoulder_motor);
    ArmStopMotor(arm_elbow_motor);
    ArmRestoreNormalSpeedPids();
    ArmClearMotorController(arm_base_motor);
    ArmClearMotorController(arm_shoulder_motor);
    ArmClearMotorController(arm_elbow_motor);
    g_arm_calibration.joint_calibrated = 0u;
    ArmUpdateCalibrationValid();
    g_arm_state.calibration_stall_condition = 0u;
    g_arm_state.calibration_state = error_state;
    g_arm_state.mode = ARM_MODE_SAFE;
    if (error_state == ARM_CAL_ERROR_OFFLINE) {
        init_error_state = ARM_SOFT_LIMIT_ERROR_OFFLINE;
    } else if (error_state == ARM_CAL_ERROR_TIMEOUT) {
        init_error_state = ARM_SOFT_LIMIT_ERROR_TIMEOUT;
    } else if (error_state == ARM_CAL_ERROR_ABORT) {
        init_error_state = ARM_SOFT_LIMIT_ABORTED;
    }
    ArmSoftLimitEnterState(init_error_state,
                           g_arm_state.soft_limit_axis,
                           HAL_GetTick());
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

    /* Remove torque immediately; zeroing waits for the configured settle time. */
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
    if (g_arm_state.calibration_stage_elapsed_ms < ARM_STOP_SETTLE_MS) {
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

static void ArmRecordOppositeSpan(DJIMotor_Instance *motor,
                                  Arm_Calibration_Motor_e channel,
                                  float ratio,
                                  Arm_Calibration_State_e done_state,
                                  uint32_t now)
{
    float motor_span;

    ArmStopMotor(motor);
    motor_span = motor->measure.total_angle;
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

static void ArmRunFindOpposite(DJIMotor_Instance *motor,
                               DJIMotor_Instance *inactive_motor,
                               Arm_Calibration_Motor_e channel,
                               float speed_dps,
                               float reference_direction,
                               float current_threshold,
                               float ratio,
                               float max_joint_travel_deg,
                               uint32_t timeout_ms,
                               uint32_t now)
{
    float max_motor_travel;

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

    ArmStopMotor(motor);
    ArmClearMotorController(motor);
    /* Record both motors from their unloaded position after settling. */
    ArmEnterCalibrationState(
        channel == ARM_CAL_MOTOR_SHOULDER ?
            ARM_CAL_SHOULDER_SETTLE_OPPOSITE :
            ARM_CAL_ELBOW_SETTLE_OPPOSITE,
        motor, now);
}

static void ArmRunSettleOpposite(DJIMotor_Instance *motor,
                                 DJIMotor_Instance *inactive_motor,
                                 Arm_Calibration_Motor_e channel,
                                 float ratio,
                                 Arm_Calibration_State_e done_state,
                                 uint32_t now)
{
    ArmStopMotor(motor);
    ArmStopMotor(inactive_motor);
    g_arm_state.calibration_stage_elapsed_ms =
        (uint32_t)(now - arm_cal_runtime.stage_start_tick);
    if (g_arm_state.calibration_stage_elapsed_ms < ARM_STOP_SETTLE_MS) {
        return;
    }

    ArmRecordOppositeSpan(motor, channel, ratio, done_state, now);
}

static void ArmStartHomingMode(Arm_Homing_Mode_e mode)
{
    uint32_t now = HAL_GetTick();

    arm_auto_calibration_attempted = 1u;
    ArmStopMotor(arm_shoulder_motor);
    ArmStopMotor(arm_elbow_motor);
    if (g_arm_state.soft_limit_state == ARM_SOFT_LIMIT_WAIT_CALIBRATION ||
        g_arm_state.soft_limit_state == ARM_SOFT_LIMIT_COMPLETE) {
        ArmBaseInitHold();
    } else {
        ArmStopMotor(arm_base_motor);
    }
    ArmSetSpeedLoop(arm_shoulder_motor);
    ArmSetSpeedLoop(arm_elbow_motor);
    g_arm_state.soft_limit_state = ARM_SOFT_LIMIT_WAIT_CALIBRATION;
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
    ArmLoadHomingSpeedPids();
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
        if (state != ARM_CAL_VALID) {
            ArmStopMotor(arm_shoulder_motor);
            ArmStopMotor(arm_elbow_motor);
        }
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
                               g_arm_elbow_stage_timeout_ms, now);
            break;
        case ARM_CAL_ELBOW_SETTLE_OPPOSITE:
            ArmRunSettleOpposite(arm_elbow_motor, arm_shoulder_motor,
                                 ARM_CAL_MOTOR_ELBOW,
                                 g_arm_elbow_motor_to_joint_ratio,
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
                               g_arm_shoulder_stage_timeout_ms, now);
            break;
        case ARM_CAL_SHOULDER_SETTLE_OPPOSITE:
            ArmRunSettleOpposite(arm_shoulder_motor, arm_elbow_motor,
                                 ARM_CAL_MOTOR_SHOULDER,
                                 g_arm_shoulder_motor_to_joint_ratio,
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

uint8_t ArmSetJointTargetDeg(float q1_deg, float q2_deg, float q3_deg)
{
    float target_q_deg[3];

    target_q_deg[0] = ArmClampFloat(q1_deg, -180.0f, 180.0f);
    target_q_deg[1] = ArmClampFloat(
        q2_deg, g_arm_calibration.shoulder_soft_min_deg,
        g_arm_calibration.shoulder_soft_max_deg);
    target_q_deg[2] = ArmClampFloat(
        q3_deg, g_arm_calibration.elbow_soft_min_deg,
        g_arm_calibration.elbow_soft_max_deg);
    if (!ArmBeginJointMove(target_q_deg)) {
        return 0u;
    }
    return ArmUpdateJointReference(target_q_deg);
}

uint8_t ArmUpdateJointReference(const float reference_q_deg[3])
{
    float base_target_raw_deg;
    float base_target_motor_deg;
    float shoulder_target_motor_deg;
    float elbow_target_motor_deg;

    if (reference_q_deg == NULL || !g_arm_calibration.calibration_valid ||
        !ArmMotorFeedbackReady(arm_base_motor) ||
        !ArmMotorFeedbackReady(arm_shoulder_motor) ||
        !ArmMotorFeedbackReady(arm_elbow_motor) ||
        !isfinite(reference_q_deg[0]) ||
        !isfinite(reference_q_deg[1]) ||
        !isfinite(reference_q_deg[2]) ||
        reference_q_deg[0] < -180.0f || reference_q_deg[0] > 180.0f ||
        reference_q_deg[1] < 0.0f || reference_q_deg[1] > 180.0f ||
        reference_q_deg[2] < -180.0f || reference_q_deg[2] > -85.0f) {
        return 0u;
    }
    base_target_raw_deg = ArmWrapTo360(
        ARM_BASE_FRONT_RAW_DEG +
        ArmSign(arm_base_direction) * reference_q_deg[0]);
    base_target_motor_deg = ArmNearestBaseTotalTarget(base_target_raw_deg);
    shoulder_target_motor_deg = ArmSoftLimitJointToMotor(
        reference_q_deg[1], ARM_SHOULDER_REFERENCE_DEG,
        g_arm_calibration.shoulder_deg_per_motor_deg);
    elbow_target_motor_deg = ArmSoftLimitJointToMotor(
        reference_q_deg[2], ARM_ELBOW_REFERENCE_DEG,
        g_arm_calibration.elbow_deg_per_motor_deg);
    if (!isfinite(base_target_motor_deg) ||
        !isfinite(shoulder_target_motor_deg) ||
        !isfinite(elbow_target_motor_deg)) {
        return 0u;
    }

    g_arm_state.q_target_deg[ARM_JOINT_BASE_YAW] = reference_q_deg[0];
    g_arm_state.q_target_deg[ARM_JOINT_SHOULDER] = reference_q_deg[1];
    g_arm_state.q_target_deg[ARM_JOINT_ELBOW] = reference_q_deg[2];
    g_arm_state.base_init_target_raw_deg = base_target_raw_deg;
    g_arm_state.base_init_target_motor_deg = base_target_motor_deg;
    arm_joint_hold_target_motor_deg[0] = shoulder_target_motor_deg;
    arm_joint_hold_target_motor_deg[1] = elbow_target_motor_deg;

    DJIMotorSetRef(arm_base_motor, base_target_motor_deg);
    DJIMotorSetRef(arm_shoulder_motor, shoulder_target_motor_deg);
    DJIMotorSetRef(arm_elbow_motor, elbow_target_motor_deg);
    return 1u;
}

uint8_t ArmBeginJointMove(const float target_q_deg[3])
{
    float current_q_deg[3];

    if (target_q_deg == NULL || !g_arm_calibration.calibration_valid ||
        !ArmMotorFeedbackReady(arm_base_motor) ||
        !ArmMotorFeedbackReady(arm_shoulder_motor) ||
        !ArmMotorFeedbackReady(arm_elbow_motor) ||
        !isfinite(target_q_deg[0]) || !isfinite(target_q_deg[1]) ||
        !isfinite(target_q_deg[2]) || target_q_deg[0] < -180.0f ||
        target_q_deg[0] > 180.0f || target_q_deg[1] < 0.0f ||
        target_q_deg[1] > 180.0f || target_q_deg[2] < -180.0f ||
        target_q_deg[2] > -85.0f) {
        return 0u;
    }
    current_q_deg[0] = g_arm_state.q_feedback_deg[ARM_JOINT_BASE_YAW];
    current_q_deg[1] = g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER];
    current_q_deg[2] = g_arm_state.q_feedback_deg[ARM_JOINT_ELBOW];
    ArmClearMotorController(arm_base_motor);
    ArmClearMotorController(arm_shoulder_motor);
    ArmClearMotorController(arm_elbow_motor);
    ArmSetAngleLoop(arm_base_motor);
    ArmSetAngleLoop(arm_shoulder_motor);
    ArmSetAngleLoop(arm_elbow_motor);
    if (!ArmUpdateJointReference(current_q_deg)) {
        return 0u;
    }
    g_arm_state.q_target_deg[ARM_JOINT_BASE_YAW] = target_q_deg[0];
    g_arm_state.q_target_deg[ARM_JOINT_SHOULDER] = target_q_deg[1];
    g_arm_state.q_target_deg[ARM_JOINT_ELBOW] = target_q_deg[2];
    return 1u;
}

void ArmMotionStopMotors(void)
{
    ArmStopMotor(arm_base_motor);
    ArmStopMotor(arm_shoulder_motor);
    ArmStopMotor(arm_elbow_motor);
    g_arm_state.mode = ARM_MODE_SAFE;
}

static void ArmAutoCalibrationTask(uint32_t now)
{
    if (arm_auto_calibration_attempted) {
        return;
    }
    if (g_arm_state.soft_limit_state == ARM_SOFT_LIMIT_WAIT_CALIBRATION &&
        g_arm_state.calibration_state == ARM_CAL_IDLE) {
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
        if ((uint32_t)(now - arm_auto_online_start_tick) <
            ARM_AUTO_START_ONLINE_MS) {
            return;
        }
        if (ARM_BOOT_MODE == ARM_BOOT_MODE_FULL_CALIBRATION) {
            ArmCalibrationStart();
        } else {
            ArmHomingStart();
        }
        return;
    }
    if (g_arm_state.soft_limit_state != ARM_SOFT_LIMIT_WAIT_ONLINE ||
        g_arm_state.calibration_state != ARM_CAL_IDLE) {
        return;
    }
    if (!ArmMotorFeedbackReady(arm_base_motor)) {
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
        ArmUpdateSoftLimitTargets();
        ArmClearMotorController(arm_base_motor);
        arm_base_last_target_error_deg =
            g_arm_state.base_init_target_motor_deg -
            arm_base_motor->measure.total_angle;
        arm_base_target_error_valid = 1u;
        arm_auto_online_waiting = 0u;
        ArmSoftLimitEnterState(ARM_SOFT_LIMIT_BASE_MOVING,
                               ARM_SOFT_LIMIT_AXIS_BASE, now);
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

static void ArmSoftLimitEnterState(Arm_Soft_Limit_State_e state,
                                   Arm_Soft_Limit_Axis_e axis,
                                   uint32_t now)
{
    g_arm_state.soft_limit_state = state;
    g_arm_state.soft_limit_axis = axis;
    g_arm_state.soft_limit_elapsed_ms = 0u;
    arm_soft_limit_state_tick = now;
    arm_soft_limit_overcurrent_tick = 0u;
}

static void ArmSoftLimitFail(Arm_Soft_Limit_State_e state, uint32_t now)
{
    Arm_Soft_Limit_Axis_e failed_axis = g_arm_state.soft_limit_axis;

    ArmStopMotor(arm_base_motor);
    ArmStopMotor(arm_shoulder_motor);
    ArmStopMotor(arm_elbow_motor);
    ArmClearMotorController(arm_base_motor);
    ArmClearMotorController(arm_shoulder_motor);
    ArmClearMotorController(arm_elbow_motor);
    ArmSoftLimitEnterState(state, failed_axis, now);
    g_arm_state.mode = ARM_MODE_SAFE;
}

static float ArmSoftLimitJointToMotor(float joint_deg,
                                      float reference_joint_deg,
                                      float scale)
{
    if (!isfinite(scale) || fabsf(scale) <= ARM_FLOAT_EPSILON) {
        return 0.0f;
    }
    return (joint_deg - reference_joint_deg) / scale;
}

static float ArmWrapTo360(float angle_deg)
{
    while (angle_deg >= 360.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float ArmNearestBaseTotalTarget(float target_raw_deg)
{
    float raw_delta_deg;
    float best_delta_deg;
    float candidate_delta_deg;

    if (arm_base_motor == NULL) {
        return 0.0f;
    }

    raw_delta_deg = target_raw_deg -
                    arm_base_motor->measure.angle_single_round;
    best_delta_deg = raw_delta_deg;
    candidate_delta_deg = raw_delta_deg - 360.0f;
    if (fabsf(candidate_delta_deg) < fabsf(best_delta_deg)) {
        best_delta_deg = candidate_delta_deg;
    }
    candidate_delta_deg = raw_delta_deg + 360.0f;
    if (fabsf(candidate_delta_deg) < fabsf(best_delta_deg)) {
        best_delta_deg = candidate_delta_deg;
    }

    return arm_base_motor->measure.total_angle + best_delta_deg;
}

static void ArmUpdateSoftLimitTargets(void)
{
    g_arm_state.soft_limit_target_joint_deg[0] =
        ARM_SHOULDER_REFERENCE_DEG - ARM_SOFT_LIMIT_MARGIN_DEG;
    g_arm_state.soft_limit_target_joint_deg[1] =
        ARM_ELBOW_REFERENCE_DEG + ARM_SOFT_LIMIT_MARGIN_DEG;
    g_arm_state.soft_limit_target_motor_deg[0] =
        ArmSoftLimitJointToMotor(
            g_arm_state.soft_limit_target_joint_deg[0],
            ARM_SHOULDER_REFERENCE_DEG,
            g_arm_calibration.shoulder_deg_per_motor_deg);
    g_arm_state.soft_limit_target_motor_deg[1] =
        ArmSoftLimitJointToMotor(
            g_arm_state.soft_limit_target_joint_deg[1],
            ARM_ELBOW_REFERENCE_DEG,
            g_arm_calibration.elbow_deg_per_motor_deg);
    g_arm_state.base_init_target_raw_deg = ARM_BASE_FRONT_RAW_DEG;
    if (arm_base_motor != NULL &&
        g_arm_state.soft_limit_state != ARM_SOFT_LIMIT_BASE_MOVING &&
        g_arm_state.soft_limit_state != ARM_SOFT_LIMIT_BASE_SETTLE &&
        g_arm_state.soft_limit_state != ARM_SOFT_LIMIT_WAIT_CALIBRATION &&
        g_arm_state.soft_limit_state != ARM_SOFT_LIMIT_COMPLETE) {
        g_arm_state.base_init_target_motor_deg =
            ArmNearestBaseTotalTarget(ARM_BASE_FRONT_RAW_DEG);
    }
}

static void ArmUpdateSoftLimitDebug(void)
{
    DJIMotor_Instance *active_motor = NULL;

    if (g_arm_state.soft_limit_axis == ARM_SOFT_LIMIT_AXIS_BASE) {
        active_motor = arm_base_motor;
    } else if (g_arm_state.soft_limit_axis == ARM_SOFT_LIMIT_AXIS_SHOULDER) {
        active_motor = arm_shoulder_motor;
    } else if (g_arm_state.soft_limit_axis == ARM_SOFT_LIMIT_AXIS_ELBOW) {
        active_motor = arm_elbow_motor;
    }
    g_arm_soft_limit_debug.arm_mode = g_arm_state.mode;
    g_arm_soft_limit_debug.calibration_state =
        g_arm_state.calibration_state;
    g_arm_soft_limit_debug.active_axis = g_arm_state.soft_limit_axis;
    g_arm_soft_limit_debug.state = g_arm_state.soft_limit_state;
    g_arm_soft_limit_debug.homing_abort = g_arm_homing_abort;
    g_arm_soft_limit_debug.base_calibrated = g_arm_state.base_calibrated;
    g_arm_soft_limit_debug.joint_calibrated = g_arm_state.joint_calibrated;
    g_arm_soft_limit_debug.kinematics_valid = g_arm_state.kinematics_valid;
    memcpy(g_arm_soft_limit_debug.motor_online, g_arm_state.motor_online,
           sizeof(g_arm_soft_limit_debug.motor_online));
    memcpy(g_arm_soft_limit_debug.motor_enabled, g_arm_state.motor_enabled,
           sizeof(g_arm_soft_limit_debug.motor_enabled));
    memcpy(g_arm_soft_limit_debug.q_feedback_deg,
           g_arm_state.q_feedback_deg,
           sizeof(g_arm_soft_limit_debug.q_feedback_deg));
    memcpy(g_arm_soft_limit_debug.motor_total_angle_deg,
           g_arm_state.motor_total_angle_deg,
           sizeof(g_arm_soft_limit_debug.motor_total_angle_deg));
    memcpy(g_arm_soft_limit_debug.motor_current, g_arm_state.motor_current,
           sizeof(g_arm_soft_limit_debug.motor_current));
    memcpy(g_arm_soft_limit_debug.motor_speed_dps,
           g_arm_state.motor_speed_dps,
           sizeof(g_arm_soft_limit_debug.motor_speed_dps));
    memcpy(g_arm_soft_limit_debug.target_joint_deg,
           g_arm_state.soft_limit_target_joint_deg,
           sizeof(g_arm_soft_limit_debug.target_joint_deg));
    memcpy(g_arm_soft_limit_debug.target_motor_deg,
           g_arm_state.soft_limit_target_motor_deg,
           sizeof(g_arm_soft_limit_debug.target_motor_deg));
    g_arm_soft_limit_debug.final_joint_deg[0] =
        g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER];
    g_arm_soft_limit_debug.final_joint_deg[1] =
        g_arm_state.q_feedback_deg[ARM_JOINT_ELBOW];
    g_arm_soft_limit_debug.final_motor_deg[0] =
        g_arm_state.motor_total_angle_deg[1];
    g_arm_soft_limit_debug.final_motor_deg[1] =
        g_arm_state.motor_total_angle_deg[2];
    g_arm_soft_limit_debug.base_target_raw_deg =
        g_arm_state.base_init_target_raw_deg;
    g_arm_soft_limit_debug.base_target_motor_deg =
        g_arm_state.base_init_target_motor_deg;
    g_arm_soft_limit_debug.base_angle_error_deg =
        ArmWrapTo180(g_arm_state.base_init_target_raw_deg -
                     g_arm_state.base_raw_deg);
    g_arm_soft_limit_debug.small_angle_test_active = 0u;
    g_arm_soft_limit_debug.command_target_joint_deg[0] =
        g_arm_state.q_target_deg[ARM_JOINT_BASE_YAW];
    g_arm_soft_limit_debug.command_target_joint_deg[1] =
        g_arm_state.q_target_deg[ARM_JOINT_SHOULDER];
    g_arm_soft_limit_debug.command_target_joint_deg[2] =
        g_arm_state.q_target_deg[ARM_JOINT_ELBOW];
    g_arm_soft_limit_debug.command_target_motor_deg[0] =
        g_arm_state.base_init_target_motor_deg;
    g_arm_soft_limit_debug.command_target_motor_deg[1] =
        arm_joint_hold_target_motor_deg[0];
    g_arm_soft_limit_debug.command_target_motor_deg[2] =
        arm_joint_hold_target_motor_deg[1];
    g_arm_soft_limit_debug.base_angle_pid_output_dps =
        arm_base_motor != NULL ?
            arm_base_motor->motor_controller.angle_PID.Output : 0.0f;
    g_arm_soft_limit_debug.base_speed_pid_output =
        arm_base_motor != NULL ?
            arm_base_motor->motor_controller.speed_PID.Output : 0.0f;
    g_arm_soft_limit_debug.angle_ref_deg[0] =
        arm_shoulder_motor != NULL ?
            arm_shoulder_motor->motor_controller.pid_ref : 0.0f;
    g_arm_soft_limit_debug.angle_ref_deg[1] =
        arm_elbow_motor != NULL ?
            arm_elbow_motor->motor_controller.pid_ref : 0.0f;
    g_arm_soft_limit_debug.angle_pid_output_dps[0] =
        arm_shoulder_motor != NULL ?
            arm_shoulder_motor->motor_controller.angle_PID.Output : 0.0f;
    g_arm_soft_limit_debug.angle_pid_output_dps[1] =
        arm_elbow_motor != NULL ?
            arm_elbow_motor->motor_controller.angle_PID.Output : 0.0f;
    g_arm_soft_limit_debug.speed_pid_output[0] =
        arm_shoulder_motor != NULL ?
            arm_shoulder_motor->motor_controller.speed_PID.Output : 0.0f;
    g_arm_soft_limit_debug.speed_pid_output[1] =
        arm_elbow_motor != NULL ?
            arm_elbow_motor->motor_controller.speed_PID.Output : 0.0f;
    g_arm_soft_limit_debug.current_pid_output[0] =
        arm_base_motor != NULL ?
            arm_base_motor->motor_controller.current_PID.Output : 0.0f;
    g_arm_soft_limit_debug.current_pid_output[1] =
        arm_shoulder_motor != NULL ?
            arm_shoulder_motor->motor_controller.current_PID.Output : 0.0f;
    g_arm_soft_limit_debug.current_pid_output[2] =
        arm_elbow_motor != NULL ?
            arm_elbow_motor->motor_controller.current_PID.Output : 0.0f;
    g_arm_soft_limit_debug.motion_test_pose_index = 0u;
    g_arm_soft_limit_debug.motion_test_target_reached = 0u;
    g_arm_soft_limit_debug.motion_test_hold_elapsed_ms = 0u;
    g_arm_soft_limit_debug.state_elapsed_ms =
        g_arm_state.soft_limit_elapsed_ms;
    if (active_motor != NULL) {
        g_arm_soft_limit_debug.active_angle_ref_deg =
            active_motor->motor_controller.pid_ref;
        g_arm_soft_limit_debug.active_angle_pid_output_dps =
            active_motor->motor_controller.angle_PID.Output;
        g_arm_soft_limit_debug.active_speed_pid_output =
            active_motor->motor_controller.speed_PID.Output;
    } else {
        g_arm_soft_limit_debug.active_angle_ref_deg = 0.0f;
        g_arm_soft_limit_debug.active_angle_pid_output_dps = 0.0f;
        g_arm_soft_limit_debug.active_speed_pid_output = 0.0f;
    }
}

static uint8_t ArmSoftLimitOvercurrent(DJIMotor_Instance *motor,
                                       float limit,
                                       uint32_t now)
{
    if (fabsf((float)motor->measure.real_current) < limit) {
        arm_soft_limit_overcurrent_tick = 0u;
        return 0u;
    }
    if (arm_soft_limit_overcurrent_tick == 0u) {
        arm_soft_limit_overcurrent_tick = now;
        return 0u;
    }
    return (uint32_t)(now - arm_soft_limit_overcurrent_tick) >=
           ARM_SOFT_LIMIT_OVERCURRENT_MS;
}

static void ArmSetAngleLoop(DJIMotor_Instance *motor)
{
    if (motor == NULL) {
        return;
    }
    motor->motor_settings.outer_loop_type = ANGLE_LOOP;
    motor->motor_settings.close_loop_type =
        (Closeloop_Type_e)(ANGLE_LOOP | SPEED_LOOP | CURRENT_LOOP);
}

static void ArmBaseResetControllerOnTargetCross(void)
{
    float target_error_deg;

    if (arm_base_motor == NULL) {
        return;
    }
    target_error_deg = g_arm_state.base_init_target_motor_deg -
                       arm_base_motor->measure.total_angle;
    if (arm_base_target_error_valid &&
        ((arm_base_last_target_error_deg > 0.05f &&
          target_error_deg < -0.05f) ||
         (arm_base_last_target_error_deg < -0.05f &&
          target_error_deg > 0.05f))) {
        ArmClearMotorController(arm_base_motor);
    }
    if (fabsf(target_error_deg) > 0.05f) {
        arm_base_last_target_error_deg = target_error_deg;
        arm_base_target_error_valid = 1u;
    }
}

static uint8_t ArmBaseInitMove(uint32_t now)
{
    float raw_error_deg;

    if (!isfinite(g_arm_state.base_init_target_motor_deg)) {
        ArmSoftLimitFail(ARM_SOFT_LIMIT_ERROR_LIMIT, now);
        return 0u;
    }
    if (!ArmMotorFeedbackReady(arm_base_motor)) {
        ArmSoftLimitFail(ARM_SOFT_LIMIT_ERROR_OFFLINE, now);
        return 0u;
    }
    g_arm_state.soft_limit_elapsed_ms =
        (uint32_t)(now - arm_soft_limit_state_tick);
    if (g_arm_state.soft_limit_elapsed_ms >= ARM_SOFT_LIMIT_TIMEOUT_MS) {
        ArmSoftLimitFail(ARM_SOFT_LIMIT_ERROR_TIMEOUT, now);
        return 0u;
    }
    if (ArmSoftLimitOvercurrent(arm_base_motor,
                                ARM_BASE_INIT_CURRENT, now)) {
        ArmSoftLimitFail(ARM_SOFT_LIMIT_ERROR_OVERCURRENT, now);
        return 0u;
    }

    ArmStopMotor(arm_shoulder_motor);
    ArmStopMotor(arm_elbow_motor);
    ArmBaseResetControllerOnTargetCross();
    ArmSetAngleLoop(arm_base_motor);
    DJIMotorSetRef(arm_base_motor,
                   g_arm_state.base_init_target_motor_deg);
    DJIMotorEnable(arm_base_motor);
    raw_error_deg = ArmWrapTo180(
        ARM_BASE_FRONT_RAW_DEG - arm_base_motor->measure.angle_single_round);
    if (fabsf(raw_error_deg) <= ARM_BASE_INIT_TARGET_TOLERANCE_DEG &&
        fabsf(arm_base_motor->measure.speed_aps) <=
            ARM_SOFT_LIMIT_TARGET_SPEED_DPS) {
        return 1u;
    }
    g_arm_state.mode = ARM_MODE_SOFT_LIMIT;
    return 0u;
}

static void ArmBaseInitHold(void)
{
    ArmBaseResetControllerOnTargetCross();
    ArmSetAngleLoop(arm_base_motor);
    DJIMotorSetRef(arm_base_motor,
                   g_arm_state.base_init_target_motor_deg);
    DJIMotorEnable(arm_base_motor);
}

static void ArmSoftLimitTask(uint32_t now)
{
    if (ARM_SOFT_LIMIT_ENABLE == 0u) {
        g_arm_state.soft_limit_state = ARM_SOFT_LIMIT_DISABLED;
        return;
    }
    ArmUpdateSoftLimitTargets();

    switch (g_arm_state.soft_limit_state) {
        case ARM_SOFT_LIMIT_WAIT_ONLINE:
            ArmStopMotor(arm_base_motor);
            ArmStopMotor(arm_shoulder_motor);
            ArmStopMotor(arm_elbow_motor);
            break;

        case ARM_SOFT_LIMIT_BASE_MOVING:
            if (ArmBaseInitMove(now)) {
                ArmSoftLimitEnterState(ARM_SOFT_LIMIT_BASE_SETTLE,
                                       ARM_SOFT_LIMIT_AXIS_BASE, now);
            }
            break;

        case ARM_SOFT_LIMIT_BASE_SETTLE:
            ArmStopMotor(arm_shoulder_motor);
            ArmStopMotor(arm_elbow_motor);
            ArmBaseInitHold();
            g_arm_state.soft_limit_elapsed_ms =
                (uint32_t)(now - arm_soft_limit_state_tick);
            if (g_arm_state.soft_limit_elapsed_ms >=
                ARM_SOFT_LIMIT_SETTLE_MS) {
                ArmSoftLimitEnterState(ARM_SOFT_LIMIT_WAIT_CALIBRATION,
                                       ARM_SOFT_LIMIT_AXIS_BASE, now);
            }
            break;

        case ARM_SOFT_LIMIT_WAIT_CALIBRATION:
            ArmBaseInitHold();
            if (g_arm_calibration.joint_calibrated &&
                g_arm_state.calibration_state == ARM_CAL_VALID) {
                ArmSoftLimitEnterState(ARM_SOFT_LIMIT_COMPLETE,
                                       ARM_SOFT_LIMIT_AXIS_BASE, now);
            }
            break;

        case ARM_SOFT_LIMIT_COMPLETE:
#if ARM_BOOT_MODE == ARM_BOOT_MODE_TEACH_POINT || \
    ARM_BOOT_MODE == ARM_BOOT_MODE_FULL_CALIBRATION
            /* 打点/维护扫描完成后保持三电机零输出，禁止保持逻辑重新使能。 */
            ArmStopMotor(arm_base_motor);
            ArmStopMotor(arm_shoulder_motor);
            ArmStopMotor(arm_elbow_motor);
#if ARM_BOOT_MODE == ARM_BOOT_MODE_TEACH_POINT
            arm_teach_mode_entered = 1u;
            g_arm_state.mode = ARM_MODE_TEACH_POINT;
#else
            g_arm_state.mode = ARM_MODE_SAFE;
#endif
            break;
#else
            if (!ArmMotorFeedbackReady(arm_base_motor) ||
                !ArmMotorFeedbackReady(arm_shoulder_motor) ||
                !ArmMotorFeedbackReady(arm_elbow_motor)) {
                /* A temporary feedback loss must not disable the other axes.
                   Keep the last references and resume when feedback returns. */
                break;
            }
            if (ArmTrajectoryOwnsControl()) {
                /* Trajectory owns the references, but all three motors must
                   remain enabled throughout staging, motion and settling. */
                ArmSetAngleLoop(arm_base_motor);
                ArmSetAngleLoop(arm_shoulder_motor);
                ArmSetAngleLoop(arm_elbow_motor);
                DJIMotorEnable(arm_base_motor);
                DJIMotorEnable(arm_shoulder_motor);
                DJIMotorEnable(arm_elbow_motor);
                g_arm_state.mode = ARM_MODE_READY;
                break;
            }
            if (!ArmTrajectoryMotorHoldAllowed()) {
                ArmMotionStopMotors();
                break;
            }
            ArmSetAngleLoop(arm_shoulder_motor);
            ArmSetAngleLoop(arm_elbow_motor);
            DJIMotorSetRef(arm_shoulder_motor,
                           arm_joint_hold_target_motor_deg[0]);
            DJIMotorSetRef(arm_elbow_motor,
                           arm_joint_hold_target_motor_deg[1]);
            DJIMotorEnable(arm_shoulder_motor);
            DJIMotorEnable(arm_elbow_motor);
            ArmBaseInitHold();
            g_arm_state.mode = ARM_MODE_READY;
            break;
#endif

        case ARM_SOFT_LIMIT_ERROR_OFFLINE:
        case ARM_SOFT_LIMIT_ERROR_LIMIT:
        case ARM_SOFT_LIMIT_ERROR_OVERCURRENT:
        case ARM_SOFT_LIMIT_ERROR_TIMEOUT:
        case ARM_SOFT_LIMIT_ABORTED:
            ArmStopMotor(arm_base_motor);
            ArmStopMotor(arm_shoulder_motor);
            ArmStopMotor(arm_elbow_motor);
            g_arm_state.mode = ARM_MODE_SAFE;
            break;

        case ARM_SOFT_LIMIT_DISABLED:
        default:
            ArmSoftLimitFail(ARM_SOFT_LIMIT_ERROR_LIMIT, now);
            break;
    }
}

void ArmInit(void)
{
    Motor_Init_Config_s base_config = ArmBaseMotorConfig();
    Motor_Init_Config_s shoulder_config = ArmShoulderMotorConfig();
    Motor_Init_Config_s elbow_config = ArmElbowMotorConfig();

    memset(&g_arm_state, 0, sizeof(g_arm_state));
    memset(&g_arm_calibration, 0, sizeof(g_arm_calibration));
    memset(&g_arm_soft_limit_debug, 0, sizeof(g_arm_soft_limit_debug));
    memset(&g_arm_kinematics_debug, 0, sizeof(g_arm_kinematics_debug));
    memset(&g_arm_teach_point, 0, sizeof(g_arm_teach_point));
    g_arm_shoulder_feedforward.active = 0u;
    g_arm_shoulder_feedforward.q2_deg = 0.0f;
    g_arm_shoulder_feedforward.q2_plus_q3_deg = 0.0f;
    g_arm_shoulder_feedforward.angle_window_scale = 0.0f;
    g_arm_shoulder_feedforward.cos_q2 = 0.0f;
    g_arm_shoulder_feedforward.cos_q2_plus_q3 = 0.0f;
    g_arm_shoulder_feedforward.shoulder_term_current = 0.0f;
    g_arm_shoulder_feedforward.link_load_term_current = 0.0f;
    g_arm_shoulder_feedforward.raw_current = 0.0f;
    g_arm_shoulder_feedforward.output_current = 0.0f;
    arm_base_motor = DJIMotorInit(&base_config);
    arm_shoulder_motor = DJIMotorInit(&shoulder_config);
    arm_elbow_motor = DJIMotorInit(&elbow_config);
    arm_motors[0] = arm_base_motor;
    arm_motors[1] = arm_shoulder_motor;
    arm_motors[2] = arm_elbow_motor;
    g_arm_state.mode = ARM_MODE_SAFE;
    g_arm_state.calibration_state = ARM_CAL_IDLE;
    ArmWristInit();
    g_arm_state.wrist_pwm_us = ArmWristGetState()->pulse_us;
    g_arm_homing_abort = 0u;
    arm_auto_calibration_attempted = 0u;
    arm_auto_online_waiting = 0u;
    arm_auto_online_start_tick = 0u;
    arm_base_front_raw_deg = ARM_BASE_FRONT_RAW_DEG;
    arm_base_direction = ARM_BASE_DIRECTION;
    g_arm_calibration.base_calibrated = ARM_BASE_ZERO_CONFIGURED != 0u;
    ArmLoadMeasuredJointMapping();
    ArmUpdateCalibrationValid();
    arm_ratio_test_state_tick = HAL_GetTick();
    arm_ratio_test_stall_tick = 0u;
    arm_soft_limit_state_tick = HAL_GetTick();
    arm_soft_limit_overcurrent_tick = 0u;
    arm_joint_hold_target_motor_deg[0] = 0.0f;
    arm_joint_hold_target_motor_deg[1] = 0.0f;
    arm_base_last_target_error_deg = 0.0f;
    arm_base_target_error_valid = 0u;
    arm_teach_mode_entered = 0u;
    arm_homing_speed_pid_loaded = 0u;
    g_arm_state.q_target_deg[ARM_JOINT_BASE_YAW] = 0.0f;
    g_arm_state.q_target_deg[ARM_JOINT_SHOULDER] =
        ARM_SHOULDER_REFERENCE_DEG;
    g_arm_state.q_target_deg[ARM_JOINT_ELBOW] =
        ARM_ELBOW_REFERENCE_DEG;
    g_arm_state.ratio_test_state =
        ARM_BOOT_MODE == ARM_BOOT_MODE_SHOULDER_RATIO_TEST ?
        ARM_RATIO_TEST_WAIT_ONLINE : ARM_RATIO_TEST_DISABLED;
    g_arm_state.soft_limit_axis = ARM_SOFT_LIMIT_AXIS_NONE;
    g_arm_state.soft_limit_state = ARM_SOFT_LIMIT_ENABLE != 0u ?
        ARM_SOFT_LIMIT_WAIT_ONLINE : ARM_SOFT_LIMIT_DISABLED;
    ArmUpdateSoftLimitTargets();
    ArmStop();
    ArmUpdateFeedback();
    ArmUpdateForwardKinematics();
    ArmUpdateKinematicsDebug();
    ArmUpdateSoftLimitDebug();
    ArmTrajectoryInit();
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
    ArmUpdateFeedback();
    ArmUpdateShoulderGravityFeedforward();

    if (g_arm_homing_abort) {
        arm_auto_calibration_attempted = 1u;
        ArmAbortMotion(ARM_MOTION_FAULT_ABORT);
        if (ARM_BOOT_MODE == ARM_BOOT_MODE_SHOULDER_RATIO_TEST) {
            ArmRatioTestFail(ARM_RATIO_TEST_ABORTED, now);
        } else {
            if (ArmCalibrationStateIsActive(
                    g_arm_state.calibration_state)) {
                ArmCalibrationAbort();
            } else {
                ArmSoftLimitFail(ARM_SOFT_LIMIT_ABORTED, now);
            }
        }
        ArmUpdateFeedback();
        ArmUpdateShoulderGravityFeedforward();
        ArmUpdateForwardKinematics();
        ArmUpdateKinematicsDebug();
        ArmUpdateSoftLimitDebug();
        return;
    }

    if (ARM_BOOT_MODE == ARM_BOOT_MODE_SHOULDER_RATIO_TEST) {
        /* 这些函数只属于正常初始化链路；保留符号引用以便维护模式单独编译。 */
        (void)ArmUpdateTeachPoint;
        (void)ArmCalibrationTask;
        (void)ArmAutoCalibrationTask;
        (void)ArmSoftLimitTask;
        Arm3508RatioTestTask(now);
        ArmUpdateFeedback();
        ArmUpdateShoulderGravityFeedforward();
        ArmUpdateForwardKinematics();
        ArmUpdateKinematicsDebug();
        ArmUpdateSoftLimitDebug();
        return;
    }
#if ARM_BOOT_MODE != ARM_BOOT_MODE_SHOULDER_RATIO_TEST

    ArmAutoCalibrationTask(now);
    ArmCalibrationTask(now);
    ArmUpdateFeedback();
    ArmSoftLimitTask(now);
    ArmUpdateFeedback();
    ArmUpdateForwardKinematics();
    ArmUpdateKinematicsDebug();
    ArmTrajectoryTask(now);
    ArmUpdateFeedback();
    ArmUpdateForwardKinematics();
    ArmUpdateTeachPoint();

    if (g_arm_state.soft_limit_state >= ARM_SOFT_LIMIT_ERROR_OFFLINE &&
        g_arm_state.soft_limit_state <= ARM_SOFT_LIMIT_ABORTED) {
        g_arm_state.mode = ARM_MODE_SAFE;
    } else if (g_arm_state.soft_limit_state ==
                   ARM_SOFT_LIMIT_BASE_MOVING ||
               g_arm_state.soft_limit_state ==
                   ARM_SOFT_LIMIT_BASE_SETTLE) {
        g_arm_state.mode = ARM_MODE_SOFT_LIMIT;
    } else if (ARM_BOOT_MODE == ARM_BOOT_MODE_TEACH_POINT &&
               arm_teach_mode_entered) {
        g_arm_state.mode = ARM_MODE_TEACH_POINT;
    } else if (g_arm_calibration.joint_calibrated &&
        g_arm_state.calibration_state == ARM_CAL_VALID) {
        g_arm_state.mode = ARM_MODE_READY;
    } else if (ArmCalibrationStateIsActive(g_arm_state.calibration_state)) {
        g_arm_state.mode = ARM_MODE_CALIBRATION;
    } else {
        g_arm_state.mode = ARM_MODE_SAFE;
    }
    ArmUpdateShoulderGravityFeedforward();
    ArmUpdateSoftLimitDebug();
#endif
}

const Arm_State_s *ArmGetState(void)
{
    return &g_arm_state;
}

const Arm_Motion_Debug_s *ArmGetMotionState(void)
{
    return &g_arm_motion_debug;
}

const Arm_Teach_Point_s *ArmGetTeachPoint(void)
{
    return &g_arm_teach_point;
}

#if ARM_BOOT_MODE == ARM_BOOT_MODE_NORMAL
static Arm_Command_Result_e ArmConvertMotionResult(
    Arm_Motion_Result_e result)
{
    switch (result) {
        case ARM_MOTION_RESULT_OK:
            return ARM_COMMAND_OK;
        case ARM_MOTION_RESULT_BUSY:
            return ARM_COMMAND_BUSY;
        case ARM_MOTION_RESULT_NOT_READY:
            return ARM_COMMAND_NOT_READY;
        case ARM_MOTION_RESULT_INVALID:
            return ARM_COMMAND_INVALID;
        case ARM_MOTION_RESULT_PREFLIGHT_FAILED:
        default:
            return ARM_COMMAND_PREFLIGHT_FAILED;
    }
}
#endif

static uint8_t ArmApplicationCommandAllowed(void)
{
    return ARM_BOOT_MODE == ARM_BOOT_MODE_NORMAL &&
           g_arm_state.soft_limit_state == ARM_SOFT_LIMIT_COMPLETE &&
           g_arm_calibration.calibration_valid &&
           g_arm_state.motor_online[0] && g_arm_state.motor_online[1] &&
           g_arm_state.motor_online[2] && !g_arm_homing_abort;
}

Arm_Command_Result_e ArmSubmitCartesianCommand(
    const Arm_Cartesian_Command_s *command)
{
#if ARM_BOOT_MODE != ARM_BOOT_MODE_NORMAL
    (void)command;
    return ARM_COMMAND_MODE_DENIED;
#else
    Arm_Motion_Result_e result;
    float speed_mm_s;
    uint8_t outside_soft_limit;

    if (command == NULL || !isfinite(command->target_mm.x_mm) ||
        !isfinite(command->target_mm.y_mm) ||
        !isfinite(command->target_mm.z_mm) ||
        !isfinite(command->max_speed_mm_s) ||
        command->max_speed_mm_s < 0.0f ||
        (command->tool_pitch_valid != 0u &&
         !isfinite(command->tool_pitch_deg))) {
        return ARM_COMMAND_INVALID;
    }
    if (command->control_point != ARM_CONTROL_POINT_WRIST_CENTER ||
        command->tool_pitch_valid != 0u) {
        return ARM_COMMAND_UNSUPPORTED;
    }
    if (command->move_type != ARM_MOVE_DIRECT &&
        command->move_type != ARM_MOVE_LINEAR) {
        return ARM_COMMAND_INVALID;
    }
    if (!ArmApplicationCommandAllowed()) {
        return ARM_COMMAND_NOT_READY;
    }
    if (ArmTrajectoryIsBusy()) {
        return ARM_COMMAND_BUSY;
    }
    speed_mm_s = command->max_speed_mm_s > 0.0f ?
        command->max_speed_mm_s : ARM_LINEAR_DEFAULT_SPEED_MM_S;
    if (speed_mm_s > ARM_LINEAR_DEFAULT_SPEED_MM_S) {
        return ARM_COMMAND_INVALID;
    }

    outside_soft_limit = !ArmJointPoseWithinSoftLimits(
        g_arm_state.q_feedback_deg);
    if (outside_soft_limit) {
        Arm_Cartesian_Command_s staged_command = *command;

        staged_command.max_speed_mm_s = speed_mm_s;
        result = ArmTrajectoryStageCartesianCommand(&staged_command);
    } else if (command->move_type == ARM_MOVE_DIRECT) {
        result = ArmSetCartesianTarget(&command->target_mm, NULL);
    } else {
        result = ArmMoveLinear(&command->target_mm, speed_mm_s);
    }
    return ArmConvertMotionResult(result);
#endif
}

Arm_Command_Result_e ArmSubmitJointCommand(
    const Arm_Joint_Command_s *command)
{
#if ARM_BOOT_MODE != ARM_BOOT_MODE_NORMAL
    (void)command;
    return ARM_COMMAND_MODE_DENIED;
#else
    Arm_Motion_Result_e result;

    if (command == NULL || !isfinite(command->q_deg[0]) ||
        !isfinite(command->q_deg[1]) || !isfinite(command->q_deg[2]) ||
        (command->move_type != ARM_MOVE_DIRECT &&
         command->move_type != ARM_MOVE_LINEAR)) {
        return ARM_COMMAND_INVALID;
    }
    if (!ArmApplicationCommandAllowed()) {
        return ARM_COMMAND_NOT_READY;
    }
    if (ArmTrajectoryIsBusy()) {
        return ARM_COMMAND_BUSY;
    }
    if (!ArmJointPoseWithinSoftLimits(command->q_deg) ||
        !ArmAutoPoseIsSafe(command->q_deg)) {
        return ARM_COMMAND_PREFLIGHT_FAILED;
    }
    result = command->move_type == ARM_MOVE_DIRECT ?
        ArmTrajectorySetJointDirect(command->q_deg) :
        ArmTrajectoryMoveJoint(command->q_deg);
    return ArmConvertMotionResult(result);
#endif
}

void ArmCancelMotion(void)
{
    if (ARM_BOOT_MODE != ARM_BOOT_MODE_NORMAL ||
        !ArmApplicationCommandAllowed()) {
        return;
    }
    ArmTrajectoryCancel();
}

void ArmEmergencyStop(void)
{
    g_arm_homing_abort = 1u;
    ArmAbortMotion(ARM_MOTION_FAULT_ABORT);
    ArmStop();
}
