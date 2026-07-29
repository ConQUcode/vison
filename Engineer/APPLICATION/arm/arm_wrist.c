#include "arm_wrist.h"

#include "arm_config.h"
#include "math.h"
#include "string.h"
#include "tim.h"

static Arm_Wrist_State_s arm_wrist_state;

static float ArmWristClamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

Arm_Wrist_Result_e ArmWristInit(void)
{
    memset(&arm_wrist_state, 0, sizeof(arm_wrist_state));
    arm_wrist_state.pulse_us = ARM_WRIST_PWM_MID_US;
#if ARM_WRIST_ENABLE == 0u
    return ARM_WRIST_NOT_CONFIGURED;
#else
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
        return ARM_WRIST_HAL_ERROR;
    }
    arm_wrist_state.configured = 1u;
    arm_wrist_state.pwm_started = 1u;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ARM_WRIST_PWM_MID_US);
    return ARM_WRIST_OK;
#endif
}

Arm_Wrist_Result_e ArmWristSetPulseUs(uint16_t pulse_us)
{
    if (!arm_wrist_state.configured || !arm_wrist_state.pwm_started) {
        return ARM_WRIST_NOT_CONFIGURED;
    }
    if (pulse_us < ARM_WRIST_PWM_MIN_US ||
        pulse_us > ARM_WRIST_PWM_MAX_US) {
        return ARM_WRIST_INVALID_ARGUMENT;
    }
    arm_wrist_state.pulse_us = pulse_us;
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse_us);
    return ARM_WRIST_OK;
}

Arm_Wrist_Result_e ArmWristSetAngleDeg(float angle_deg)
{
    float normalized_angle;
    float pulse_us;

    if (!arm_wrist_state.configured || !arm_wrist_state.pwm_started) {
        return ARM_WRIST_NOT_CONFIGURED;
    }
    if (!isfinite(angle_deg) || angle_deg < ARM_WRIST_MIN_ANGLE_DEG ||
        angle_deg > ARM_WRIST_MAX_ANGLE_DEG) {
        return ARM_WRIST_INVALID_ARGUMENT;
    }
    normalized_angle = ARM_WRIST_DIRECTION *
        (angle_deg + ARM_WRIST_ZERO_OFFSET_DEG);
    normalized_angle = ArmWristClamp(normalized_angle,
                                    ARM_WRIST_MIN_ANGLE_DEG,
                                    ARM_WRIST_MAX_ANGLE_DEG);
    pulse_us = (float)ARM_WRIST_PWM_MIN_US +
        (normalized_angle - ARM_WRIST_MIN_ANGLE_DEG) *
        (float)(ARM_WRIST_PWM_MAX_US - ARM_WRIST_PWM_MIN_US) /
        (ARM_WRIST_MAX_ANGLE_DEG - ARM_WRIST_MIN_ANGLE_DEG);
    arm_wrist_state.target_angle_deg = angle_deg;
    return ArmWristSetPulseUs((uint16_t)(pulse_us + 0.5f));
}

const Arm_Wrist_State_s *ArmWristGetState(void)
{
    return &arm_wrist_state;
}
