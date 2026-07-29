#ifndef __ARM_WRIST_H__
#define __ARM_WRIST_H__

#include "stdint.h"

typedef enum {
    ARM_WRIST_OK = 0,
    ARM_WRIST_NOT_CONFIGURED,
    ARM_WRIST_INVALID_ARGUMENT,
    ARM_WRIST_HAL_ERROR
} Arm_Wrist_Result_e;

typedef struct {
    uint8_t configured;
    uint8_t pwm_started;
    uint16_t pulse_us;
    float target_angle_deg;
} Arm_Wrist_State_s;

Arm_Wrist_Result_e ArmWristInit(void);
Arm_Wrist_Result_e ArmWristSetPulseUs(uint16_t pulse_us);
Arm_Wrist_Result_e ArmWristSetAngleDeg(float angle_deg);
const Arm_Wrist_State_s *ArmWristGetState(void);

#endif
