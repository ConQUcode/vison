/**
 * @file mg995_servo.c
 * @brief 使用TIM8_CH2/CH3输出两路50Hz MG995控制脉冲。
 */

#include "mg995_servo.h"

#include <math.h>
#include <string.h>

#include "tim.h"

Mg995_Servo_Debug_s g_mg995_servo_debug;

static uint8_t Mg995ServoAngleValid(float angle_deg)
{
    return (uint8_t)(isfinite(angle_deg) &&
        angle_deg >= MG995_SERVO_MIN_ANGLE_DEG &&
        angle_deg <= MG995_SERVO_MAX_ANGLE_DEG);
}

static uint8_t Mg995CameraAngleValid(float angle_deg)
{
    return (uint8_t)(isfinite(angle_deg) &&
        angle_deg >= MG995_CAMERA_MIN_ANGLE_DEG &&
        angle_deg <= MG995_CAMERA_MAX_ANGLE_DEG);
}

static float Mg995CameraToServoAngle(Mg995_Servo_Id_e servo,
                                     float camera_angle_deg)
{
    float sign = servo == MG995_SERVO_LEFT ?
        MG995_LEFT_CAMERA_TO_SERVO_SIGN :
        MG995_RIGHT_CAMERA_TO_SERVO_SIGN;

    return MG995_SERVO_CENTER_ANGLE_DEG + sign * camera_angle_deg;
}

static uint16_t Mg995ServoAngleToPulse(float angle_deg)
{
    const float angle_span = MG995_SERVO_MAX_ANGLE_DEG -
        MG995_SERVO_MIN_ANGLE_DEG;
    const float pulse_span = (float)(MG995_SERVO_MAX_PULSE_US -
        MG995_SERVO_MIN_PULSE_US);
    float pulse = (float)MG995_SERVO_MIN_PULSE_US +
        (angle_deg - MG995_SERVO_MIN_ANGLE_DEG) * pulse_span / angle_span;

    return (uint16_t)(pulse + 0.5f);
}

static void Mg995ServoWrite(Mg995_Servo_Id_e servo,
                            float angle_deg,
                            uint16_t pulse_us)
{
    if (servo == MG995_SERVO_LEFT) {
        __HAL_TIM_SET_COMPARE(&htim8, MG995_SERVO_LEFT_CHANNEL, pulse_us);
        g_mg995_servo_debug.left_angle_deg = angle_deg;
        g_mg995_servo_debug.left_camera_angle_deg =
            (angle_deg - MG995_SERVO_CENTER_ANGLE_DEG) /
            MG995_LEFT_CAMERA_TO_SERVO_SIGN;
        g_mg995_servo_debug.left_pulse_us = pulse_us;
    } else {
        __HAL_TIM_SET_COMPARE(&htim8, MG995_SERVO_RIGHT_CHANNEL, pulse_us);
        g_mg995_servo_debug.right_angle_deg = angle_deg;
        g_mg995_servo_debug.right_camera_angle_deg =
            (angle_deg - MG995_SERVO_CENTER_ANGLE_DEG) /
            MG995_RIGHT_CAMERA_TO_SERVO_SIGN;
        g_mg995_servo_debug.right_pulse_us = pulse_us;
    }
}

uint8_t Mg995ServoInit(void)
{
    float left_servo_angle_deg;
    float right_servo_angle_deg;
    uint16_t left_pulse_us;
    uint16_t right_pulse_us;

    memset(&g_mg995_servo_debug, 0, sizeof(g_mg995_servo_debug));
    g_mg995_servo_debug.init_count = 1u;

    if (htim8.Instance != TIM8 ||
        htim8.Init.Prescaler != MG995_SERVO_PWM_PRESCALER ||
        htim8.Init.Period != MG995_SERVO_PWM_PERIOD) {
        g_mg995_servo_debug.state = MG995_SERVO_STATE_CONFIG_ERROR;
        return 0u;
    }

    left_servo_angle_deg = Mg995CameraToServoAngle(
        MG995_SERVO_LEFT, MG995_CAMERA_STARTUP_ANGLE_DEG);
    right_servo_angle_deg = Mg995CameraToServoAngle(
        MG995_SERVO_RIGHT, MG995_CAMERA_STARTUP_ANGLE_DEG);
    left_pulse_us = Mg995ServoAngleToPulse(left_servo_angle_deg);
    right_pulse_us = Mg995ServoAngleToPulse(right_servo_angle_deg);
    Mg995ServoWrite(MG995_SERVO_RIGHT,
                    right_servo_angle_deg,
                    right_pulse_us);
    Mg995ServoWrite(MG995_SERVO_LEFT,
                    left_servo_angle_deg,
                    left_pulse_us);

    if (HAL_TIM_PWM_Start(&htim8, MG995_SERVO_RIGHT_CHANNEL) != HAL_OK) {
        g_mg995_servo_debug.state = MG995_SERVO_STATE_HAL_ERROR;
        g_mg995_servo_debug.hal_error_count++;
        return 0u;
    }
    if (HAL_TIM_PWM_Start(&htim8, MG995_SERVO_LEFT_CHANNEL) != HAL_OK) {
        (void)HAL_TIM_PWM_Stop(&htim8, MG995_SERVO_RIGHT_CHANNEL);
        g_mg995_servo_debug.state = MG995_SERVO_STATE_HAL_ERROR;
        g_mg995_servo_debug.hal_error_count++;
        return 0u;
    }

    g_mg995_servo_debug.initialized = 1u;
    g_mg995_servo_debug.state = MG995_SERVO_STATE_READY;
    return 1u;
}

uint8_t Mg995ServoSetAngle(Mg995_Servo_Id_e servo, float angle_deg)
{
    uint16_t pulse_us;

    if (g_mg995_servo_debug.initialized == 0u ||
        (servo != MG995_SERVO_LEFT && servo != MG995_SERVO_RIGHT) ||
        Mg995ServoAngleValid(angle_deg) == 0u) {
        return 0u;
    }
    pulse_us = Mg995ServoAngleToPulse(angle_deg);
    Mg995ServoWrite(servo, angle_deg, pulse_us);
    g_mg995_servo_debug.set_count++;
    return 1u;
}

uint8_t Mg995ServoSetBothAngles(float left_angle_deg,
                                float right_angle_deg)
{
    uint16_t left_pulse_us;
    uint16_t right_pulse_us;

    if (g_mg995_servo_debug.initialized == 0u ||
        Mg995ServoAngleValid(left_angle_deg) == 0u ||
        Mg995ServoAngleValid(right_angle_deg) == 0u) {
        return 0u;
    }
    left_pulse_us = Mg995ServoAngleToPulse(left_angle_deg);
    right_pulse_us = Mg995ServoAngleToPulse(right_angle_deg);
    Mg995ServoWrite(MG995_SERVO_LEFT, left_angle_deg, left_pulse_us);
    Mg995ServoWrite(MG995_SERVO_RIGHT, right_angle_deg, right_pulse_us);
    g_mg995_servo_debug.set_count++;
    return 1u;
}

uint8_t Mg995ServoSetCameraAngles(float left_camera_angle_deg,
                                  float right_camera_angle_deg)
{
    float left_servo_angle_deg;
    float right_servo_angle_deg;

    if (Mg995CameraAngleValid(left_camera_angle_deg) == 0u ||
        Mg995CameraAngleValid(right_camera_angle_deg) == 0u) {
        return 0u;
    }
    left_servo_angle_deg = Mg995CameraToServoAngle(
        MG995_SERVO_LEFT, left_camera_angle_deg);
    right_servo_angle_deg = Mg995CameraToServoAngle(
        MG995_SERVO_RIGHT, right_camera_angle_deg);
    return Mg995ServoSetBothAngles(left_servo_angle_deg,
                                   right_servo_angle_deg);
}
