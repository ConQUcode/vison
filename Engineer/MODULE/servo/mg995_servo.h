/**
 * @file mg995_servo.h
 * @brief TIM8双路MG995 PWM控制接口。
 */

#ifndef MG995_SERVO_H
#define MG995_SERVO_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

/* 当前板级映射：PI6/TIM8_CH2为右侧，PI7/TIM8_CH3为左侧。 */
#define MG995_SERVO_RIGHT_CHANNEL       TIM_CHANNEL_2
#define MG995_SERVO_LEFT_CHANNEL        TIM_CHANNEL_3

/* TIM8以1 MHz计数，比较值数值等于高电平脉宽us。 */
#define MG995_SERVO_PWM_PRESCALER             167u
#define MG995_SERVO_PWM_PERIOD              19999u
#define MG995_SERVO_MIN_PULSE_US             1000u
#define MG995_SERVO_CENTER_PULSE_US          1500u
#define MG995_SERVO_MAX_PULSE_US             2000u
#define MG995_SERVO_MIN_ANGLE_DEG              0.0f
#define MG995_SERVO_CENTER_ANGLE_DEG           90.0f
#define MG995_SERVO_MAX_ANGLE_DEG             180.0f

/* 摄像头坐标：舵机90deg为水平0deg，正角度表示摄像头向上。 */
#define MG995_CAMERA_MIN_ANGLE_DEG             (-90.0f)
#define MG995_CAMERA_MAX_ANGLE_DEG               90.0f
#define MG995_CAMERA_STARTUP_ANGLE_DEG            0.0f
#define MG995_LEFT_CAMERA_TO_SERVO_SIGN           1.0f
#define MG995_RIGHT_CAMERA_TO_SERVO_SIGN         (-1.0f)

typedef enum {
    MG995_SERVO_RIGHT = 0,
    MG995_SERVO_LEFT
} Mg995_Servo_Id_e;

typedef enum {
    MG995_SERVO_STATE_UNINITIALIZED = 0,
    MG995_SERVO_STATE_READY,
    MG995_SERVO_STATE_CONFIG_ERROR,
    MG995_SERVO_STATE_HAL_ERROR
} Mg995_Servo_State_e;

typedef struct {
    Mg995_Servo_State_e state;
    uint8_t initialized;
    uint16_t right_pulse_us;
    uint16_t left_pulse_us;
    float right_angle_deg;
    float left_angle_deg;
    float right_camera_angle_deg;
    float left_camera_angle_deg;
    uint32_t init_count;
    uint32_t set_count;
    uint32_t hal_error_count;
} Mg995_Servo_Debug_s;

extern Mg995_Servo_Debug_s g_mg995_servo_debug;

/** 校验TIM8配置，随后让左右舵机直接运动到上电目标角；成功返回1。 */
uint8_t Mg995ServoInit(void);

/** 设置单侧目标角，当前允许范围0..180deg；成功返回1。 */
uint8_t Mg995ServoSetAngle(Mg995_Servo_Id_e servo, float angle_deg);

/** 同一调用中更新左右两侧目标角；参数非法时两路都不改变。 */
uint8_t Mg995ServoSetBothAngles(float left_angle_deg,
                                float right_angle_deg);

/** 以水平为0deg、向上为正，同时设置左右摄像头物理目标角。 */
uint8_t Mg995ServoSetCameraAngles(float left_camera_angle_deg,
                                  float right_camera_angle_deg);

#endif
