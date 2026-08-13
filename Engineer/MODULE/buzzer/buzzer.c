/**
 * @file buzzer.c
 * @brief 蜂鸣器非阻塞定时控制及 Watch 统计。
 */

#include "buzzer.h"

#include "tim.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define BUZZER_TIM_CHANNEL TIM_CHANNEL_3
#define BUZZER_COMPARE_50_PERCENT 125u

Buzzer_Debug_s g_buzzer_debug;

void BuzzerInit(void)
{
    memset(&g_buzzer_debug, 0, sizeof(g_buzzer_debug));
    __HAL_TIM_SET_COMPARE(&htim4, BUZZER_TIM_CHANNEL, 0u);
    (void)HAL_TIM_PWM_Stop(&htim4, BUZZER_TIM_CHANNEL);
}

uint8_t BuzzerStart(uint32_t duration_ms)
{
    if (duration_ms == 0u) {
        BuzzerStop();
        return 1u;
    }
    __HAL_TIM_SET_COMPARE(&htim4, BUZZER_TIM_CHANNEL,
                          BUZZER_COMPARE_50_PERCENT);
    if (HAL_TIM_PWM_Start(&htim4, BUZZER_TIM_CHANNEL) != HAL_OK) {
        __HAL_TIM_SET_COMPARE(&htim4, BUZZER_TIM_CHANNEL, 0u);
        g_buzzer_debug.active = 0u;
        g_buzzer_debug.last_result = 0u;
        g_buzzer_debug.fail_count++;
        return 0u;
    }
    g_buzzer_debug.active = 1u;
    g_buzzer_debug.last_result = 1u;
    g_buzzer_debug.duration_ms = duration_ms;
    g_buzzer_debug.start_tick = HAL_GetTick();
    g_buzzer_debug.trigger_count++;
    return 1u;
}

void BuzzerStop(void)
{
    if (g_buzzer_debug.active != 0u) {
        g_buzzer_debug.complete_count++;
    }
    g_buzzer_debug.active = 0u;
    __HAL_TIM_SET_COMPARE(&htim4, BUZZER_TIM_CHANNEL, 0u);
    (void)HAL_TIM_PWM_Stop(&htim4, BUZZER_TIM_CHANNEL);
}

void BuzzerTask(uint32_t now_ms)
{
    if (g_buzzer_debug.active != 0u &&
        (uint32_t)(now_ms - g_buzzer_debug.start_tick) >=
            g_buzzer_debug.duration_ms) {
        BuzzerStop();
    }
}
