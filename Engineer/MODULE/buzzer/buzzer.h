/**
 * @file buzzer.h
 * @brief 蜂鸣器初始化、限时鸣响、停止和周期维护接口。
 */

#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

typedef struct {
    uint8_t active;
    uint8_t last_result;
    uint32_t start_tick;
    uint32_t duration_ms;
    uint32_t trigger_count;
    uint32_t complete_count;
    uint32_t fail_count;
} Buzzer_Debug_s;

extern Buzzer_Debug_s g_buzzer_debug;

/** 初始化 PWM 输出并确保蜂鸣器处于关闭状态。 */
void BuzzerInit(void);
/** 启动一次限时鸣响，duration_ms 单位为毫秒；返回 1 表示启动成功。 */
uint8_t BuzzerStart(uint32_t duration_ms);
/** 立即停止蜂鸣器。 */
void BuzzerStop(void);
/** 每个 USB 应用周期调用，用于到时自动停止。 */
void BuzzerTask(uint32_t now_ms);

#endif
