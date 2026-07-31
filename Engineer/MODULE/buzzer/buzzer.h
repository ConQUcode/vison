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

void BuzzerInit(void);
uint8_t BuzzerStart(uint32_t duration_ms);
void BuzzerStop(void);
void BuzzerTask(uint32_t now_ms);

#endif
