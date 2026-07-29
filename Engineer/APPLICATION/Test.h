#ifndef __TEST_H
#define __TEST_H

#include "arm.h"

/* 本次坐标接口回放的唯一Watch变量，正式比赛逻辑不依赖此结构。 */
typedef struct {
    uint8_t enabled;
    uint8_t submitted;
    uint32_t submit_count;
    Arm_Command_Result_e result;
    Arm_Position_s target_mm;
} Arm_Api_Test_Debug_s;

extern Arm_Api_Test_Debug_s g_arm_api_test_debug;

void all_init_Task(void);
void all_cmd_Task(void);



#endif
