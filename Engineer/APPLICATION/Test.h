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

typedef enum {
    ARM_HOST_SIM_WAIT_READY = 0,
    ARM_HOST_SIM_MOVE_TO_CENTER,
    ARM_HOST_SIM_WAIT_CENTER,
    ARM_HOST_SIM_STREAMING
} Arm_Host_Sim_State_e;

/* Watch只需观察此结构，确认100Hz发送、目标坐标和返回状态。 */
typedef struct {
    uint8_t enabled;
    Arm_Host_Sim_State_e state;
    Arm_Command_Result_e last_result;
    uint32_t command_id;
    uint32_t send_count;
    uint32_t reject_count;
    uint32_t last_send_tick;
    uint32_t stream_start_tick;
    float phase_rad;
    float generated_q_deg[3];
    Arm_Position_s target_mm;
} Arm_Host_Sim_Debug_s;

extern Arm_Host_Sim_Debug_s g_arm_host_sim_debug;

void all_init_Task(void);
void all_cmd_Task(void);



#endif
