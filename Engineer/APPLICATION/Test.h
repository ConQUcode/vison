#ifndef __TEST_H
#define __TEST_H

#include "arm.h"
#include "hsl_servo.h"

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

typedef enum {
    FEETECH_TEST_STATE_DISABLED = 0,
    FEETECH_TEST_STATE_WAIT_START,
    FEETECH_TEST_STATE_SEND,
    FEETECH_TEST_STATE_WAIT_STEP,
    FEETECH_TEST_STATE_DONE,
    FEETECH_TEST_STATE_ERROR
} Feetech_Servo_Test_State_e;

typedef struct {
    uint8_t enabled;
    Feetech_Servo_Test_State_e state;
    uint8_t id;
    uint8_t step;
    uint16_t target_deg;
    uint16_t target_position;
    HSLServo_Result_e last_result;
    uint32_t start_tick;
    uint32_t last_send_tick;
    uint32_t send_count;
    uint32_t busy_count;
    uint32_t error_count;
} Feetech_Servo_Test_Debug_s;

extern Feetech_Servo_Test_Debug_s g_feetech_servo_test_debug;

void all_init_Task(void);
void all_cmd_Task(void);



#endif
