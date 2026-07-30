#ifndef __ARM_TOOL_H__
#define __ARM_TOOL_H__

#include "arm_host.h"
#include "stdint.h"

typedef enum {
    ARM_TOOL_ERROR_NONE = 0,
    ARM_TOOL_ERROR_INVALID_ARGUMENT,
    ARM_TOOL_ERROR_SERVO_RANGE,
    ARM_TOOL_ERROR_SERVO_TX,
    ARM_TOOL_ERROR_GEOMETRY
} Arm_Tool_Error_e;

typedef enum {
    ARM_TOOL_INIT_DISABLED = 0,
    ARM_TOOL_INIT_SERVO1_PENDING,
    ARM_TOOL_INIT_SERVO1_WAIT,
    ARM_TOOL_INIT_SERVO2_PENDING,
    ARM_TOOL_INIT_SERVO2_WAIT,
    ARM_TOOL_INIT_RETRY_WAIT,
    ARM_TOOL_INIT_DONE,
    ARM_TOOL_INIT_ERROR
} Arm_Tool_Init_State_e;

typedef struct {
    uint8_t initialized;
    uint8_t tool_ready;
    Arm_Tool_Init_State_e init_state;
    uint8_t init_step;
    uint8_t init_repeat_count;
    Arm_Command_Result_e init_result[2];
    uint8_t magnet_on;
    uint8_t servo_online[2];
    float servo_target_deg[2];
    uint16_t servo_target_pos[2];
    float servo1_compensation_target_deg;
    uint32_t servo1_slew_tick;
    uint8_t servo1_slew_active;
    uint8_t vertical_down_enabled;
    uint32_t error_code;
    uint32_t tx_count[2];
    uint32_t tx_fail_count[2];
    uint32_t last_update_tick;
    Arm_Position_s wrist_center_mm;
    Arm_Position_s tool_tip_mm;
    float tool_arm_pitch_deg;
} Arm_Tool_State_s;

extern Arm_Tool_State_s g_arm_tool_debug;

void ArmToolInit(void);
void ArmToolTask(uint32_t now_ms);
void ArmToolSetMagnet(uint8_t on);
void ArmToolStopServo1Tracking(void);
Arm_Command_Result_e ArmToolSetServo1Angle(float angle_deg);
Arm_Command_Result_e ArmToolSetServo2Angle(float angle_deg);
Arm_Command_Result_e ArmToolSetVerticalDownFromPitch(
    float small_link_pitch_deg);
uint8_t ArmToolGetTipFromWrist(const Arm_Position_s *wrist,
                               float base_yaw_deg,
                               float tool_arm_pitch_deg,
                               Arm_Position_s *tip);
uint8_t ArmToolGetWristFromTipVerticalDown(const Arm_Position_s *tip,
                                           Arm_Position_s *wrist);
const Arm_Tool_State_s *ArmToolGetState(void);
uint16_t ArmToolAngleDegToPos(uint8_t servo_id, float angle_deg);
float ArmToolServo1AngleForVerticalDown(float small_link_pitch_deg);
uint8_t ArmToolServo1AngleValid(float angle_deg);

#endif
