#ifndef ARM_USB_BRIDGE_H
#define ARM_USB_BRIDGE_H

#include "protocol.h"
#include <stdint.h>

typedef enum {
    ARM_USB_ACTION_IDLE = 0,
    ARM_USB_TARGET_LIFT,
    ARM_USB_TARGET_MOVE,
    ARM_USB_HOME_RUNNING,
    ARM_USB_TASK5_OPENING,
    ARM_USB_TASK5_DESCENDING,
    ARM_USB_TASK5_CLOSING,
    ARM_USB_TASK5_RAISING,
    ARM_USB_TASK6_DESCENDING,
    ARM_USB_TASK6_OPENING,
    ARM_USB_TASK6_RAISING,
    ARM_USB_TASK6_READYING,
    ARM_USB_STOP_CANCEL_PENDING,
    ARM_USB_STOP_HOME_RUNNING,
    ARM_USB_STOP_OPENING,
    ARM_USB_TOOL_PITCH_RUNNING,
    ARM_USB_TOOL_GRIPPER_RUNNING,
    ARM_USB_ACTION_FAILED
} Arm_Usb_Action_State_e;

typedef enum {
    ARM_USB_BUSINESS_IDLE = 0,
    ARM_USB_BUSINESS_TARGET_MOVE,
    ARM_USB_BUSINESS_HOME,
    ARM_USB_BUSINESS_TASK5_GRIP,
    ARM_USB_BUSINESS_TASK6_RELEASE,
    ARM_USB_BUSINESS_STOP,
    ARM_USB_BUSINESS_TOOL
} Arm_Usb_Business_e;

typedef struct {
    uint8_t link_online;
    uint8_t connection_ready;
    uint8_t active_task_id;
    uint8_t action_state;
    uint8_t active_business;
    uint8_t pending_task_end;
    uint8_t last_action_failed;
    uint8_t stop_task_id;
    uint8_t stop_safe_latched;
    uint8_t target_move_phase;
    uint32_t internal_arm_command_id;
    uint32_t active_tool_command_id;
    uint32_t last_tool_command_id;
    uint8_t last_tool_state;
    uint8_t last_tool_fault;
    float requested_x_mm;
    float requested_y_mm;
    float requested_pitch_deg;
    float target_x_mm;
    float target_y_mm;
    float target_z_mm;
    float target_pitch_deg;
    float current_x_mm;
    float current_y_mm;
    float current_z_mm;
    float current_pitch_deg;
    uint8_t gripper_state;
    uint16_t gripper_feedback_pos;
    uint8_t motion_state;
    uint8_t motion_fault;
    uint32_t state_tick;
    uint32_t rx_frame_count;
    uint32_t crc_fail_count;
    uint32_t duplicate_count;
    uint32_t tool_duplicate_count;
    uint32_t usb_rx_overflow_count;
    uint32_t usb_tx_fail_count;
    uint8_t usb_tx_busy;
    uint8_t usb_tx_high_queue_count;
    uint8_t usb_tx_normal_queue_count;
    uint32_t usb_tx_timeout_count;
    uint32_t usb_tx_reset_count;
} Arm_Usb_Debug_s;

typedef struct {
    uint8_t usb_task_alive;
    uint8_t connection_ready;
    uint8_t rx_packet_id;
    uint8_t rx_task_id;
    uint8_t rx_task_status;
    float rx_x_mm;
    float rx_y_mm;
    float rx_pitch_deg;
    uint8_t active_task_id;
    uint8_t active_business;
    uint8_t action_state;
    uint8_t motion_state;
    uint8_t motion_fault;
    float target_x_mm;
    float target_y_mm;
    float target_z_mm;
    float target_pitch_deg;
    float current_x_mm;
    float current_y_mm;
    float current_z_mm;
    float current_pitch_deg;
    uint8_t gripper_state;
    uint32_t task_tick;
    uint32_t rx_frame_count;
    uint32_t crc_fail_count;
    uint32_t duplicate_count;
    uint32_t ack_tx_count;
    uint32_t ack_rx_count;
    uint32_t motion_status_tx_count;
    uint32_t callback_status_tx_count;
    uint32_t tool_status_tx_count;
} Arm_Usb_Comm_Debug_s;

extern Arm_Usb_Debug_s g_arm_usb_debug;
extern Arm_Usb_Comm_Debug_s g_arm_usb_comm_debug;

void ArmUsbBridgeInit(void);
void ArmUsbBridgeTask(uint32_t now_ms);
void ArmUsbBridgeOnTaskStatus(const Packet_TaskStatus *pkt);
void ArmUsbBridgeOnTargetControl(const Packet_TargetControl *pkt);
void ArmUsbBridgeOnToolControl(const Packet_ToolControl *pkt);
uint8_t ArmUsbBridgeRequestTaskStartFromKey(uint8_t task_id);

#endif
