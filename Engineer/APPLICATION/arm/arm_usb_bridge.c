#include "arm_usb_bridge.h"

#include "arm.h"
#include "arm_config.h"
#include "arm_host.h"
#include "arm_tool.h"
#include "buzzer.h"
#include "stm32f4xx_hal.h"
#include "usb.h"

#include <math.h>
#include <string.h>

Arm_Usb_Debug_s g_arm_usb_debug;
Arm_Usb_Comm_Debug_s g_arm_usb_comm_debug;

static uint32_t next_internal_command_id = 1u;
static uint32_t active_arm_command_id;
static uint32_t active_tool_command_id;
static uint32_t last_tool_command_id;
static uint8_t last_tool_state;
static uint8_t last_tool_fault;
static uint8_t pending_tool_gripper_action;
static uint8_t active_task_id;
static uint8_t active_business;
static uint8_t pending_task_end;
static uint8_t last_action_failed;
static uint8_t stop_task_id;
static uint8_t stop_safe_latched;
static uint8_t target_move_phase;
static uint8_t last_motion_state;
static uint8_t last_motion_fault;
static float action_x_mm;
static float action_y_mm;
static float pending_target_x_mm;
static float pending_target_y_mm;
static float pending_target_pitch_deg;

static uint32_t ArmUsbNextInternalCommandId(void)
{
    if (next_internal_command_id == 0u) {
        next_internal_command_id = 1u;
    }
    return next_internal_command_id++;
}

static float ArmUsbZMapRatioForX(float x_mm)
{
    float range = ARM_USB_Z_MAP_X_MAX_MM - ARM_USB_Z_MAP_X_MIN_MM;

    if (range <= 0.000001f || x_mm <= ARM_USB_Z_MAP_X_MIN_MM) {
        return 0.0f;
    }
    if (x_mm >= ARM_USB_Z_MAP_X_MAX_MM) {
        return 1.0f;
    }
    return (x_mm - ARM_USB_Z_MAP_X_MIN_MM) / range;
}

static float ArmUsbTravelZForX(float x_mm)
{
    float ratio = ArmUsbZMapRatioForX(x_mm);

    return ARM_USB_MOVE_Z_AT_X_MIN_MM + ratio *
        (ARM_USB_MOVE_Z_AT_X_MAX_MM - ARM_USB_MOVE_Z_AT_X_MIN_MM);
}

static float ArmUsbGripperZForX(float x_mm)
{
    float ratio = ArmUsbZMapRatioForX(x_mm);

    return ARM_USB_GRIPPER_Z_AT_X_MIN_MM + ratio *
        (ARM_USB_GRIPPER_Z_AT_X_MAX_MM -
         ARM_USB_GRIPPER_Z_AT_X_MIN_MM);
}

static MotionFault ArmUsbMotionFaultFromResult(Arm_Command_Result_e result)
{
    if (result == ARM_COMMAND_PREFLIGHT_FAILED) {
        return MOTIONFAULT_IK_UNREACHABLE;
    }
    if (result == ARM_COMMAND_INVALID ||
        result == ARM_COMMAND_UNSUPPORTED ||
        result == ARM_COMMAND_DUPLICATE) {
        return MOTIONFAULT_COLLISION;
    }
    if (result == ARM_COMMAND_OK) {
        return MOTIONFAULT_NONE;
    }
    return MOTIONFAULT_TIMEOUT;
}

static ToolFault ArmUsbToolFaultFromResult(Arm_Command_Result_e result)
{
    if (result == ARM_COMMAND_PREFLIGHT_FAILED) {
        return TOOLFAULT_PITCH_LIMIT;
    }
    if (result == ARM_COMMAND_BUSY || result == ARM_COMMAND_DUPLICATE) {
        return TOOLFAULT_BUSY;
    }
    if (result == ARM_COMMAND_NOT_READY ||
        result == ARM_COMMAND_MODE_DENIED) {
        return TOOLFAULT_NOT_READY;
    }
    if (result == ARM_COMMAND_TIMEOUT) {
        return TOOLFAULT_TIMEOUT;
    }
    if (result != ARM_COMMAND_OK) {
        return TOOLFAULT_INVALID_COMMAND;
    }
    return TOOLFAULT_NONE;
}

static ToolFault ArmUsbToolFaultFromHost(const Arm_Host_Status_s *host)
{
    if (host == NULL || host->servo_online[0] == 0u ||
        host->servo_online[1] == 0u) {
        return TOOLFAULT_SERVO_OFFLINE;
    }
    if (host->gripper_state == ARM_GRIPPER_JAMMED) {
        return TOOLFAULT_JAMMED;
    }
    if (host->tool_error_code == ARM_TOOL_ERROR_SERVO_TIMEOUT) {
        return TOOLFAULT_TIMEOUT;
    }
    if (host->tool_error_code != ARM_TOOL_ERROR_NONE ||
        host->gripper_state == ARM_GRIPPER_FAULT) {
        return TOOLFAULT_NOT_READY;
    }
    return TOOLFAULT_NONE;
}

static void ArmUsbSetState(Arm_Usb_Action_State_e state, uint32_t now_ms)
{
    g_arm_usb_debug.action_state = (uint8_t)state;
    g_arm_usb_debug.state_tick = now_ms;
}

static void ArmUsbSetBusiness(Arm_Usb_Business_e business)
{
    active_business = (uint8_t)business;
    g_arm_usb_debug.active_business = active_business;
}

static void ArmUsbUpdateCurrent(const Arm_Host_Status_s *host)
{
    if (host == NULL) {
        return;
    }
    g_arm_usb_debug.current_x_mm = host->tool_tip_mm.x_mm;
    g_arm_usb_debug.current_y_mm = host->tool_tip_mm.y_mm;
    g_arm_usb_debug.current_z_mm = host->tool_tip_mm.z_mm;
    g_arm_usb_debug.current_pitch_deg = host->tool_pitch_feedback_deg;
    g_arm_usb_debug.gripper_state = host->gripper_state;
    g_arm_usb_debug.gripper_feedback_pos = host->gripper_feedback_pos;
    g_arm_usb_comm_debug.current_x_mm = host->tool_tip_mm.x_mm;
    g_arm_usb_comm_debug.current_y_mm = host->tool_tip_mm.y_mm;
    g_arm_usb_comm_debug.current_z_mm = host->tool_tip_mm.z_mm;
    g_arm_usb_comm_debug.current_pitch_deg =
        host->tool_pitch_feedback_deg;
    g_arm_usb_comm_debug.gripper_state = host->gripper_state;
}

static void ArmUsbSendCallback(uint8_t callback_id, uint8_t status)
{
    Packet_CallbackStatus packet;

    packet.callback_id = callback_id;
    packet.callback_status = status;
    (void)protocol_send_callback_status(&packet);
}

static void ArmUsbSendMotion(MotionState state, MotionFault fault)
{
    Packet_MotionStatus packet;
    Arm_Host_Status_s host;

    memset(&packet, 0, sizeof(packet));
    packet.state = (uint8_t)state;
    packet.fault = (uint8_t)fault;
    if (ArmGetHostStatus(&host) != 0u) {
        ArmUsbUpdateCurrent(&host);
        packet.x_mm = host.tool_tip_mm.x_mm;
        packet.y_mm = host.tool_tip_mm.y_mm;
        packet.pitch_deg = host.tool_pitch_feedback_deg;
    }
    if ((uint8_t)state != last_motion_state ||
        (uint8_t)fault != last_motion_fault) {
        (void)protocol_send_motion_status(&packet);
        last_motion_state = (uint8_t)state;
        last_motion_fault = (uint8_t)fault;
    }
    g_arm_usb_debug.motion_state = (uint8_t)state;
    g_arm_usb_debug.motion_fault = (uint8_t)fault;
}

static void ArmUsbSendTool(uint32_t command_id, ToolState state,
                           ToolFault fault)
{
    Packet_ToolStatus packet;
    Arm_Host_Status_s host;

    memset(&packet, 0, sizeof(packet));
    packet.command_id = command_id;
    packet.state = (uint8_t)state;
    packet.fault = (uint8_t)fault;
    if (ArmGetHostStatus(&host) != 0u) {
        ArmUsbUpdateCurrent(&host);
        packet.pitch_deg = host.tool_pitch_feedback_deg;
        packet.gripper_state = host.gripper_state;
    }
    (void)protocol_send_tool_status(&packet);
    if (last_tool_command_id == 0u ||
        (int32_t)(command_id - last_tool_command_id) >= 0) {
        last_tool_command_id = command_id;
        last_tool_state = (uint8_t)state;
        last_tool_fault = (uint8_t)fault;
        g_arm_usb_debug.last_tool_command_id = command_id;
        g_arm_usb_debug.last_tool_state = (uint8_t)state;
        g_arm_usb_debug.last_tool_fault = (uint8_t)fault;
    }
}

static Arm_Command_Result_e ArmUsbSubmitMove(float x_mm, float y_mm,
                                             float z_mm,
                                             float speed_mm_s,
                                             uint8_t pitch_valid,
                                             float pitch_deg)
{
    Arm_Command_s command;

    memset(&command, 0, sizeof(command));
    active_arm_command_id = ArmUsbNextInternalCommandId();
    command.command_id = active_arm_command_id;
    command.type = ARM_COMMAND_TYPE_CARTESIAN;
    command.payload.cartesian.control_point = ARM_CONTROL_POINT_TOOL_TIP;
    command.payload.cartesian.move_type = ARM_MOVE_LINEAR;
    command.payload.cartesian.target_mm.x_mm = x_mm;
    command.payload.cartesian.target_mm.y_mm = y_mm;
    command.payload.cartesian.target_mm.z_mm = z_mm;
    command.payload.cartesian.max_speed_mm_s = speed_mm_s;
    command.payload.cartesian.tool_pitch_valid = pitch_valid;
    command.payload.cartesian.tool_pitch_deg = pitch_deg;
    command.payload.cartesian.tool_yaw_valid = 0u;
    g_arm_usb_debug.internal_arm_command_id = active_arm_command_id;
    g_arm_usb_debug.target_x_mm = x_mm;
    g_arm_usb_debug.target_y_mm = y_mm;
    g_arm_usb_debug.target_z_mm = z_mm;
    if (pitch_valid != 0u) {
        g_arm_usb_debug.target_pitch_deg = pitch_deg;
    }
    return ArmSubmitCommand(&command);
}

static Arm_Command_Result_e ArmUsbSubmitToolAction(
    Arm_Tool_Action_e action, float pitch_deg)
{
    Arm_Command_s command;

    memset(&command, 0, sizeof(command));
    active_arm_command_id = ArmUsbNextInternalCommandId();
    command.command_id = active_arm_command_id;
    command.type = ARM_COMMAND_TYPE_TOOL;
    command.payload.tool.action = action;
    command.payload.tool.pitch_deg = pitch_deg;
    g_arm_usb_debug.internal_arm_command_id = active_arm_command_id;
    return ArmSubmitCommand(&command);
}

static Arm_Command_Result_e ArmUsbSubmitCancel(void)
{
    Arm_Command_s command;

    memset(&command, 0, sizeof(command));
    active_arm_command_id = ArmUsbNextInternalCommandId();
    command.command_id = active_arm_command_id;
    command.type = ARM_COMMAND_TYPE_CANCEL_MOTION;
    g_arm_usb_debug.internal_arm_command_id = active_arm_command_id;
    return ArmSubmitCommand(&command);
}

static uint8_t ArmUsbCommandCompleted(const Arm_Host_Status_s *host)
{
    return host != NULL && host->last_command_id == active_arm_command_id &&
           host->last_command_state == ARM_COMMAND_STATE_COMPLETED;
}

static uint8_t ArmUsbCommandFailed(const Arm_Host_Status_s *host)
{
    return host != NULL && host->last_command_id == active_arm_command_id &&
           (host->last_command_state == ARM_COMMAND_STATE_REJECTED ||
            host->last_command_state == ARM_COMMAND_STATE_CANCELLED ||
            host->last_command_state == ARM_COMMAND_STATE_FAULTED);
}

static void ArmUsbClearBusiness(uint32_t now_ms)
{
    active_arm_command_id = 0u;
    target_move_phase = 0u;
    g_arm_usb_debug.internal_arm_command_id = 0u;
    g_arm_usb_debug.target_move_phase = 0u;
    ArmUsbSetBusiness(ARM_USB_BUSINESS_IDLE);
    ArmUsbSetState(ARM_USB_ACTION_IDLE, now_ms);
}

static void ArmUsbFailBusiness(MotionFault fault, uint32_t now_ms)
{
    if (active_business == ARM_USB_BUSINESS_TARGET_MOVE) {
        ArmUsbSendMotion(MOTIONSTATE_FAILED, fault);
    } else if (active_business == ARM_USB_BUSINESS_TOOL) {
        Arm_Host_Status_s host;
        ToolFault tool_fault = TOOLFAULT_NOT_READY;

        if (ArmGetHostStatus(&host) != 0u) {
            tool_fault = ArmUsbToolFaultFromHost(&host);
        }
        ArmUsbSendTool(active_tool_command_id, TOOLSTATE_FAILED,
                       tool_fault);
    } else {
        uint8_t callback_id = active_business == ARM_USB_BUSINESS_HOME ?
            0u : (active_business == ARM_USB_BUSINESS_TASK5_GRIP ? 5u :
                  (active_business == ARM_USB_BUSINESS_TASK6_RELEASE ?
                   6u : stop_task_id));
        ArmUsbSendCallback(callback_id, STATUS_FAULT_RETRY);
    }
    last_action_failed = 1u;
    ArmUsbClearBusiness(now_ms);
}

static uint8_t ArmUsbHostReadyAndIdle(const Arm_Host_Status_s *host)
{
    return host != NULL && host->ready != 0u && host->busy == 0u &&
           host->fault_code == ARM_FAULT_NONE;
}

void ArmUsbBridgeInit(void)
{
    memset(&g_arm_usb_debug, 0, sizeof(g_arm_usb_debug));
    memset(&g_arm_usb_comm_debug, 0, sizeof(g_arm_usb_comm_debug));
    next_internal_command_id = 1u;
    active_arm_command_id = 0u;
    active_tool_command_id = 0u;
    last_tool_command_id = 0u;
    last_tool_state = 0u;
    last_tool_fault = 0u;
    pending_tool_gripper_action = TOOL_GRIPPER_HOLD;
    active_task_id = 0u;
    active_business = ARM_USB_BUSINESS_IDLE;
    pending_task_end = 0u;
    last_action_failed = 0u;
    stop_task_id = 0u;
    stop_safe_latched = 0u;
    target_move_phase = 0u;
    last_motion_state = 0u;
    last_motion_fault = 0u;
    action_x_mm = 0.0f;
    action_y_mm = 0.0f;
    pending_target_x_mm = 0.0f;
    pending_target_y_mm = 0.0f;
    pending_target_pitch_deg = 0.0f;
    ArmUsbSetState(ARM_USB_ACTION_IDLE, HAL_GetTick());
}

uint8_t ArmUsbBridgeRequestTaskStartFromKey(uint8_t task_id)
{
    Packet_CallbackStatus packet;

    if (task_id < 1u || task_id > 4u) {
        return 0u;
    }
    packet.callback_id = task_id;
    packet.callback_status = STATUS_START;
    return protocol_send_callback_status(&packet) != 0 ? 1u : 0u;
}

void ArmUsbBridgeOnTaskStatus(const Packet_TaskStatus *pkt)
{
    Arm_Host_Status_s host;
    Arm_Command_Result_e result;
    uint32_t now_ms = HAL_GetTick();

    if (pkt == NULL) {
        return;
    }
    g_arm_usb_comm_debug.rx_packet_id = PACKET_ID_TASKSTATUS;
    g_arm_usb_comm_debug.rx_task_id = pkt->task_id;
    g_arm_usb_comm_debug.rx_task_status = pkt->task_status;

    if (pkt->task_status == STATUS_STOP) {
        stop_task_id = pkt->task_id;
        pending_task_end = 0u;
        if (stop_safe_latched != 0u &&
            active_business == ARM_USB_BUSINESS_IDLE) {
            ArmUsbSendCallback(stop_task_id, STATUS_STOP);
            return;
        }
        if (active_business == ARM_USB_BUSINESS_STOP) {
            return;
        }
        if (ArmGetHostStatus(&host) == 0u ||
            host.fault_code != ARM_FAULT_NONE) {
            ArmUsbSendCallback(stop_task_id, STATUS_FAULT_RETRY);
            return;
        }
        stop_safe_latched = 0u;
        ArmUsbSetBusiness(ARM_USB_BUSINESS_STOP);
        result = host.busy != 0u ? ArmUsbSubmitCancel() :
            ArmUsbSubmitMove(ARM_USB_HOME_X_MM, ARM_USB_HOME_Y_MM,
                             ARM_USB_HOME_Z_MM, ARM_USB_HOME_SPEED_MM_S,
                             0u, 0.0f);
        if (result != ARM_COMMAND_OK) {
            ArmUsbFailBusiness(ArmUsbMotionFaultFromResult(result), now_ms);
        } else {
            ArmUsbSetState(host.busy != 0u ?
                ARM_USB_STOP_CANCEL_PENDING :
                ARM_USB_STOP_HOME_RUNNING, now_ms);
        }
        return;
    }

    if (pkt->task_status == STATUS_FAULT_RETRY) {
        if (ArmGetHostStatus(&host) != 0u && host.busy != 0u) {
            (void)ArmUsbSubmitCancel();
            ArmUsbSetState(ARM_USB_ACTION_FAILED, now_ms);
        } else {
            ArmUsbClearBusiness(now_ms);
        }
        return;
    }

    if (pkt->task_status == STATUS_END) {
        if (pkt->task_id >= 1u && pkt->task_id <= 4u) {
            active_task_id = pkt->task_id;
            pending_task_end = 1u;
            last_action_failed = 0u;
        }
        return;
    }
    if (pkt->task_status != STATUS_START) {
        return;
    }

    active_task_id = pkt->task_id;
    pending_task_end = 0u;
    last_action_failed = 0u;
    if (pkt->task_id >= 1u && pkt->task_id <= 4u) {
        return;
    }
    if (active_business != ARM_USB_BUSINESS_IDLE ||
        ArmGetHostStatus(&host) == 0u ||
        !ArmUsbHostReadyAndIdle(&host)) {
        ArmUsbSendCallback(pkt->task_id, STATUS_FAULT_RETRY);
        return;
    }

    action_x_mm = host.tool_tip_mm.x_mm;
    action_y_mm = host.tool_tip_mm.y_mm;
    if (pkt->task_id == 0u) {
        ArmUsbSetBusiness(ARM_USB_BUSINESS_HOME);
        result = ArmUsbSubmitMove(ARM_USB_HOME_X_MM, ARM_USB_HOME_Y_MM,
            ARM_USB_HOME_Z_MM, ARM_USB_HOME_SPEED_MM_S, 0u, 0.0f);
        ArmUsbSetState(ARM_USB_HOME_RUNNING, now_ms);
    } else if (pkt->task_id == 5u) {
        ArmUsbSetBusiness(ARM_USB_BUSINESS_TASK5_GRIP);
        result = ArmUsbSubmitToolAction(ARM_TOOL_ACTION_GRIPPER_OPEN, 0.0f);
        ArmUsbSetState(ARM_USB_TASK5_OPENING, now_ms);
    } else if (pkt->task_id == 6u) {
        ArmUsbSetBusiness(ARM_USB_BUSINESS_TASK6_RELEASE);
        result = ArmUsbSubmitMove(action_x_mm, action_y_mm,
            ArmUsbGripperZForX(action_x_mm), ARM_USB_GRIPPER_Z_SPEED_MM_S,
            0u, 0.0f);
        ArmUsbSetState(ARM_USB_TASK6_DESCENDING, now_ms);
    } else {
        result = ARM_COMMAND_INVALID;
    }
    if (result != ARM_COMMAND_OK) {
        ArmUsbFailBusiness(ArmUsbMotionFaultFromResult(result), now_ms);
    }
}

void ArmUsbBridgeOnTargetControl(const Packet_TargetControl *pkt)
{
    Arm_Host_Status_s host;
    Arm_Command_Result_e result;
    uint32_t now_ms = HAL_GetTick();

    if (pkt == NULL) {
        return;
    }
    last_motion_state = 0u;
    last_motion_fault = 0u;
    g_arm_usb_comm_debug.rx_packet_id = PACKET_ID_TARGETCONTROL;
    g_arm_usb_debug.requested_x_mm = pkt->x_mm;
    g_arm_usb_debug.requested_y_mm = pkt->y_mm;
    g_arm_usb_debug.requested_pitch_deg = pkt->pitch_deg;
    if (!isfinite(pkt->x_mm) || !isfinite(pkt->y_mm) ||
        !isfinite(pkt->pitch_deg) ||
        pkt->pitch_deg < ARM_USB_TOOL_PITCH_MIN_DEG ||
        pkt->pitch_deg > ARM_USB_TOOL_PITCH_MAX_DEG ||
        active_business != ARM_USB_BUSINESS_IDLE ||
        ArmGetHostStatus(&host) == 0u ||
        !ArmUsbHostReadyAndIdle(&host)) {
        ArmUsbSendMotion(MOTIONSTATE_FAILED, MOTIONFAULT_COLLISION);
        return;
    }

    pending_target_x_mm = pkt->x_mm;
    pending_target_y_mm = pkt->y_mm;
    pending_target_pitch_deg = pkt->pitch_deg;
    stop_safe_latched = 0u;
    ArmUsbSetBusiness(ARM_USB_BUSINESS_TARGET_MOVE);
    ArmUsbSendMotion(MOTIONSTATE_ACCEPTED, MOTIONFAULT_NONE);
    ArmUsbSendMotion(MOTIONSTATE_RUNNING, MOTIONFAULT_NONE);
    if (host.gripper_state == ARM_GRIPPER_HELD_CONTACT ||
        host.gripper_state == ARM_GRIPPER_CLOSED_EMPTY) {
        target_move_phase = 1u;
        result = ArmUsbSubmitMove(host.tool_tip_mm.x_mm,
            host.tool_tip_mm.y_mm, ArmUsbTravelZForX(host.tool_tip_mm.x_mm),
            ARM_USB_GRIPPER_Z_SPEED_MM_S, 0u, 0.0f);
        ArmUsbSetState(ARM_USB_TARGET_LIFT, now_ms);
    } else {
        target_move_phase = 2u;
        result = ArmUsbSubmitMove(pkt->x_mm, pkt->y_mm,
            ArmUsbTravelZForX(pkt->x_mm), ARM_USB_MOVE_SPEED_MM_S,
            1u, pkt->pitch_deg);
        ArmUsbSetState(ARM_USB_TARGET_MOVE, now_ms);
    }
    g_arm_usb_debug.target_move_phase = target_move_phase;
    if (result != ARM_COMMAND_OK) {
        ArmUsbFailBusiness(ArmUsbMotionFaultFromResult(result), now_ms);
    }
}

void ArmUsbBridgeOnToolControl(const Packet_ToolControl *pkt)
{
    Arm_Command_Result_e result;
    ToolFault fault;
    uint32_t now_ms = HAL_GetTick();

    if (pkt == NULL) {
        return;
    }
    if (pkt->command_id == last_tool_command_id &&
        last_tool_command_id != 0u) {
        g_arm_usb_debug.tool_duplicate_count++;
        ArmUsbSendTool(pkt->command_id, (ToolState)last_tool_state,
                       (ToolFault)last_tool_fault);
        return;
    }
    if (pkt->command_id == 0u ||
        (last_tool_command_id != 0u &&
         (int32_t)(pkt->command_id - last_tool_command_id) <= 0) ||
        !isfinite(pkt->pitch_deg) ||
        pkt->pitch_deg < ARM_USB_TOOL_PITCH_MIN_DEG ||
        pkt->pitch_deg > ARM_USB_TOOL_PITCH_MAX_DEG ||
        pkt->gripper_action > TOOL_GRIPPER_CLOSE) {
        ArmUsbSendTool(pkt->command_id, TOOLSTATE_FAILED,
                       TOOLFAULT_INVALID_COMMAND);
        return;
    }
    if (active_business != ARM_USB_BUSINESS_IDLE) {
        ArmUsbSendTool(pkt->command_id, TOOLSTATE_FAILED, TOOLFAULT_BUSY);
        return;
    }

    active_tool_command_id = pkt->command_id;
    pending_tool_gripper_action = pkt->gripper_action;
    ArmUsbSetBusiness(ARM_USB_BUSINESS_TOOL);
    ArmUsbSendTool(pkt->command_id, TOOLSTATE_ACCEPTED, TOOLFAULT_NONE);
    result = ArmUsbSubmitToolAction(ARM_TOOL_ACTION_SET_PITCH,
                                    pkt->pitch_deg);
    if (result == ARM_COMMAND_OK) {
        ArmUsbSendTool(pkt->command_id, TOOLSTATE_RUNNING, TOOLFAULT_NONE);
        ArmUsbSetState(ARM_USB_TOOL_PITCH_RUNNING, now_ms);
        return;
    }
    fault = ArmUsbToolFaultFromResult(result);
    ArmUsbSendTool(pkt->command_id, TOOLSTATE_FAILED, fault);
    ArmUsbClearBusiness(now_ms);
}

static void ArmUsbUpdateDebug(uint32_t now_ms,
                              const Protocol_Debug_s *protocol_debug)
{
    g_arm_usb_debug.connection_ready = protocol_connection_ready();
    g_arm_usb_debug.link_online = protocol_link_is_online();
    g_arm_usb_debug.active_task_id = active_task_id;
    g_arm_usb_debug.pending_task_end = pending_task_end;
    g_arm_usb_debug.last_action_failed = last_action_failed;
    g_arm_usb_debug.stop_task_id = stop_task_id;
    g_arm_usb_debug.stop_safe_latched = stop_safe_latched;
    g_arm_usb_debug.active_tool_command_id = active_tool_command_id;
    g_arm_usb_debug.rx_frame_count = protocol_debug->rx_frame_count;
    g_arm_usb_debug.crc_fail_count = protocol_debug->crc_fail_count;
    g_arm_usb_debug.duplicate_count = protocol_debug->duplicate_count;
    g_arm_usb_debug.usb_rx_overflow_count = g_usb_rx_overflow_count;
    g_arm_usb_debug.usb_tx_fail_count = g_usb_tx_fail_count;
    g_arm_usb_debug.usb_tx_busy = g_usb_tx_debug.busy;
    g_arm_usb_debug.usb_tx_high_queue_count =
        g_usb_tx_debug.high_queue_count;
    g_arm_usb_debug.usb_tx_normal_queue_count =
        g_usb_tx_debug.normal_queue_count;
    g_arm_usb_debug.usb_tx_timeout_count = g_usb_tx_debug.timeout_count;
    g_arm_usb_debug.usb_tx_reset_count = g_usb_tx_debug.reset_count;

    g_arm_usb_comm_debug.usb_task_alive = 1u;
    g_arm_usb_comm_debug.connection_ready =
        g_arm_usb_debug.connection_ready;
    g_arm_usb_comm_debug.rx_task_id = protocol_debug->last_task_id;
    g_arm_usb_comm_debug.rx_task_status = protocol_debug->last_task_status;
    g_arm_usb_comm_debug.rx_x_mm = protocol_debug->last_target_x_mm;
    g_arm_usb_comm_debug.rx_y_mm = protocol_debug->last_target_y_mm;
    g_arm_usb_comm_debug.rx_pitch_deg =
        protocol_debug->last_target_pitch_deg;
    g_arm_usb_comm_debug.active_task_id = active_task_id;
    g_arm_usb_comm_debug.active_business = active_business;
    g_arm_usb_comm_debug.action_state = g_arm_usb_debug.action_state;
    g_arm_usb_comm_debug.motion_state = g_arm_usb_debug.motion_state;
    g_arm_usb_comm_debug.motion_fault = g_arm_usb_debug.motion_fault;
    g_arm_usb_comm_debug.target_x_mm = g_arm_usb_debug.target_x_mm;
    g_arm_usb_comm_debug.target_y_mm = g_arm_usb_debug.target_y_mm;
    g_arm_usb_comm_debug.target_z_mm = g_arm_usb_debug.target_z_mm;
    g_arm_usb_comm_debug.target_pitch_deg =
        g_arm_usb_debug.target_pitch_deg;
    g_arm_usb_comm_debug.task_tick = now_ms;
    g_arm_usb_comm_debug.rx_frame_count = protocol_debug->rx_frame_count;
    g_arm_usb_comm_debug.crc_fail_count = protocol_debug->crc_fail_count;
    g_arm_usb_comm_debug.duplicate_count = protocol_debug->duplicate_count;
    g_arm_usb_comm_debug.ack_tx_count = protocol_debug->ack_tx_count;
    g_arm_usb_comm_debug.ack_rx_count = protocol_debug->ack_rx_count;
    g_arm_usb_comm_debug.motion_status_tx_count =
        protocol_debug->motion_status_tx_count;
    g_arm_usb_comm_debug.callback_status_tx_count =
        protocol_debug->callback_status_tx_count;
    g_arm_usb_comm_debug.tool_status_tx_count =
        protocol_debug->tool_status_tx_count;
}

void ArmUsbBridgeTask(uint32_t now_ms)
{
    Arm_Host_Status_s host;
    Arm_Command_Result_e result;
    const Protocol_Debug_s *protocol_debug = protocol_get_debug();

    ArmUsbUpdateDebug(now_ms, protocol_debug);
    if (ArmGetHostStatus(&host) == 0u) {
        return;
    }
    ArmUsbUpdateCurrent(&host);

    if (host.fault_code != ARM_FAULT_NONE &&
        active_business != ARM_USB_BUSINESS_IDLE) {
        ArmUsbFailBusiness(host.fault_code == ARM_FAULT_EMERGENCY_STOP ?
            MOTIONFAULT_ESTOP : MOTIONFAULT_TIMEOUT, now_ms);
        return;
    }

    switch ((Arm_Usb_Action_State_e)g_arm_usb_debug.action_state) {
        case ARM_USB_TARGET_LIFT:
            if (ArmUsbCommandCompleted(&host)) {
                target_move_phase = 2u;
                result = ArmUsbSubmitMove(pending_target_x_mm,
                    pending_target_y_mm,
                    ArmUsbTravelZForX(pending_target_x_mm),
                    ARM_USB_MOVE_SPEED_MM_S, 1u,
                    pending_target_pitch_deg);
                if (result == ARM_COMMAND_OK) {
                    ArmUsbSetState(ARM_USB_TARGET_MOVE, now_ms);
                } else {
                    ArmUsbFailBusiness(
                        ArmUsbMotionFaultFromResult(result), now_ms);
                }
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbFailBusiness(ArmUsbMotionFaultFromResult(
                    host.last_command_result), now_ms);
            }
            break;

        case ARM_USB_TARGET_MOVE:
            if (ArmUsbCommandCompleted(&host)) {
                ArmUsbSendMotion(MOTIONSTATE_COMPLETED, MOTIONFAULT_NONE);
                last_action_failed = 0u;
                ArmUsbClearBusiness(now_ms);
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbFailBusiness(ArmUsbMotionFaultFromResult(
                    host.last_command_result), now_ms);
            }
            break;

        case ARM_USB_HOME_RUNNING:
            if (ArmUsbCommandCompleted(&host)) {
                ArmUsbSendCallback(0u, STATUS_END);
                ArmUsbClearBusiness(now_ms);
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbFailBusiness(ArmUsbMotionFaultFromResult(
                    host.last_command_result), now_ms);
            }
            break;

        case ARM_USB_TASK5_OPENING:
            if (ArmUsbCommandCompleted(&host)) {
                result = ArmUsbSubmitMove(action_x_mm, action_y_mm,
                    ArmUsbGripperZForX(action_x_mm),
                    ARM_USB_GRIPPER_Z_SPEED_MM_S, 0u, 0.0f);
                if (result == ARM_COMMAND_OK) {
                    ArmUsbSetState(ARM_USB_TASK5_DESCENDING, now_ms);
                } else {
                    ArmUsbFailBusiness(
                        ArmUsbMotionFaultFromResult(result), now_ms);
                }
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbFailBusiness(MOTIONFAULT_TIMEOUT, now_ms);
            }
            break;

        case ARM_USB_TASK5_DESCENDING:
            if (ArmUsbCommandCompleted(&host)) {
                result = ArmUsbSubmitToolAction(
                    ARM_TOOL_ACTION_GRIPPER_CLOSE, 0.0f);
                if (result == ARM_COMMAND_OK) {
                    ArmUsbSetState(ARM_USB_TASK5_CLOSING, now_ms);
                } else {
                    ArmUsbFailBusiness(
                        ArmUsbMotionFaultFromResult(result), now_ms);
                }
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbFailBusiness(MOTIONFAULT_IK_UNREACHABLE, now_ms);
            }
            break;

        case ARM_USB_TASK5_CLOSING:
            if (ArmUsbCommandCompleted(&host)) {
                result = ArmUsbSubmitMove(action_x_mm, action_y_mm,
                    ArmUsbTravelZForX(action_x_mm),
                    ARM_USB_GRIPPER_Z_SPEED_MM_S, 0u, 0.0f);
                if (result == ARM_COMMAND_OK) {
                    ArmUsbSetState(ARM_USB_TASK5_RAISING, now_ms);
                } else {
                    ArmUsbFailBusiness(
                        ArmUsbMotionFaultFromResult(result), now_ms);
                }
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbFailBusiness(MOTIONFAULT_TIMEOUT, now_ms);
            }
            break;

        case ARM_USB_TASK5_RAISING:
            if (ArmUsbCommandCompleted(&host)) {
                ArmUsbSendCallback(5u, STATUS_END);
                ArmUsbClearBusiness(now_ms);
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbFailBusiness(MOTIONFAULT_TIMEOUT, now_ms);
            }
            break;

        case ARM_USB_TASK6_DESCENDING:
            if (ArmUsbCommandCompleted(&host)) {
                result = ArmUsbSubmitToolAction(
                    ARM_TOOL_ACTION_GRIPPER_OPEN, 0.0f);
                if (result == ARM_COMMAND_OK) {
                    ArmUsbSetState(ARM_USB_TASK6_OPENING, now_ms);
                } else {
                    ArmUsbFailBusiness(MOTIONFAULT_TIMEOUT, now_ms);
                }
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbFailBusiness(MOTIONFAULT_IK_UNREACHABLE, now_ms);
            }
            break;

        case ARM_USB_TASK6_OPENING:
            if (ArmUsbCommandCompleted(&host)) {
                result = ArmUsbSubmitMove(action_x_mm, action_y_mm,
                    ArmUsbTravelZForX(action_x_mm),
                    ARM_USB_GRIPPER_Z_SPEED_MM_S, 0u, 0.0f);
                if (result == ARM_COMMAND_OK) {
                    ArmUsbSetState(ARM_USB_TASK6_RAISING, now_ms);
                } else {
                    ArmUsbFailBusiness(MOTIONFAULT_TIMEOUT, now_ms);
                }
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbFailBusiness(MOTIONFAULT_TIMEOUT, now_ms);
            }
            break;

        case ARM_USB_TASK6_RAISING:
            if (ArmUsbCommandCompleted(&host)) {
                result = ArmUsbSubmitToolAction(
                    ARM_TOOL_ACTION_GRIPPER_READY, 0.0f);
                if (result == ARM_COMMAND_OK) {
                    ArmUsbSetState(ARM_USB_TASK6_READYING, now_ms);
                } else {
                    ArmUsbFailBusiness(MOTIONFAULT_TIMEOUT, now_ms);
                }
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbFailBusiness(MOTIONFAULT_TIMEOUT, now_ms);
            }
            break;

        case ARM_USB_TASK6_READYING:
            if (ArmUsbCommandCompleted(&host)) {
                ArmUsbSendCallback(6u, STATUS_END);
                ArmUsbClearBusiness(now_ms);
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbFailBusiness(MOTIONFAULT_TIMEOUT, now_ms);
            }
            break;

        case ARM_USB_STOP_CANCEL_PENDING:
            if (ArmUsbCommandCompleted(&host)) {
                result = ArmUsbSubmitMove(ARM_USB_HOME_X_MM,
                    ARM_USB_HOME_Y_MM, ARM_USB_HOME_Z_MM,
                    ARM_USB_HOME_SPEED_MM_S, 0u, 0.0f);
                if (result == ARM_COMMAND_OK) {
                    ArmUsbSetState(ARM_USB_STOP_HOME_RUNNING, now_ms);
                } else {
                    ArmUsbFailBusiness(MOTIONFAULT_TIMEOUT, now_ms);
                }
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbFailBusiness(MOTIONFAULT_TIMEOUT, now_ms);
            }
            break;

        case ARM_USB_STOP_HOME_RUNNING:
            if (ArmUsbCommandCompleted(&host)) {
                result = ArmUsbSubmitToolAction(
                    ARM_TOOL_ACTION_GRIPPER_OPEN, 0.0f);
                if (result == ARM_COMMAND_OK) {
                    ArmUsbSetState(ARM_USB_STOP_OPENING, now_ms);
                } else {
                    ArmUsbFailBusiness(MOTIONFAULT_TIMEOUT, now_ms);
                }
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbFailBusiness(MOTIONFAULT_TIMEOUT, now_ms);
            }
            break;

        case ARM_USB_STOP_OPENING:
            if (ArmUsbCommandCompleted(&host)) {
                ArmUsbSendCallback(stop_task_id, STATUS_STOP);
                active_task_id = 0u;
                stop_safe_latched = 1u;
                ArmUsbClearBusiness(now_ms);
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbFailBusiness(MOTIONFAULT_TIMEOUT, now_ms);
            }
            break;

        case ARM_USB_TOOL_PITCH_RUNNING:
            if (ArmUsbCommandCompleted(&host)) {
                if (pending_tool_gripper_action == TOOL_GRIPPER_HOLD) {
                    ArmUsbSendTool(active_tool_command_id,
                        TOOLSTATE_COMPLETED, TOOLFAULT_NONE);
                    ArmUsbClearBusiness(now_ms);
                } else {
                    Arm_Tool_Action_e action =
                        pending_tool_gripper_action == TOOL_GRIPPER_OPEN ?
                        ARM_TOOL_ACTION_GRIPPER_OPEN :
                        ARM_TOOL_ACTION_GRIPPER_CLOSE;

                    result = ArmUsbSubmitToolAction(action, 0.0f);
                    if (result == ARM_COMMAND_OK) {
                        ArmUsbSetState(ARM_USB_TOOL_GRIPPER_RUNNING,
                                       now_ms);
                    } else {
                        ArmUsbSendTool(active_tool_command_id,
                            TOOLSTATE_FAILED,
                            ArmUsbToolFaultFromResult(result));
                        ArmUsbClearBusiness(now_ms);
                    }
                }
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbSendTool(active_tool_command_id, TOOLSTATE_FAILED,
                    ArmUsbToolFaultFromResult(host.last_command_result));
                ArmUsbClearBusiness(now_ms);
            }
            break;

        case ARM_USB_TOOL_GRIPPER_RUNNING:
            if (ArmUsbCommandCompleted(&host)) {
                ArmUsbSendTool(active_tool_command_id,
                    TOOLSTATE_COMPLETED, TOOLFAULT_NONE);
                ArmUsbClearBusiness(now_ms);
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbSendTool(active_tool_command_id, TOOLSTATE_FAILED,
                    ArmUsbToolFaultFromHost(&host));
                ArmUsbClearBusiness(now_ms);
            }
            break;

        case ARM_USB_ACTION_FAILED:
            if (host.busy == 0u) {
                ArmUsbClearBusiness(now_ms);
            }
            break;

        case ARM_USB_ACTION_IDLE:
        default:
            if (pending_task_end != 0u && host.busy == 0u &&
                host.fault_code == ARM_FAULT_NONE &&
                last_action_failed == 0u) {
                pending_task_end = 0u;
                (void)BuzzerStart(3000u);
            }
            break;
    }
}

void on_receive_TaskStatus(const Packet_TaskStatus *pkt)
{
    ArmUsbBridgeOnTaskStatus(pkt);
}

void on_receive_TargetControl(const Packet_TargetControl *pkt)
{
    ArmUsbBridgeOnTargetControl(pkt);
}

void on_receive_ToolControl(const Packet_ToolControl *pkt)
{
    ArmUsbBridgeOnToolControl(pkt);
}
