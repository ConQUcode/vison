#include "arm_usb_bridge.h"

#include "arm_config.h"
#include "arm.h"
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
static uint8_t active_task_id;
static uint8_t active_business;
static uint8_t last_motion_state;
static uint8_t last_motion_fault;
static uint8_t pending_task_end;
static uint8_t last_action_failed;
static float action_x_mm;
static float action_y_mm;
static float active_target_x_mm;
static float active_target_y_mm;
static float active_target_z_mm;
static float active_target_yaw_deg;
static float pending_target_x_mm;
static float pending_target_y_mm;
static float pending_target_yaw_deg;
static uint8_t target_move_phase;
static uint8_t target_yaw_pending;
static uint8_t target_yaw_waiting;
static uint32_t target_yaw_complete_tick;
static uint8_t magnet_action_done;
static uint8_t magnet_off_reset_done;
static uint32_t magnet_off_reset_tick;
static uint32_t state_tick;

static float ArmUsbServo2MaxYawAbsDeg(void)
{
    return fmaxf(fabsf(ARM_USB_YAW_MIN_DEG),
                 fabsf(ARM_USB_YAW_MAX_DEG));
}

static float ArmUsbServo2HalfRangePos(void)
{
    return ((float)ARM_TOOL_SERVO2_POS_MAX -
            (float)ARM_TOOL_SERVO2_POS_MIN);
}

static uint16_t ArmUsbYawToServo2Position(float yaw_deg)
{
    float max_yaw_deg = ArmUsbServo2MaxYawAbsDeg();
    float half_range_pos = ArmUsbServo2HalfRangePos();
    float pos_f;

    if (!isfinite(yaw_deg) ||
        max_yaw_deg <= 0.000001f ||
        half_range_pos <= 0.000001f) {
        return ARM_TOOL_SERVO2_NEUTRAL_POS;
    }
    pos_f = (float)ARM_TOOL_SERVO2_NEUTRAL_POS +
        ARM_TOOL_SERVO2_YAW_DIRECTION * yaw_deg * half_range_pos /
        ARM_TOOL_SERVO2_RANGE_DEG;
    if (pos_f < (float)ARM_TOOL_SERVO2_POS_MIN) {
        pos_f = (float)ARM_TOOL_SERVO2_POS_MIN;
    }
    if (pos_f > (float)ARM_TOOL_SERVO2_POS_MAX) {
        pos_f = (float)ARM_TOOL_SERVO2_POS_MAX;
    }
    return (uint16_t)(pos_f + 0.5f);
}

static uint8_t ArmUsbFinite3(float a, float b, float c)
{
    return isfinite(a) && isfinite(b) && isfinite(c);
}

static uint32_t ArmUsbNextInternalCommandId(void)
{
    if (next_internal_command_id == 0u) {
        next_internal_command_id = 1u;
    }
    return next_internal_command_id++;
}

static MotionFault ArmUsbFaultFromCommandResult(Arm_Command_Result_e result)
{
    switch (result) {
        case ARM_COMMAND_PREFLIGHT_FAILED:
            return MOTIONFAULT_IK_UNREACHABLE;
        case ARM_COMMAND_INVALID:
        case ARM_COMMAND_UNSUPPORTED:
        case ARM_COMMAND_DUPLICATE:
            return MOTIONFAULT_COLLISION;
        case ARM_COMMAND_BUSY:
        case ARM_COMMAND_NOT_READY:
        case ARM_COMMAND_MODE_DENIED:
            return MOTIONFAULT_TIMEOUT;
        case ARM_COMMAND_OK:
        default:
            return MOTIONFAULT_NONE;
    }
}

static MotionFault ArmUsbFaultFromHost(const Arm_Host_Status_s *status)
{
    if (status == NULL) {
        return MOTIONFAULT_TIMEOUT;
    }
    if (status->fault_code == ARM_FAULT_EMERGENCY_STOP) {
        return MOTIONFAULT_ESTOP;
    }
    if (status->fault_code != ARM_FAULT_NONE) {
        return MOTIONFAULT_TIMEOUT;
    }
    if (status->tool_error_code != 0u) {
        return MOTIONFAULT_TIMEOUT;
    }
    return MOTIONFAULT_NONE;
}

static void ArmUsbFillCurrentFromHost(const Arm_Host_Status_s *host)
{
    float current_yaw_deg;

    if (host == NULL) {
        return;
    }
    current_yaw_deg = host->tool_yaw_target_deg;
    g_arm_usb_debug.current_x_mm = host->tool_tip_mm.x_mm;
    g_arm_usb_debug.current_y_mm = host->tool_tip_mm.y_mm;
    g_arm_usb_debug.current_z_mm = host->tool_tip_mm.z_mm;
    g_arm_usb_debug.current_yaw_deg = current_yaw_deg;
    g_arm_usb_debug.target_yaw_servo_deg = host->servo_target_deg[1];
    g_arm_usb_debug.target_yaw_pos = host->servo_target_pos[1];
    g_arm_usb_debug.magnet_on = host->magnet_on;

    g_arm_usb_comm_debug.current_x_mm = host->tool_tip_mm.x_mm;
    g_arm_usb_comm_debug.current_y_mm = host->tool_tip_mm.y_mm;
    g_arm_usb_comm_debug.current_z_mm = host->tool_tip_mm.z_mm;
    g_arm_usb_comm_debug.current_yaw_deg = current_yaw_deg;
    g_arm_usb_comm_debug.magnet_on = host->magnet_on;
}

static void ArmUsbSendMotionStatus(MotionState state, MotionFault fault)
{
    Packet_MotionStatus status;
    Arm_Host_Status_s host;

    memset(&status, 0, sizeof(status));
    status.state = (uint8_t)state;
    status.fault = (uint8_t)fault;
    if (ArmGetHostStatus(&host)) {
        ArmUsbFillCurrentFromHost(&host);
        status.x_mm = host.tool_tip_mm.x_mm;
        status.y_mm = host.tool_tip_mm.y_mm;
        status.yaw_deg = host.tool_yaw_target_deg;
    }
    if (state != last_motion_state || fault != last_motion_fault) {
        (void)protocol_send_motion_status(&status);
        last_motion_state = (uint8_t)state;
        last_motion_fault = (uint8_t)fault;
    }
    g_arm_usb_debug.motion_state = (uint8_t)state;
    g_arm_usb_debug.motion_fault = (uint8_t)fault;
}

static void ArmUsbSendCallbackStatus(uint8_t callback_id, uint8_t status)
{
    Packet_CallbackStatus packet;

    packet.callback_id = callback_id;
    packet.callback_status = status;
    (void)protocol_send_callback_status(&packet);
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

static uint8_t ArmUsbBusinessCallbackId(void)
{
    if (active_business == ARM_USB_BUSINESS_HOME) {
        return 0u;
    }
    if (active_business == ARM_USB_BUSINESS_MAGNET_ON) {
        return 5u;
    }
    if (active_business == ARM_USB_BUSINESS_MAGNET_OFF) {
        return 6u;
    }
    return 255u;
}

static void ArmUsbSetState(Arm_Usb_Action_State_e state, uint32_t now_ms)
{
    g_arm_usb_debug.action_state = (uint8_t)state;
    state_tick = now_ms;
    g_arm_usb_debug.state_tick = now_ms;
    g_arm_usb_debug.dwell_elapsed_ms = 0u;
    if (state == ARM_USB_MAGNET_ON_DWELL ||
        state == ARM_USB_MAGNET_OFF_DWELL) {
        magnet_action_done = 0u;
    }
    if (state == ARM_USB_MAGNET_OFF_RAISING) {
        magnet_off_reset_done = 0u;
        magnet_off_reset_tick = 0u;
    }
}

static void ArmUsbSetBusiness(Arm_Usb_Business_e business)
{
    active_business = (uint8_t)business;
    g_arm_usb_debug.active_business = active_business;
}

static Arm_Command_Result_e ArmUsbSubmitTipMove(float x_mm, float y_mm,
                                                float z_mm,
                                                float speed_mm_s,
                                                uint8_t yaw_valid,
                                                float yaw_deg)
{
    Arm_Command_s command;

    memset(&command, 0, sizeof(command));
    active_target_x_mm = x_mm;
    active_target_y_mm = y_mm;
    active_target_z_mm = z_mm;
    if (yaw_valid != 0u) {
        active_target_yaw_deg = yaw_deg;
    }
    active_arm_command_id = ArmUsbNextInternalCommandId();
    command.command_id = active_arm_command_id;
    command.type = ARM_COMMAND_TYPE_CARTESIAN;
    command.payload.cartesian.control_point = ARM_CONTROL_POINT_TOOL_TIP;
    command.payload.cartesian.move_type = ARM_MOVE_LINEAR;
    command.payload.cartesian.target_mm.x_mm = x_mm;
    command.payload.cartesian.target_mm.y_mm = y_mm;
    command.payload.cartesian.target_mm.z_mm = z_mm;
    command.payload.cartesian.max_speed_mm_s = speed_mm_s;
    command.payload.cartesian.tool_pitch_valid = 0u;
    command.payload.cartesian.tool_yaw_valid = yaw_valid;
    command.payload.cartesian.tool_yaw_deg = yaw_deg;
    g_arm_usb_debug.internal_arm_command_id = active_arm_command_id;
    g_arm_usb_debug.commanded_z_mm = z_mm;
    return ArmSubmitCommand(&command);
}

static Arm_Command_Result_e ArmUsbSubmitTargetMoveLift(
    const Arm_Host_Status_s *host)
{
    if (host == NULL) {
        return ARM_COMMAND_INVALID;
    }
    return ArmUsbSubmitTipMove(host->tool_tip_mm.x_mm,
                               host->tool_tip_mm.y_mm,
                               ARM_USB_MOVE_Z_MM,
                               ARM_USB_MAGNET_Z_SPEED_MM_S,
                               0u, 0.0f);
}

static Arm_Command_Result_e ArmUsbSubmitTargetMoveFinal(void)
{
    return ArmUsbSubmitTipMove(pending_target_x_mm, pending_target_y_mm,
                               ARM_USB_MOVE_Z_MM,
                               ARM_USB_MOVE_SPEED_MM_S,
                               1u, pending_target_yaw_deg);
}

static Arm_Command_Result_e ArmUsbSubmitServo2Reset(void)
{
    Arm_Command_s command;

    memset(&command, 0, sizeof(command));
    active_arm_command_id = ArmUsbNextInternalCommandId();
    command.command_id = active_arm_command_id;
    command.type = ARM_COMMAND_TYPE_TOOL;
    command.payload.tool.action = ARM_TOOL_ACTION_SERVO2_WORLD_YAW;
    command.payload.tool.servo2_deg = 0.0f;
    g_arm_usb_debug.internal_arm_command_id = active_arm_command_id;
    g_arm_usb_debug.target_yaw_servo_deg = ARM_TOOL_SERVO2_FIXED_DEG;
    g_arm_usb_debug.target_yaw_pos = ARM_TOOL_SERVO2_NEUTRAL_POS;
    g_arm_usb_debug.target_yaw_result = ARM_COMMAND_BUSY;
    return ArmSubmitCommand(&command);
}

static uint8_t ArmUsbHostReadyAndIdle(const Arm_Host_Status_s *status)
{
    return status != NULL && status->ready != 0u && status->busy == 0u &&
           status->fault_code == ARM_FAULT_NONE;
}

static uint8_t ArmUsbCommandCompleted(const Arm_Host_Status_s *status)
{
    return status != NULL &&
           status->last_command_id == active_arm_command_id &&
           status->last_command_state == ARM_COMMAND_STATE_COMPLETED;
}

static uint8_t ArmUsbCommandFailed(const Arm_Host_Status_s *status)
{
    return status != NULL &&
           status->last_command_id == active_arm_command_id &&
           (status->last_command_state == ARM_COMMAND_STATE_REJECTED ||
            status->last_command_state == ARM_COMMAND_STATE_CANCELLED ||
            status->last_command_state == ARM_COMMAND_STATE_FAULTED);
}

static void ArmUsbFailActive(MotionFault fault, uint8_t release_magnet)
{
    Arm_Command_s cancel_command;

    target_move_phase = 0u;
    g_arm_usb_debug.target_move_phase = target_move_phase;
    target_yaw_pending = 0u;
    target_yaw_waiting = 0u;
    g_arm_usb_debug.target_yaw_pending = target_yaw_pending;
    g_arm_usb_debug.target_yaw_waiting = target_yaw_waiting;
    if (release_magnet != 0u) {
        ArmToolSetMagnet(0u);
    }
    memset(&cancel_command, 0, sizeof(cancel_command));
    cancel_command.command_id = ArmUsbNextInternalCommandId();
    cancel_command.type = ARM_COMMAND_TYPE_CANCEL_MOTION;
    (void)ArmSubmitCommand(&cancel_command);
    if (active_business == ARM_USB_BUSINESS_TARGET_MOVE) {
        ArmUsbSendMotionStatus(MOTIONSTATE_FAILED, fault);
    } else {
        ArmUsbSendCallbackStatus(ArmUsbBusinessCallbackId(), STATUS_FAULT_RETRY);
    }
    last_action_failed = 1u;
    ArmUsbSetState(ARM_USB_ACTION_FAILED, g_arm_usb_debug.state_tick);
}

static void ArmUsbStartBusiness(Arm_Usb_Business_e business, uint32_t now_ms)
{
    ArmUsbSetBusiness(business);
    last_motion_state = 0u;
    last_motion_fault = 0u;
    if (business == ARM_USB_BUSINESS_TARGET_MOVE) {
        ArmUsbSendMotionStatus(MOTIONSTATE_ACCEPTED, MOTIONFAULT_NONE);
        ArmUsbSendMotionStatus(MOTIONSTATE_RUNNING, MOTIONFAULT_NONE);
        ArmUsbSetState(ARM_USB_TARGET_MOVE_RUNNING, now_ms);
    } else {
        ArmUsbSetState(ARM_USB_ACTION_IDLE, now_ms);
    }
}

static void ArmUsbCompleteActive(uint32_t now_ms)
{
    uint8_t finished_business = active_business;

    target_move_phase = 0u;
    g_arm_usb_debug.target_move_phase = target_move_phase;
    target_yaw_pending = 0u;
    target_yaw_waiting = 0u;
    g_arm_usb_debug.target_yaw_pending = target_yaw_pending;
    g_arm_usb_debug.target_yaw_waiting = target_yaw_waiting;
    if (finished_business == ARM_USB_BUSINESS_TARGET_MOVE) {
        ArmUsbSendMotionStatus(MOTIONSTATE_COMPLETED, MOTIONFAULT_NONE);
    } else if (finished_business == ARM_USB_BUSINESS_HOME ||
               finished_business == ARM_USB_BUSINESS_MAGNET_ON ||
               finished_business == ARM_USB_BUSINESS_MAGNET_OFF) {
        ArmUsbSendCallbackStatus(ArmUsbBusinessCallbackId(), STATUS_END);
    }
    last_action_failed = 0u;
    active_arm_command_id = 0u;
    ArmUsbSetBusiness(ARM_USB_BUSINESS_IDLE);
    ArmUsbSetState(ARM_USB_ACTION_IDLE, now_ms);
}

void ArmUsbBridgeInit(void)
{
    memset(&g_arm_usb_debug, 0, sizeof(g_arm_usb_debug));
    memset(&g_arm_usb_comm_debug, 0, sizeof(g_arm_usb_comm_debug));
    next_internal_command_id = 1u;
    active_arm_command_id = 0u;
    active_task_id = 0u;
    active_business = ARM_USB_BUSINESS_IDLE;
    last_motion_state = 0u;
    last_motion_fault = 0u;
    pending_task_end = 0u;
    last_action_failed = 0u;
    action_x_mm = 0.0f;
    action_y_mm = 0.0f;
    active_target_x_mm = 0.0f;
    active_target_y_mm = 0.0f;
    active_target_z_mm = 0.0f;
    active_target_yaw_deg = 0.0f;
    pending_target_x_mm = 0.0f;
    pending_target_y_mm = 0.0f;
    pending_target_yaw_deg = 0.0f;
    target_move_phase = 0u;
    target_yaw_pending = 0u;
    target_yaw_waiting = 0u;
    target_yaw_complete_tick = 0u;
    magnet_action_done = 0u;
    magnet_off_reset_done = 0u;
    magnet_off_reset_tick = 0u;
    ArmUsbSetState(ARM_USB_ACTION_IDLE, 0u);
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

    if (pkt->task_status == STATUS_FAULT_RETRY) {
        last_action_failed = 0u;
        return;
    }

    if (pkt->task_status == STATUS_END) {
        if (pkt->task_id >= 1u && pkt->task_id <= 4u) {
            active_task_id = pkt->task_id;
            g_arm_usb_debug.active_task_id = active_task_id;
            pending_task_end = 1u;
            ArmUsbSetBusiness(ARM_USB_BUSINESS_TASK_END_RECORD);
        }
        return;
    }

    if (pkt->task_status != STATUS_START) {
        return;
    }

    active_task_id = pkt->task_id;
    g_arm_usb_debug.active_task_id = active_task_id;
    pending_task_end = 0u;
    last_action_failed = 0u;

    if (active_business != ARM_USB_BUSINESS_IDLE &&
        active_business != ARM_USB_BUSINESS_TASK_START_RECORD &&
        active_business != ARM_USB_BUSINESS_TASK_END_RECORD) {
        ArmUsbSendCallbackStatus(pkt->task_id, STATUS_FAULT_RETRY);
        return;
    }

    if (pkt->task_id >= 1u && pkt->task_id <= 4u) {
        ArmUsbSetBusiness(ARM_USB_BUSINESS_TASK_START_RECORD);
        return;
    }

    if (!ArmGetHostStatus(&host)) {
        ArmUsbSendCallbackStatus(pkt->task_id, STATUS_FAULT_RETRY);
        return;
    }
    if (!ArmUsbHostReadyAndIdle(&host)) {
        ArmUsbSendCallbackStatus(pkt->task_id, STATUS_FAULT_RETRY);
        return;
    }

    action_x_mm = host.tool_tip_mm.x_mm;
    action_y_mm = host.tool_tip_mm.y_mm;

    if (pkt->task_id == 0u) {
        ArmUsbStartBusiness(ARM_USB_BUSINESS_HOME, now_ms);
        result = ArmUsbSubmitTipMove(ARM_USB_HOME_X_MM, ARM_USB_HOME_Y_MM,
                                     ARM_USB_HOME_Z_MM,
                                     ARM_USB_HOME_SPEED_MM_S,
                                     1u, 0.0f);
        if (result == ARM_COMMAND_OK) {
            ArmUsbSetState(ARM_USB_HOME_RUNNING, now_ms);
        } else {
            ArmUsbSendCallbackStatus(pkt->task_id, STATUS_FAULT_RETRY);
            last_action_failed = 1u;
            ArmUsbSetState(ARM_USB_ACTION_FAILED, now_ms);
        }
    } else if (pkt->task_id == 5u) {
        ArmUsbStartBusiness(ARM_USB_BUSINESS_MAGNET_ON, now_ms);
        result = ArmUsbSubmitTipMove(action_x_mm, action_y_mm,
                                     ARM_USB_MAGNET_ACTION_Z_MM,
                                     ARM_USB_MAGNET_Z_SPEED_MM_S,
                                     0u, 0.0f);
        if (result == ARM_COMMAND_OK) {
            ArmUsbSetState(ARM_USB_MAGNET_ON_DESCENDING, now_ms);
        } else {
            ArmUsbSendCallbackStatus(pkt->task_id, STATUS_FAULT_RETRY);
            last_action_failed = 1u;
            ArmUsbSetState(ARM_USB_ACTION_FAILED, now_ms);
        }
    } else if (pkt->task_id == 6u) {
        ArmUsbStartBusiness(ARM_USB_BUSINESS_MAGNET_OFF, now_ms);
        result = ArmUsbSubmitTipMove(action_x_mm, action_y_mm,
                                     ARM_USB_MAGNET_ACTION_Z_MM,
                                     ARM_USB_MAGNET_Z_SPEED_MM_S,
                                     0u, 0.0f);
        if (result == ARM_COMMAND_OK) {
            ArmUsbSetState(ARM_USB_MAGNET_OFF_DESCENDING, now_ms);
        } else {
            ArmUsbSendCallbackStatus(pkt->task_id, STATUS_FAULT_RETRY);
            last_action_failed = 1u;
            ArmUsbSetState(ARM_USB_ACTION_FAILED, now_ms);
        }
    }
}

void ArmUsbBridgeOnTargetControl(const Packet_TargetControl *pkt)
{
    Arm_Host_Status_s host;
    Arm_Command_Result_e result;
    MotionFault fault;
    uint32_t now_ms = HAL_GetTick();

    if (pkt == NULL) {
        return;
    }
    g_arm_usb_comm_debug.rx_packet_id = PACKET_ID_TARGETCONTROL;
    g_arm_usb_debug.requested_x_mm = pkt->x_mm;
    g_arm_usb_debug.requested_y_mm = pkt->y_mm;
    g_arm_usb_debug.requested_yaw_deg = pkt->yaw_deg;

    if (g_arm_usb_debug.action_state != ARM_USB_ACTION_IDLE ||
        active_business != ARM_USB_BUSINESS_IDLE) {
        ArmUsbSendMotionStatus(MOTIONSTATE_FAILED, MOTIONFAULT_TIMEOUT);
        return;
    }
    if (!ArmUsbFinite3(pkt->x_mm, pkt->y_mm, pkt->yaw_deg) ||
        pkt->yaw_deg < ARM_USB_YAW_MIN_DEG ||
        pkt->yaw_deg > ARM_USB_YAW_MAX_DEG) {
        ArmUsbSendMotionStatus(MOTIONSTATE_FAILED, MOTIONFAULT_COLLISION);
        return;
    }
    if (!ArmGetHostStatus(&host)) {
        ArmUsbSendMotionStatus(MOTIONSTATE_FAILED, MOTIONFAULT_TIMEOUT);
        return;
    }
    if (!ArmUsbHostReadyAndIdle(&host)) {
        ArmUsbSendMotionStatus(MOTIONSTATE_FAILED, MOTIONFAULT_TIMEOUT);
        return;
    }

    ArmUsbStartBusiness(ARM_USB_BUSINESS_TARGET_MOVE, now_ms);
    pending_target_x_mm = pkt->x_mm;
    pending_target_y_mm = pkt->y_mm;
    pending_target_yaw_deg = pkt->yaw_deg;
    active_target_yaw_deg = pkt->yaw_deg;
    g_arm_usb_debug.target_yaw_servo_deg =
        ARM_USB_YAW_NEUTRAL_DEG + pkt->yaw_deg;
    g_arm_usb_debug.target_yaw_pos = ArmUsbYawToServo2Position(pkt->yaw_deg);
    g_arm_usb_debug.target_yaw_result = ARM_COMMAND_BUSY;
    /*
     * 只有电磁铁吸住工件后移动XY时，才强制先抬到安全Z高度。
     * 空载普通移动直接去目标点，避免HOME等低位姿态后“当前XY抬Z”
     * 这一段先被IK/限位拒绝，导致motion看起来完全不执行。
     */
    target_move_phase = host.magnet_on != 0u ? 1u : 2u;
    g_arm_usb_debug.target_move_phase = target_move_phase;
    result = target_move_phase == 1u ?
        ArmUsbSubmitTargetMoveLift(&host) : ArmUsbSubmitTargetMoveFinal();
    if (result == ARM_COMMAND_OK) {
        ArmUsbSetState(ARM_USB_TARGET_MOVE_RUNNING, now_ms);
    } else {
        fault = ArmUsbFaultFromCommandResult(result);
        ArmUsbSendMotionStatus(MOTIONSTATE_FAILED, fault);
        last_action_failed = 1u;
        ArmUsbSetState(ARM_USB_ACTION_FAILED, now_ms);
    }
}

void ArmUsbBridgeTask(uint32_t now_ms)
{
    Arm_Host_Status_s host;
    const Protocol_Debug_s *protocol_debug = protocol_get_debug();
    MotionFault fault;
    Arm_Command_Result_e result;

    g_arm_usb_debug.connection_ready = protocol_connection_ready();
    g_arm_usb_debug.link_online = protocol_link_is_online();
    g_arm_usb_debug.rx_frame_count = protocol_debug->rx_frame_count;
    g_arm_usb_debug.crc_fail_count = protocol_debug->crc_fail_count;
    g_arm_usb_debug.duplicate_count = protocol_debug->duplicate_count;
    g_arm_usb_debug.usb_rx_overflow_count = g_usb_rx_overflow_count;
    g_arm_usb_debug.usb_tx_fail_count = g_usb_tx_fail_count;
    g_arm_usb_debug.state_tick = state_tick;

    g_arm_usb_comm_debug.usb_task_alive = 1u;
    g_arm_usb_comm_debug.connection_ready =
        g_arm_usb_debug.connection_ready;
    g_arm_usb_comm_debug.rx_task_id = protocol_debug->last_task_id;
    g_arm_usb_comm_debug.rx_task_status =
        protocol_debug->last_task_status;
    g_arm_usb_comm_debug.rx_x_mm = protocol_debug->last_target_x_mm;
    g_arm_usb_comm_debug.rx_y_mm = protocol_debug->last_target_y_mm;
    g_arm_usb_comm_debug.rx_yaw_deg =
        protocol_debug->last_target_yaw_deg;
    g_arm_usb_comm_debug.active_task_id = active_task_id;
    g_arm_usb_comm_debug.active_business = active_business;
    g_arm_usb_comm_debug.action_state = g_arm_usb_debug.action_state;
    g_arm_usb_comm_debug.motion_state = g_arm_usb_debug.motion_state;
    g_arm_usb_comm_debug.motion_fault = g_arm_usb_debug.motion_fault;
    g_arm_usb_comm_debug.target_x_mm = active_target_x_mm;
    g_arm_usb_comm_debug.target_y_mm = active_target_y_mm;
    g_arm_usb_comm_debug.target_z_mm = active_target_z_mm;
    g_arm_usb_comm_debug.target_yaw_deg = active_target_yaw_deg;
    g_arm_usb_comm_debug.task_tick = now_ms;
    g_arm_usb_comm_debug.rx_frame_count = protocol_debug->rx_frame_count;
    g_arm_usb_comm_debug.crc_fail_count = protocol_debug->crc_fail_count;
    g_arm_usb_comm_debug.duplicate_count =
        protocol_debug->duplicate_count + g_arm_usb_debug.duplicate_count;
    g_arm_usb_comm_debug.ack_tx_count = protocol_debug->ack_tx_count;
    g_arm_usb_comm_debug.ack_rx_count = protocol_debug->ack_rx_count;
    g_arm_usb_comm_debug.motion_status_tx_count =
        protocol_debug->motion_status_tx_count;
    g_arm_usb_comm_debug.callback_status_tx_count =
        protocol_debug->callback_status_tx_count;
    g_arm_usb_comm_debug.usb_rx_overflow_count =
        g_usb_rx_overflow_count;
    g_arm_usb_comm_debug.usb_tx_fail_count = g_usb_tx_fail_count;

    if (!ArmGetHostStatus(&host)) {
        return;
    }
    ArmUsbFillCurrentFromHost(&host);

    if (host.fault_code != ARM_FAULT_NONE &&
        g_arm_usb_debug.action_state != ARM_USB_ACTION_IDLE &&
        g_arm_usb_debug.action_state != ARM_USB_ACTION_FAILED) {
        ArmUsbFailActive(ArmUsbFaultFromHost(&host), 0u);
        return;
    }

    switch ((Arm_Usb_Action_State_e)g_arm_usb_debug.action_state) {
        case ARM_USB_TARGET_MOVE_RUNNING:
            if (ArmUsbCommandCompleted(&host)) {
                if (target_move_phase == 1u) {
                    result = ArmUsbSubmitTargetMoveFinal();
                    if (result == ARM_COMMAND_OK) {
                        target_move_phase = 2u;
                        g_arm_usb_debug.target_move_phase =
                            target_move_phase;
                    } else {
                        ArmUsbFailActive(
                            ArmUsbFaultFromCommandResult(result), 0u);
                        last_action_failed = 1u;
                    }
                } else {
                    if (target_yaw_pending != 0u ||
                        target_yaw_waiting != 0u) {
                        break;
                    }
                    g_arm_usb_debug.target_yaw_result = ARM_COMMAND_OK;
                    g_arm_usb_debug.target_yaw_pos =
                        host.servo_target_pos[1];
                    target_move_phase = 0u;
                    g_arm_usb_debug.target_move_phase = target_move_phase;
                    ArmUsbCompleteActive(now_ms);
                }
            } else if (ArmUsbCommandFailed(&host)) {
                target_move_phase = 0u;
                g_arm_usb_debug.target_move_phase = target_move_phase;
                ArmUsbFailActive(
                    ArmUsbFaultFromCommandResult(host.last_command_result),
                    0u);
                last_action_failed = 1u;
            }
            break;

        case ARM_USB_HOME_RUNNING:
            if (ArmUsbCommandCompleted(&host)) {
                ArmUsbCompleteActive(now_ms);
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbFailActive(
                    ArmUsbFaultFromCommandResult(host.last_command_result),
                    0u);
                last_action_failed = 1u;
            }
            break;

        case ARM_USB_MAGNET_ON_DESCENDING:
            if (ArmUsbCommandCompleted(&host)) {
                ArmUsbSetState(ARM_USB_MAGNET_ON_DWELL, now_ms);
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbFailActive(MOTIONFAULT_IK_UNREACHABLE, 1u);
            }
            break;

        case ARM_USB_MAGNET_ON_DWELL:
            g_arm_usb_debug.dwell_elapsed_ms = now_ms - state_tick;
            if (g_arm_usb_debug.dwell_elapsed_ms <
                ARM_USB_MAGNET_ACTION_DELAY_MS) {
                break;
            }
            if (magnet_action_done == 0u && host.magnet_on == 0u) {
                ArmToolSetMagnet(1u);
                magnet_action_done = 1u;
            }
            if ((uint32_t)(now_ms - state_tick) >=
                ARM_USB_MAGNET_ACTION_DELAY_MS +
                ARM_USB_MAGNET_DWELL_MS) {
                result = ArmUsbSubmitTipMove(action_x_mm, action_y_mm,
                    ARM_USB_MOVE_Z_MM, ARM_USB_MAGNET_Z_SPEED_MM_S,
                    0u, 0.0f);
                if (result == ARM_COMMAND_OK) {
                    ArmUsbSetState(ARM_USB_MAGNET_ON_RAISING, now_ms);
                } else {
                    ArmUsbFailActive(ArmUsbFaultFromCommandResult(result),
                                     1u);
                }
            }
            break;

        case ARM_USB_MAGNET_ON_RAISING:
            if (ArmUsbCommandCompleted(&host)) {
                ArmUsbCompleteActive(now_ms);
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbFailActive(MOTIONFAULT_IK_UNREACHABLE, 1u);
            }
            break;

        case ARM_USB_MAGNET_OFF_DESCENDING:
            if (ArmUsbCommandCompleted(&host)) {
                ArmUsbSetState(ARM_USB_MAGNET_OFF_DWELL, now_ms);
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbSendCallbackStatus(ArmUsbBusinessCallbackId(),
                                         STATUS_FAULT_RETRY);
                last_action_failed = 1u;
                ArmUsbSetState(ARM_USB_ACTION_FAILED, now_ms);
            }
            break;

        case ARM_USB_MAGNET_OFF_DWELL:
            g_arm_usb_debug.dwell_elapsed_ms = now_ms - state_tick;
            if (magnet_action_done == 0u) {
                if (g_arm_usb_debug.dwell_elapsed_ms <
                    ARM_USB_MAGNET_ACTION_DELAY_MS) {
                    break;
                }
                ArmToolSetMagnet(0u);
                magnet_action_done = 1u;
                state_tick = now_ms;
                g_arm_usb_debug.state_tick = now_ms;
                g_arm_usb_debug.dwell_elapsed_ms = 0u;
                break;
            }
            if ((uint32_t)(now_ms - state_tick) >=
                ARM_USB_MAGNET_DWELL_MS) {
                result = ArmUsbSubmitTipMove(action_x_mm, action_y_mm,
                    ARM_USB_MOVE_Z_MM, ARM_USB_MAGNET_Z_SPEED_MM_S,
                    0u, 0.0f);
                if (result == ARM_COMMAND_OK) {
                    ArmUsbSetState(ARM_USB_MAGNET_OFF_RAISING, now_ms);
                } else {
                    fault = ArmUsbFaultFromCommandResult(result);
                    (void)fault;
                    ArmUsbSendCallbackStatus(ArmUsbBusinessCallbackId(),
                                             STATUS_FAULT_RETRY);
                    last_action_failed = 1u;
                    ArmUsbSetState(ARM_USB_ACTION_FAILED, now_ms);
                }
            }
            break;

        case ARM_USB_MAGNET_OFF_RAISING:
            if (magnet_off_reset_done == 0u) {
                if (ArmUsbCommandCompleted(&host)) {
                    result = ArmUsbSubmitServo2Reset();
                    if (result == ARM_COMMAND_BUSY) {
                        break;
                    }
                    if (result != ARM_COMMAND_OK) {
                        ArmUsbSendCallbackStatus(ArmUsbBusinessCallbackId(),
                                                 STATUS_FAULT_RETRY);
                        last_action_failed = 1u;
                        ArmUsbSetState(ARM_USB_ACTION_FAILED, now_ms);
                        break;
                    }
                    magnet_off_reset_done = 1u;
                    magnet_off_reset_tick = now_ms;
                    break;
                }
                if (ArmUsbCommandFailed(&host)) {
                    ArmUsbSendCallbackStatus(ArmUsbBusinessCallbackId(),
                                             STATUS_FAULT_RETRY);
                    last_action_failed = 1u;
                    ArmUsbSetState(ARM_USB_ACTION_FAILED, now_ms);
                }
            } else if (ArmUsbCommandCompleted(&host)) {
                if ((uint32_t)(now_ms - magnet_off_reset_tick) <
                    ARM_USB_YAW_MOVE_TIME_MS + ARM_USB_YAW_SETTLE_MS) {
                    break;
                }
                g_arm_usb_debug.target_yaw_result = ARM_COMMAND_OK;
                ArmUsbCompleteActive(now_ms);
            } else if (ArmUsbCommandFailed(&host)) {
                ArmUsbSendCallbackStatus(ArmUsbBusinessCallbackId(),
                                         STATUS_FAULT_RETRY);
                last_action_failed = 1u;
                ArmUsbSetState(ARM_USB_ACTION_FAILED, now_ms);
            }
            break;

        case ARM_USB_ACTION_FAILED:
            if (host.busy == 0u) {
                active_arm_command_id = 0u;
                ArmUsbSetBusiness(ARM_USB_BUSINESS_IDLE);
                ArmUsbSetState(ARM_USB_ACTION_IDLE, now_ms);
            }
            break;

        case ARM_USB_ACTION_IDLE:
        default:
            if (pending_task_end != 0u && host.busy == 0u &&
                host.fault_code == ARM_FAULT_NONE &&
                last_action_failed == 0u) {
                pending_task_end = 0u;
                ArmUsbSetBusiness(ARM_USB_BUSINESS_IDLE);
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
