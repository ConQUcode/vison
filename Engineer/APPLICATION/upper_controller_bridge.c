/**
 * @file upper_controller_bridge.c
 * @brief 将新版上位机业务包适配到固件已有的非阻塞控制接口。
 */

#include "upper_controller_bridge.h"

#include <math.h>
#include <string.h>

#include "app_arm_command_id.h"
#include "mg995_servo.h"
#include "protocol_port.h"
#include "protocol_runtime.h"

#define UPPER_TASK_GRIPPER                 0u
#define UPPER_TASK_CAMERA_GIMBAL           1u
#define UPPER_TASK_STATUS_PRIMARY          0u
#define UPPER_TASK_STATUS_SECONDARY        1u
#define UPPER_CALLBACK_GRIPPER             0u
#define UPPER_CALLBACK_CAMERA_GIMBAL       1u
#define UPPER_CALLBACK_COMPLETED           0u
#define UPPER_CALLBACK_EXECUTING           1u
#define UPPER_ARM_TARGET_MAX_ABS_M        10.0f
#define UPPER_ARM_TARGET_MAX_Z_TYPE       15u
#define UPPER_CHASSIS_COMMAND_ID_SEED 0xC2000000u

Upper_Controller_Debug_s g_upper_controller_debug;

static Packet_StateMachineCommand upper_pending_discrete;
static uint32_t upper_next_chassis_command_id;

static uint8_t UpperControllerSendCallback(uint8_t callback_id,
                                           uint8_t callback_status)
{
    Packet_ExecutionCallback packet;

    packet.callback_id = callback_id;
    packet.callback_status = callback_status;
    send_ExecutionCallback(&packet);
    if (ProtocolPortLastWriteOk() == 0u) {
        g_upper_controller_debug.execution_callback_tx_fail_count++;
        return 0u;
    }
    g_upper_controller_debug.execution_callback_tx_count++;
    return 1u;
}

static uint32_t UpperControllerNextChassisCommandId(void)
{
    upper_next_chassis_command_id++;
    if (upper_next_chassis_command_id == 0u) {
        upper_next_chassis_command_id = 1u;
    }
    return upper_next_chassis_command_id;
}

static uint8_t UpperControllerPacketAllowed(void)
{
    return (uint8_t)(g_upper_controller_debug.initialized != 0u &&
        ProtocolRuntimeConnectionReady() != 0u &&
        ProtocolRuntimeLinkOnline() != 0u);
}

static void UpperControllerRunCameraCommand(uint32_t now_ms)
{
    float camera_angle_deg =
        upper_pending_discrete.task_status == UPPER_TASK_STATUS_PRIMARY ?
            UPPER_CAMERA_LOOK_DOWN_DEG : UPPER_CAMERA_LOOK_UP_DEG;

    if (Mg995ServoSetCameraAngles(camera_angle_deg,
                                  camera_angle_deg) != 0u) {
        (void)UpperControllerSendCallback(
            UPPER_CALLBACK_CAMERA_GIMBAL, UPPER_CALLBACK_EXECUTING);
        g_upper_controller_debug.camera_motion_start_tick = now_ms;
        g_upper_controller_debug.discrete_state = UPPER_DISCRETE_RUNNING;
    } else {
        g_upper_controller_debug.discrete_invalid_count++;
        g_upper_controller_debug.discrete_state = UPPER_DISCRETE_IDLE;
    }
}

static void UpperControllerPollCameraCommand(uint32_t now_ms)
{
    if ((uint32_t)(now_ms -
            g_upper_controller_debug.camera_motion_start_tick) <
        UPPER_CAMERA_SETTLE_MS) {
        return;
    }
    (void)UpperControllerSendCallback(
        UPPER_CALLBACK_CAMERA_GIMBAL, UPPER_CALLBACK_COMPLETED);
    g_upper_controller_debug.discrete_complete_count++;
    g_upper_controller_debug.discrete_state = UPPER_DISCRETE_IDLE;
}

static void UpperControllerSubmitGripperCommand(void)
{
    Arm_Command_s command;
    Arm_Command_Result_e result;

    memset(&command, 0, sizeof(command));
    if (g_upper_controller_debug.gripper_command_id == 0u) {
        g_upper_controller_debug.gripper_command_id =
            AppArmCommandIdNext();
    }
    command.command_id = g_upper_controller_debug.gripper_command_id;
    command.type = ARM_COMMAND_TYPE_TOOL;
    command.payload.tool.action =
        upper_pending_discrete.task_status == UPPER_TASK_STATUS_PRIMARY ?
            ARM_TOOL_ACTION_GRIPPER_CLOSE : ARM_TOOL_ACTION_GRIPPER_OPEN;
    result = ArmSubmitCommand(&command);
    g_upper_controller_debug.gripper_submit_result = result;
    if (result == ARM_COMMAND_BUSY || result == ARM_COMMAND_NOT_READY) {
        return;
    }
    if (result != ARM_COMMAND_OK) {
        g_upper_controller_debug.discrete_invalid_count++;
        g_upper_controller_debug.discrete_state = UPPER_DISCRETE_IDLE;
        g_upper_controller_debug.gripper_command_id = 0u;
        return;
    }
    g_upper_controller_debug.discrete_state = UPPER_DISCRETE_RUNNING;
    (void)UpperControllerSendCallback(
        UPPER_CALLBACK_GRIPPER, UPPER_CALLBACK_EXECUTING);
}

static void UpperControllerPollGripperCommand(void)
{
    Arm_Host_Status_s status;
    uint32_t command_id = g_upper_controller_debug.gripper_command_id;

    if (ArmGetHostStatus(&status) == 0u || command_id == 0u) {
        return;
    }
    if (status.last_command_id != command_id) {
        return;
    }
    if (status.last_command_state == ARM_COMMAND_STATE_COMPLETED) {
        (void)UpperControllerSendCallback(
            UPPER_CALLBACK_GRIPPER, UPPER_CALLBACK_COMPLETED);
        g_upper_controller_debug.discrete_complete_count++;
    } else if (status.last_command_state != ARM_COMMAND_STATE_REJECTED &&
               status.last_command_state != ARM_COMMAND_STATE_CANCELLED &&
               status.last_command_state != ARM_COMMAND_STATE_FAULTED) {
        return;
    } else {
        g_upper_controller_debug.discrete_invalid_count++;
    }
    g_upper_controller_debug.discrete_state = UPPER_DISCRETE_IDLE;
    g_upper_controller_debug.gripper_command_id = 0u;
}

void UpperControllerBridgeInit(void)
{
    memset(&g_upper_controller_debug, 0,
           sizeof(g_upper_controller_debug));
    memset(&upper_pending_discrete, 0,
           sizeof(upper_pending_discrete));
    upper_next_chassis_command_id = UPPER_CHASSIS_COMMAND_ID_SEED;
    g_upper_controller_debug.initialized = 1u;
}

void UpperControllerBridgeTask(uint32_t now_ms)
{
    if (g_upper_controller_debug.initialized == 0u) {
        return;
    }
    if (g_upper_controller_debug.discrete_state ==
            UPPER_DISCRETE_PENDING) {
        if (upper_pending_discrete.task_id ==
            UPPER_TASK_CAMERA_GIMBAL) {
            UpperControllerRunCameraCommand(now_ms);
        } else {
            UpperControllerSubmitGripperCommand();
        }
    } else if (g_upper_controller_debug.discrete_state ==
               UPPER_DISCRETE_RUNNING) {
        if (upper_pending_discrete.task_id ==
            UPPER_TASK_CAMERA_GIMBAL) {
            UpperControllerPollCameraCommand(now_ms);
        } else {
            UpperControllerPollGripperCommand();
        }
    }
}

void on_receive_StateMachineCommand(
    const Packet_StateMachineCommand *packet)
{
    ProtocolRuntimeNotifyApplicationRx();
    if (packet == NULL ||
        UpperControllerPacketAllowed() == 0u ||
        packet->task_id > UPPER_TASK_CAMERA_GIMBAL ||
        packet->task_status > UPPER_TASK_STATUS_SECONDARY) {
        g_upper_controller_debug.discrete_invalid_count++;
        return;
    }
    g_upper_controller_debug.discrete_rx_count++;
    if (g_upper_controller_debug.discrete_state !=
            UPPER_DISCRETE_IDLE) {
        if (packet->task_id == upper_pending_discrete.task_id &&
            packet->task_status ==
                upper_pending_discrete.task_status) {
            g_upper_controller_debug.discrete_duplicate_count++;
        } else {
            g_upper_controller_debug.discrete_busy_count++;
        }
        return;
    }
    upper_pending_discrete = *packet;
    g_upper_controller_debug.pending_task_id = packet->task_id;
    g_upper_controller_debug.pending_task_status = packet->task_status;
    g_upper_controller_debug.discrete_state = UPPER_DISCRETE_PENDING;
}

void on_receive_ArmTarget(const Packet_ArmTarget *packet)
{
    uint8_t valid;

    ProtocolRuntimeNotifyApplicationRx();
    g_upper_controller_debug.arm_target_rx_count++;
    valid = (uint8_t)(packet != NULL &&
        UpperControllerPacketAllowed() != 0u &&
        isfinite(packet->target_x) && isfinite(packet->target_y) &&
        isfinite(packet->target_z) &&
        fabsf(packet->target_x) <= UPPER_ARM_TARGET_MAX_ABS_M &&
        fabsf(packet->target_y) <= UPPER_ARM_TARGET_MAX_ABS_M &&
        fabsf(packet->target_z) <= UPPER_ARM_TARGET_MAX_ABS_M &&
        packet->z_type <= UPPER_ARM_TARGET_MAX_Z_TYPE);
    g_upper_controller_debug.arm_target_valid = valid;
    if (valid == 0u) {
        g_upper_controller_debug.arm_target_invalid_count++;
        return;
    }
    g_upper_controller_debug.arm_target_camera_m[0] = packet->target_x;
    g_upper_controller_debug.arm_target_camera_m[1] = packet->target_y;
    g_upper_controller_debug.arm_target_camera_m[2] = packet->target_z;
    g_upper_controller_debug.arm_target_camera_mm[0] =
        packet->target_x * 1000.0f;
    g_upper_controller_debug.arm_target_camera_mm[1] =
        packet->target_y * 1000.0f;
    g_upper_controller_debug.arm_target_camera_mm[2] =
        packet->target_z * 1000.0f;
    g_upper_controller_debug.arm_target_z_type = packet->z_type;
    /* 相机外参和拍照姿态尚未定义，可靠ACK只表示已收到，不表示执行。 */
    g_upper_controller_debug.arm_target_deferred_count++;
}

void on_receive_VelocityCommand(const Packet_VelocityCommand *packet)
{
    Chassis_Velocity_Command_s command;
    Chassis_Command_Result_e result;

    ProtocolRuntimeNotifyApplicationRx();
    g_upper_controller_debug.velocity_rx_count++;
    if (packet == NULL ||
        UpperControllerPacketAllowed() == 0u ||
        !isfinite(packet->linear_x) ||
        !isfinite(packet->angular_z)) {
        g_upper_controller_debug.velocity_reject_count++;
        return;
    }
    g_upper_controller_debug.velocity_linear_x_m_s = packet->linear_x;
    g_upper_controller_debug.velocity_angular_z_rad_s =
        packet->angular_z;
    g_upper_controller_debug.velocity_vx_mm_s =
        packet->linear_x * 1000.0f;
    command.command_id = UpperControllerNextChassisCommandId();
    command.vx_mm_s = g_upper_controller_debug.velocity_vx_mm_s;
    command.wz_rad_s = packet->angular_z;
    result = ChassisSubmitVelocityCommand(&command);
    g_upper_controller_debug.velocity_command_id = command.command_id;
    g_upper_controller_debug.velocity_submit_result = result;
    if (result == CHASSIS_COMMAND_ACCEPTED) {
        g_upper_controller_debug.velocity_accept_count++;
    } else {
        g_upper_controller_debug.velocity_reject_count++;
    }
}

void on_receive_ExecutionCallback(
    const Packet_ExecutionCallback *packet)
{
    ProtocolRuntimeNotifyApplicationRx();
    if (packet != NULL) {
        g_upper_controller_debug.unexpected_callback_rx_count++;
    }
}
