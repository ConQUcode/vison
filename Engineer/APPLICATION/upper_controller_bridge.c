/**
 * @file upper_controller_bridge.c
 * @brief 将新版上位机业务包适配到固件已有的非阻塞控制接口。
 */

#include "upper_controller_bridge.h"

#include <math.h>
#include <string.h>

#include "app_arm_command_id.h"
#include "app_arm_side_pick_place.h"
#include "app_config.h"
#include "arm.h"
#include "arm_config.h"
#include "arm_tool.h"
#include "camera_target_transform_config.h"
#include "mg995_servo.h"
#include "protocol_port.h"
#include "protocol_runtime.h"
#include "stm32f4xx_hal.h"

#define UPPER_TASK_GRIPPER                 0u
#define UPPER_TASK_CAMERA_GIMBAL           1u
#define UPPER_TASK_AC_SIDE_PICK            2u
#define UPPER_TASK_QR_RECOGNITION_POSE     4u
#define UPPER_TASK_CURRENT_AREA            5u
#define UPPER_TASK_RETURN_INITIAL_POSE     6u
#define UPPER_TASK_ARM_RETRACT_POSE        7u
#define UPPER_TASK_STATUS_PRIMARY          0u
#define UPPER_TASK_STATUS_SECONDARY        1u
#define UPPER_TASK_STATUS_AREA_D            3u
#define UPPER_CALLBACK_GRIPPER             0u
#define UPPER_CALLBACK_CAMERA_GIMBAL       1u
#define UPPER_CALLBACK_AC_SIDE_PICK        2u
#define UPPER_CALLBACK_ARM_TARGET          3u
#define UPPER_CALLBACK_QR_RECOGNITION_POSE 4u
#define UPPER_CALLBACK_CURRENT_AREA        5u
#define UPPER_CALLBACK_RETURN_INITIAL_POSE 6u
#define UPPER_CALLBACK_ARM_RETRACT_POSE    7u
#define UPPER_CALLBACK_COMPLETED           0u
#define UPPER_CALLBACK_EXECUTING           1u
/* 协议统一失败终态；ArmTarget使用callback_id=3上报。 */
#define UPPER_CALLBACK_FAILED              2u
#define UPPER_ARM_TARGET_MAX_ABS_M        10.0f
#define UPPER_ARM_TARGET_MAX_Z_TYPE       15u
#define UPPER_CHASSIS_COMMAND_ID_SEED 0xC2000000u
#define UPPER_AC_CAPTURE_ID_SEED      0xAC000000u
#define UPPER_ARM_TARGET_POSE_AGE_UNKNOWN 0xFFFFFFFFu
#define UPPER_ARM_TARGET_GATE_AC_NOT_HOLDING   (1u << 0)
#define UPPER_ARM_TARGET_GATE_DISCRETE_BUSY    (1u << 1)
#define UPPER_ARM_TARGET_GATE_PICK_RUNNING     (1u << 2)
#define UPPER_ARM_TARGET_GATE_SIDE_PICK_RUNNING (1u << 3)
#define UPPER_ARM_TARGET_GATE_BD_PICK_UNSUPPORTED (1u << 4)
#define UPPER_RESET_HOME_DONE_CANCEL  (1u << 0)
#define UPPER_RESET_HOME_DONE_ARM     (1u << 1)
#define UPPER_RESET_HOME_DONE_BASE    (1u << 2)
#define UPPER_RESET_HOME_DONE_PITCH   (1u << 3)
#define UPPER_RESET_HOME_DONE_GRIPPER (1u << 4)

Upper_Controller_Debug_s g_upper_controller_debug;
Upper_Arm_Target_Debug_s g_arm_target_debug;
Upper_Arm_Reject_Diagnostic_s g_upper_arm_reject_diagnostic;

typedef enum {
    UPPER_ARM_TARGET_FLOW_IDLE = 0,
    UPPER_ARM_TARGET_FLOW_PICK,
    UPPER_ARM_TARGET_FLOW_PLACE
} Upper_Arm_Target_Flow_State_e;

static Packet_StateMachineCommand upper_pending_discrete;
static uint32_t upper_next_chassis_command_id;
static uint32_t upper_next_ac_capture_id;
static uint8_t upper_current_area_callback_pending;
static App_Arm_Place_Profile_s upper_arm_target_place_profile;
static Upper_Arm_Target_Flow_State_e upper_arm_target_flow_state;

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

/** 将当前侧允许的小幅近端欠距钳位到已验证边界，异常侧别或超差仍拒绝。 */
static uint8_t UpperControllerClampAcNearY(
    App_Fruit_Side_e side, float raw_y_mm, float *command_y_mm)
{
    float side_sign;
    float side_distance_mm;
    float shortfall_mm;

    if (command_y_mm == NULL || !isfinite(raw_y_mm) ||
        (side != APP_FRUIT_SIDE_LEFT && side != APP_FRUIT_SIDE_RIGHT)) {
        g_upper_controller_debug.arm_target_near_y_reject_count++;
        return 0u;
    }
    side_sign = side == APP_FRUIT_SIDE_RIGHT ? -1.0f : 1.0f;
    side_distance_mm = side_sign * raw_y_mm;
    shortfall_mm = APP_ARM_AC_CLOSED_LOOP_NEAR_Y_MIN_MM - side_distance_mm;
    g_upper_controller_debug.arm_target_approach_y_raw_mm = raw_y_mm;
    g_upper_controller_debug.arm_target_near_y_min_mm =
        APP_ARM_AC_CLOSED_LOOP_NEAR_Y_MIN_MM;
    g_upper_controller_debug.arm_target_near_y_shortfall_mm =
        shortfall_mm > 0.0f ? shortfall_mm : 0.0f;
    g_upper_controller_debug.arm_target_near_y_clamped = 0u;

    if (shortfall_mm > APP_ARM_AC_CLOSED_LOOP_NEAR_Y_CLAMP_MAX_MM) {
        g_upper_controller_debug.arm_target_near_y_reject_count++;
        return 0u;
    }
    if (shortfall_mm > 0.0f) {
        *command_y_mm = side_sign * APP_ARM_AC_CLOSED_LOOP_NEAR_Y_MIN_MM;
        g_upper_controller_debug.arm_target_near_y_clamped = 1u;
        g_upper_controller_debug.arm_target_near_y_clamp_count++;
    } else {
        *command_y_mm = raw_y_mm;
    }
    g_upper_controller_debug.arm_target_approach_y_command_mm =
        *command_y_mm;
    return 1u;
}

static void UpperControllerRecordAcAdvanceResult(
    const App_Arm_Advance_Result_s *result)
{
    if (result == NULL) {
        return;
    }
    g_upper_controller_debug.arm_target_advance_requested_mm =
        result->requested_mm;
    g_upper_controller_debug.arm_target_advance_selected_mm =
        result->selected_mm;
    g_upper_controller_debug.arm_target_advance_reject_reason =
        result->reject_reason;
    g_upper_controller_debug.arm_target_advance_approach_failed =
        result->approach_failed;
    g_upper_controller_debug.arm_target_advance_planner_status =
        result->planner_status;
    g_upper_controller_debug.arm_target_advance_ik_status =
        result->ik_status;
    g_upper_controller_debug.arm_target_advance_workspace_result =
        result->workspace_safety_result;
    g_upper_controller_debug.arm_target_advance_failed_check_mask =
        result->failed_check_mask;
    g_upper_controller_debug.arm_target_advance_failed_sample =
        result->failed_sample;
    memcpy(g_upper_controller_debug.arm_target_advance_failed_center_mm,
           result->failed_center_mm,
           sizeof(result->failed_center_mm));
    g_upper_controller_debug.arm_target_advance_reduced =
        result->selected_mm > 0.0f &&
        result->selected_mm + 0.0001f < result->requested_mm;
    if (result->reject_reason != APP_ARM_ADVANCE_REJECT_NONE) {
        g_upper_controller_debug.arm_target_advance_reject_count++;
    } else if (g_upper_controller_debug.arm_target_advance_reduced != 0u) {
        g_upper_controller_debug.arm_target_advance_reduced = 1u;
        g_upper_controller_debug.arm_target_advance_reduce_count++;
    }
}

static void UpperControllerLatchArmFailure(
    Upper_Arm_Reject_Source_e source,
    Upper_Arm_Target_Debug_Stage_e failure_stage,
    uint8_t flow_diagnostic_valid,
    uint32_t bridge_command_id,
    uint32_t bridge_submit_result)
{
    Arm_Host_Status_s host;

    g_upper_arm_reject_diagnostic.valid = 0u;
    g_upper_arm_reject_diagnostic.count++;
    g_upper_arm_reject_diagnostic.tick_ms = HAL_GetTick();
    g_upper_arm_reject_diagnostic.source = source;
    g_upper_arm_reject_diagnostic.stage = failure_stage;
    g_upper_arm_reject_diagnostic.reset_home_state =
        g_upper_controller_debug.reset_home_state;
    g_upper_arm_reject_diagnostic.reset_home_completed_mask =
        g_upper_controller_debug.reset_home_completed_mask;
    g_upper_arm_reject_diagnostic.flow_diagnostic_valid =
        flow_diagnostic_valid;
    g_upper_arm_reject_diagnostic.flow_status =
        (App_Arm_Flow_Status_e)g_app_arm_pick_place_test_debug.flow_status;
    g_upper_arm_reject_diagnostic.pick_step =
        g_app_arm_pick_place_test_debug.pick_step;
    g_upper_arm_reject_diagnostic.place_step =
        g_app_arm_pick_place_test_debug.place_step;
    g_upper_arm_reject_diagnostic.failure_source =
        g_app_arm_pick_place_test_debug.failure_source;
    g_upper_arm_reject_diagnostic.bridge_command_id = bridge_command_id;
    g_upper_arm_reject_diagnostic.bridge_submit_result =
        bridge_submit_result;
    g_upper_arm_reject_diagnostic.active_command_id =
        g_app_arm_pick_place_test_debug.active_command_id;
    g_upper_arm_reject_diagnostic.command_state =
        g_app_arm_pick_place_test_debug.command_state;
    g_upper_arm_reject_diagnostic.command_result =
        g_app_arm_pick_place_test_debug.command_result;
    g_upper_arm_reject_diagnostic.arm_fault_code =
        g_app_arm_pick_place_test_debug.arm_fault_code;
    g_upper_arm_reject_diagnostic.motion_state =
        g_app_arm_pick_place_test_debug.motion_state;
    g_upper_arm_reject_diagnostic.motion_fault =
        g_app_arm_pick_place_test_debug.motion_fault;
    g_upper_arm_reject_diagnostic.tool_error_code =
        g_app_arm_pick_place_test_debug.tool_error_code;
    g_upper_arm_reject_diagnostic.ik_status =
        g_app_arm_pick_place_test_debug.ik_status;
    g_upper_arm_reject_diagnostic.workspace_safety_result =
        g_app_arm_pick_place_test_debug.workspace_safety_result;
    g_upper_arm_reject_diagnostic.failed_check_mask =
        g_app_arm_pick_place_test_debug.preflight_failed_check_mask;
    g_upper_arm_reject_diagnostic.failed_segment =
        g_app_arm_pick_place_test_debug.preflight_failed_segment;
    g_upper_arm_reject_diagnostic.failed_sample =
        g_app_arm_pick_place_test_debug.preflight_failed_sample;
    memcpy(g_upper_arm_reject_diagnostic.failed_center_mm,
           g_app_arm_pick_place_test_debug.preflight_failed_center_mm,
           sizeof(g_upper_arm_reject_diagnostic.failed_center_mm));
    memcpy(g_upper_arm_reject_diagnostic.failed_q_deg,
           g_app_arm_pick_place_test_debug.preflight_failed_q_deg,
           sizeof(g_upper_arm_reject_diagnostic.failed_q_deg));
    g_upper_arm_reject_diagnostic.advance_reject_reason =
        g_upper_controller_debug.arm_target_advance_reject_reason;
    g_upper_arm_reject_diagnostic.advance_planner_status =
        g_upper_controller_debug.arm_target_advance_planner_status;
    g_upper_arm_reject_diagnostic.advance_approach_failed =
        g_upper_controller_debug.arm_target_advance_approach_failed;
    g_upper_arm_reject_diagnostic.advance_requested_mm =
        g_upper_controller_debug.arm_target_advance_requested_mm;
    g_upper_arm_reject_diagnostic.advance_selected_mm =
        g_upper_controller_debug.arm_target_advance_selected_mm;

    memset(&host, 0, sizeof(host));
    g_upper_arm_reject_diagnostic.host_status_valid =
        ArmGetHostStatus(&host);
    g_upper_arm_reject_diagnostic.host_fault_code = host.fault_code;
    g_upper_arm_reject_diagnostic.host_last_command_id =
        host.last_command_id;
    g_upper_arm_reject_diagnostic.host_last_command_type =
        host.last_command_type;
    g_upper_arm_reject_diagnostic.host_last_command_state =
        host.last_command_state;
    g_upper_arm_reject_diagnostic.host_last_command_result =
        host.last_command_result;
    g_upper_arm_reject_diagnostic.valid = 1u;
}

/** 上报ArmTarget拒绝/失败并结束本次任务；保持原位，不自动取消或HOME。 */
static void UpperControllerHandleArmTargetFailure(
    Upper_Arm_Target_Debug_Stage_e failure_stage)
{
    uint8_t flow_diagnostic_valid =
        g_upper_controller_debug.arm_target_pick_running;

    UpperControllerLatchArmFailure(
        UPPER_ARM_REJECT_SOURCE_ARM_TARGET, failure_stage,
        flow_diagnostic_valid, 0u, 0u);
    g_upper_controller_debug.arm_target_pick_running = 0u;
    upper_arm_target_flow_state = UPPER_ARM_TARGET_FLOW_IDLE;
    g_upper_controller_debug.arm_target_pick_flow_status =
        APP_ARM_FLOW_FAILED;
    g_arm_target_debug.stage = failure_stage;
    g_upper_controller_debug.arm_target_pick_fail_count++;
    g_upper_controller_debug.discrete_invalid_count++;
    if (UpperControllerSendCallback(
            UPPER_CALLBACK_ARM_TARGET, UPPER_CALLBACK_FAILED) != 0u) {
        g_upper_controller_debug.arm_target_failed_callback_count++;
    } else {
        g_upper_controller_debug.arm_target_failed_callback_fail_count++;
    }
}

static uint32_t UpperControllerNextAcCaptureId(void)
{
    upper_next_ac_capture_id++;
    if (upper_next_ac_capture_id == 0u) {
        upper_next_ac_capture_id = 1u;
    }
    return upper_next_ac_capture_id;
}

static uint8_t UpperControllerPacketAllowed(void)
{
    return (uint8_t)(g_upper_controller_debug.initialized != 0u &&
        ProtocolRuntimeConnectionReady() != 0u &&
        ProtocolRuntimeLinkOnline() != 0u);
}

static uint8_t UpperControllerArmTargetGateFlags(void)
{
    uint8_t flags = 0u;

    if (g_upper_controller_debug.ac_observe_state !=
        UPPER_AC_OBSERVE_HOLDING) {
        flags |= UPPER_ARM_TARGET_GATE_AC_NOT_HOLDING;
    }
    if (g_upper_controller_debug.discrete_state != UPPER_DISCRETE_IDLE) {
        flags |= UPPER_ARM_TARGET_GATE_DISCRETE_BUSY;
    }
    if (g_upper_controller_debug.arm_target_pick_running != 0u) {
        flags |= UPPER_ARM_TARGET_GATE_PICK_RUNNING;
    }
    if (AppArmSidePickPlaceGetStatus() ==
        APP_ARM_SIDE_PICK_PLACE_RUNNING) {
        flags |= UPPER_ARM_TARGET_GATE_SIDE_PICK_RUNNING;
    }
    if (g_upper_controller_debug.observe_area_group !=
        UPPER_OBSERVE_AREA_AC) {
        flags |= UPPER_ARM_TARGET_GATE_BD_PICK_UNSUPPORTED;
    }
    return flags;
}

static Upper_Observe_Area_Group_e UpperControllerAreaGroup(
    Upper_Controller_Area_e area)
{
    if (area == UPPER_CONTROLLER_AREA_A ||
        area == UPPER_CONTROLLER_AREA_C) {
        return UPPER_OBSERVE_AREA_AC;
    }
    if (area == UPPER_CONTROLLER_AREA_B ||
        area == UPPER_CONTROLLER_AREA_D) {
        return UPPER_OBSERVE_AREA_BD;
    }
    return UPPER_OBSERVE_AREA_UNKNOWN;
}

static uint8_t UpperControllerDiscreteCommandValid(
    const Packet_StateMachineCommand *packet)
{
    if (packet == NULL) {
        return 0u;
    }
    if (packet->task_id == UPPER_TASK_GRIPPER ||
        packet->task_id == UPPER_TASK_CAMERA_GIMBAL) {
        return packet->task_status <= UPPER_TASK_STATUS_SECONDARY;
    }
    if (packet->task_id == UPPER_TASK_AC_SIDE_PICK) {
        return packet->task_status <= UPPER_TASK_STATUS_SECONDARY;
    }
    if (packet->task_id == UPPER_TASK_QR_RECOGNITION_POSE) {
        return packet->task_status == UPPER_TASK_STATUS_PRIMARY;
    }
    if (packet->task_id == UPPER_TASK_CURRENT_AREA) {
        return packet->task_status <= UPPER_TASK_STATUS_AREA_D;
    }
    if (packet->task_id == UPPER_TASK_RETURN_INITIAL_POSE) {
        return packet->task_status == UPPER_TASK_STATUS_PRIMARY;
    }
    if (packet->task_id == UPPER_TASK_ARM_RETRACT_POSE) {
        return packet->task_status == UPPER_TASK_STATUS_PRIMARY;
    }
    return 0u;
}

static void UpperControllerApplyCurrentArea(
    const Packet_StateMachineCommand *packet)
{
    Upper_Controller_Area_e new_area =
        (Upper_Controller_Area_e)packet->task_status;

    if (g_upper_controller_debug.current_area_valid != 0u &&
        g_upper_controller_debug.current_area != new_area &&
        g_upper_controller_debug.ac_observe_state ==
            UPPER_AC_OBSERVE_HOLDING) {
        g_upper_controller_debug.ac_observe_state = UPPER_AC_OBSERVE_IDLE;
        g_upper_controller_debug.ac_operation_status =
            (uint8_t)UPPER_AC_OBSERVE_IDLE;
        g_upper_controller_debug.ac_observe_capture_id = 0u;
        g_upper_controller_debug.observe_area_group =
            UPPER_OBSERVE_AREA_UNKNOWN;
        g_upper_controller_debug.observe_area =
            UPPER_CONTROLLER_AREA_UNKNOWN;
    }
    g_upper_controller_debug.current_area =
        new_area;
    g_upper_controller_debug.current_area_valid = 1u;
    g_upper_controller_debug.current_area_update_count++;
    /*
     * AC区地面水果识别需要左右摄像头提前斜向下看。上位机声明当前
     * 区域为A/C后，下位机直接复用camera gimbal的向下角度语义，
     * 同步把左右摄像头置为-45deg；BD区观察/抓取后续单独确认。
     */
    if (UpperControllerAreaGroup(new_area) == UPPER_OBSERVE_AREA_AC) {
        (void)Mg995ServoSetCameraAngles(UPPER_CAMERA_LOOK_DOWN_DEG,
                                        UPPER_CAMERA_LOOK_DOWN_DEG);
    }
    upper_current_area_callback_pending = 1u;
    g_upper_controller_debug.current_area_callback_pending = 1u;
}

static void UpperControllerServiceCurrentAreaCallback(void)
{
    if (upper_current_area_callback_pending == 0u) {
        return;
    }
    (void)UpperControllerSendCallback(
        UPPER_CALLBACK_CURRENT_AREA, UPPER_CALLBACK_EXECUTING);
    (void)UpperControllerSendCallback(
        UPPER_CALLBACK_CURRENT_AREA, UPPER_CALLBACK_COMPLETED);
    upper_current_area_callback_pending = 0u;
    g_upper_controller_debug.current_area_callback_pending = 0u;
    g_upper_controller_debug.discrete_complete_count++;
}

static void UpperControllerSubmitQrPose(void)
{
    Arm_Joint_Command_s command;
    Arm_Command_Result_e result;

    if (g_upper_controller_debug.qr_pose_command_id == 0u) {
        g_upper_controller_debug.qr_pose_command_id =
            AppArmCommandIdNext();
    }
    memset(&command, 0, sizeof(command));
    command.command_id = g_upper_controller_debug.qr_pose_command_id;
    command.move_type = ARM_MOVE_LINEAR;
    command.q_deg[ARM_JOINT_BASE_YAW] = APP_ARM_QR_POSE_Q1_DEG;
    command.q_deg[ARM_JOINT_SHOULDER] = APP_ARM_QR_POSE_Q2_DEG;
    command.q_deg[ARM_JOINT_ELBOW] = APP_ARM_QR_POSE_Q3_DEG;
    command.tool_relative_pitch_valid = 1u;
    command.tool_relative_pitch_deg =
        APP_ARM_QR_POSE_TOOL_REL_PITCH_DEG;
    result = ArmSubmitJointCommand(&command);
    g_upper_controller_debug.qr_pose_submit_result = result;
    if (result == ARM_COMMAND_BUSY || result == ARM_COMMAND_NOT_READY) {
        return;
    }
    if (result != ARM_COMMAND_OK) {
        g_upper_controller_debug.qr_pose_fail_count++;
        g_upper_controller_debug.discrete_invalid_count++;
        g_upper_controller_debug.discrete_state = UPPER_DISCRETE_IDLE;
        g_upper_controller_debug.qr_pose_command_id = 0u;
        return;
    }
    g_upper_controller_debug.qr_pose_start_count++;
    g_upper_controller_debug.discrete_state = UPPER_DISCRETE_RUNNING;
    (void)UpperControllerSendCallback(
        UPPER_CALLBACK_QR_RECOGNITION_POSE,
        UPPER_CALLBACK_EXECUTING);
}

static void UpperControllerPollQrPose(void)
{
    Arm_Host_Status_s status;
    uint32_t command_id = g_upper_controller_debug.qr_pose_command_id;

    if (ArmGetHostStatus(&status) == 0u || command_id == 0u) {
        return;
    }
    if (status.last_command_id != command_id) {
        return;
    }
    if (status.last_command_state == ARM_COMMAND_STATE_COMPLETED &&
        status.last_command_result == ARM_COMMAND_OK) {
        (void)UpperControllerSendCallback(
            UPPER_CALLBACK_QR_RECOGNITION_POSE,
            UPPER_CALLBACK_COMPLETED);
        g_upper_controller_debug.qr_pose_complete_count++;
        g_upper_controller_debug.discrete_complete_count++;
    } else if (status.last_command_state != ARM_COMMAND_STATE_REJECTED &&
               status.last_command_state != ARM_COMMAND_STATE_CANCELLED &&
               status.last_command_state != ARM_COMMAND_STATE_FAULTED) {
        return;
    } else {
        g_upper_controller_debug.qr_pose_submit_result =
            status.last_command_result;
        g_upper_controller_debug.qr_pose_fail_count++;
        g_upper_controller_debug.discrete_invalid_count++;
    }
    g_upper_controller_debug.discrete_state = UPPER_DISCRETE_IDLE;
    g_upper_controller_debug.qr_pose_command_id = 0u;
}

static uint8_t UpperControllerResetHomeActive(void)
{
    return (uint8_t)(
        g_upper_controller_debug.reset_home_state !=
            UPPER_RESET_HOME_IDLE &&
        g_upper_controller_debug.reset_home_state !=
            UPPER_RESET_HOME_FAILED);
}

static void UpperControllerClearTaskStateForReset(uint32_t now_ms)
{
    memset(&upper_pending_discrete, 0, sizeof(upper_pending_discrete));
    memset(&upper_arm_target_place_profile, 0,
           sizeof(upper_arm_target_place_profile));
    upper_arm_target_flow_state = UPPER_ARM_TARGET_FLOW_IDLE;
    upper_current_area_callback_pending = 0u;
    g_upper_controller_debug.current_area_callback_pending = 0u;
    g_upper_controller_debug.gripper_command_id = 0u;
    g_upper_controller_debug.qr_pose_command_id = 0u;
    g_upper_controller_debug.arm_retract_command_id = 0u;
    g_upper_controller_debug.camera_motion_start_tick = 0u;
    g_upper_controller_debug.ac_right_pending = 0u;
    g_upper_controller_debug.ac_operation_status =
        (uint8_t)UPPER_AC_OBSERVE_IDLE;
    g_upper_controller_debug.ac_observe_state = UPPER_AC_OBSERVE_IDLE;
    g_upper_controller_debug.ac_observe_command_id = 0u;
    g_upper_controller_debug.ac_observe_base_command_id = 0u;
    g_upper_controller_debug.ac_observe_target_command_id = 0u;
    g_upper_controller_debug.ac_observe_capture_id = 0u;
    g_upper_controller_debug.arm_target_pick_running = 0u;
    g_upper_controller_debug.arm_target_pick_flow_status =
        APP_ARM_FLOW_IDLE;
    g_arm_target_debug.stage = UPPER_ARM_TARGET_DEBUG_IDLE;
    g_arm_target_debug.gate_flags = 0u;
    AppArmSidePickPlaceAbort(now_ms);
    AppArmFlowAbort(now_ms);
}

static void UpperControllerFailResetHome(void)
{
    UpperControllerLatchArmFailure(
        UPPER_ARM_REJECT_SOURCE_RESET_HOME, g_arm_target_debug.stage, 0u,
        g_upper_controller_debug.reset_home_command_id,
        (uint32_t)g_upper_controller_debug.reset_home_submit_result);
    g_upper_controller_debug.reset_home_fail_count++;
    g_upper_controller_debug.discrete_invalid_count++;
    g_upper_controller_debug.reset_home_state =
        UPPER_RESET_HOME_FAILED;
    g_upper_controller_debug.discrete_state = UPPER_DISCRETE_IDLE;
    g_upper_controller_debug.reset_home_command_id = 0u;
}

static void UpperControllerRequestResetHome(
    const Packet_StateMachineCommand *packet, uint32_t now_ms)
{
    if (UpperControllerResetHomeActive() != 0u) {
        g_upper_controller_debug.discrete_duplicate_count++;
        (void)UpperControllerSendCallback(
            UPPER_CALLBACK_RETURN_INITIAL_POSE,
            UPPER_CALLBACK_EXECUTING);
        return;
    }
    UpperControllerClearTaskStateForReset(now_ms);
    upper_pending_discrete = *packet;
    g_upper_controller_debug.pending_task_id = packet->task_id;
    g_upper_controller_debug.pending_task_status = packet->task_status;
    g_upper_controller_debug.reset_home_request_count++;
    g_upper_controller_debug.reset_home_command_id = 0u;
    g_upper_controller_debug.reset_home_submit_result = ARM_COMMAND_OK;
    g_upper_controller_debug.reset_home_completed_mask = 0u;
    g_upper_controller_debug.reset_home_state =
        UPPER_RESET_HOME_SUBMIT_CANCEL;
    g_upper_controller_debug.discrete_state = UPPER_DISCRETE_RUNNING;
    (void)UpperControllerSendCallback(
        UPPER_CALLBACK_RETURN_INITIAL_POSE, UPPER_CALLBACK_EXECUTING);
}

static void UpperControllerServiceResetHome(void)
{
    Arm_Host_Status_s host;
    Arm_Command_Result_e result;

    switch (g_upper_controller_debug.reset_home_state) {
    case UPPER_RESET_HOME_SUBMIT_CANCEL:
    {
        Arm_Command_s command;

        memset(&command, 0, sizeof(command));
        command.command_id = AppArmCommandIdNext();
        command.type = ARM_COMMAND_TYPE_CANCEL_MOTION;
        result = ArmSubmitCommand(&command);
        g_upper_controller_debug.reset_home_command_id =
            command.command_id;
        g_upper_controller_debug.reset_home_submit_result = result;
        if (result == ARM_COMMAND_OK) {
            g_upper_controller_debug.reset_home_state =
                UPPER_RESET_HOME_WAIT_CANCEL;
        } else if (result != ARM_COMMAND_BUSY) {
            UpperControllerFailResetHome();
        }
        break;
    }

    case UPPER_RESET_HOME_WAIT_CANCEL:
        if (ArmGetHostStatus(&host) == 0u ||
            host.last_command_id !=
                g_upper_controller_debug.reset_home_command_id) {
            break;
        }
        if (host.last_command_state == ARM_COMMAND_STATE_COMPLETED &&
            host.last_command_result == ARM_COMMAND_OK) {
            g_upper_controller_debug.reset_home_completed_mask |=
                UPPER_RESET_HOME_DONE_CANCEL;
            g_upper_controller_debug.reset_home_state =
                UPPER_RESET_HOME_SUBMIT_ARM;
        } else if (host.last_command_state == ARM_COMMAND_STATE_REJECTED ||
                   host.last_command_state == ARM_COMMAND_STATE_CANCELLED ||
                   host.last_command_state == ARM_COMMAND_STATE_FAULTED) {
            g_upper_controller_debug.reset_home_submit_result =
                host.last_command_result;
            UpperControllerFailResetHome();
        }
        break;

    case UPPER_RESET_HOME_SUBMIT_ARM:
        if (ArmGetHostStatus(&host) == 0u ||
            host.ready == 0u || host.busy != 0u) {
            break;
        }
        {
            Arm_Joint_Command_s command;

            memset(&command, 0, sizeof(command));
            command.command_id = AppArmCommandIdNext();
            command.move_type = ARM_MOVE_LINEAR;
            /* 与上电初始化一致：先保持q1，只同步收回q2/q3。 */
            command.q_deg[ARM_JOINT_BASE_YAW] =
                host.q_feedback_deg[ARM_JOINT_BASE_YAW];
            command.q_deg[ARM_JOINT_SHOULDER] = ARM_SAFE_Q2_DEG;
            command.q_deg[ARM_JOINT_ELBOW] = ARM_SAFE_Q3_DEG;
            result = ArmSubmitJointCommand(&command);
            g_upper_controller_debug.reset_home_command_id =
                command.command_id;
            g_upper_controller_debug.reset_home_submit_result = result;
            if (result == ARM_COMMAND_OK) {
                g_upper_controller_debug.reset_home_state =
                    UPPER_RESET_HOME_WAIT_ARM;
            } else if (result != ARM_COMMAND_BUSY &&
                       result != ARM_COMMAND_NOT_READY) {
                UpperControllerFailResetHome();
            }
        }
        break;

    case UPPER_RESET_HOME_WAIT_ARM:
        if (ArmGetHostStatus(&host) == 0u ||
            host.last_command_id !=
                g_upper_controller_debug.reset_home_command_id) {
            break;
        }
        if (host.last_command_state == ARM_COMMAND_STATE_COMPLETED &&
            host.last_command_result == ARM_COMMAND_OK) {
            g_upper_controller_debug.reset_home_completed_mask |=
                UPPER_RESET_HOME_DONE_ARM;
            g_upper_controller_debug.reset_home_state =
                UPPER_RESET_HOME_SUBMIT_BASE;
        } else if (host.last_command_state == ARM_COMMAND_STATE_REJECTED ||
                   host.last_command_state == ARM_COMMAND_STATE_CANCELLED ||
                   host.last_command_state == ARM_COMMAND_STATE_FAULTED) {
            g_upper_controller_debug.reset_home_submit_result =
                host.last_command_result;
            UpperControllerFailResetHome();
        }
        break;

    case UPPER_RESET_HOME_SUBMIT_BASE:
        if (ArmGetHostStatus(&host) == 0u ||
            host.ready == 0u || host.busy != 0u) {
            break;
        }
        {
            Arm_Joint_Command_s command;

            memset(&command, 0, sizeof(command));
            command.command_id = AppArmCommandIdNext();
            command.move_type = ARM_MOVE_LINEAR;
            command.q_deg[ARM_JOINT_BASE_YAW] = ARM_SAFE_Q1_DEG;
            command.q_deg[ARM_JOINT_SHOULDER] = ARM_SAFE_Q2_DEG;
            command.q_deg[ARM_JOINT_ELBOW] = ARM_SAFE_Q3_DEG;
            result = ArmSubmitJointCommand(&command);
            g_upper_controller_debug.reset_home_command_id =
                command.command_id;
            g_upper_controller_debug.reset_home_submit_result = result;
            if (result == ARM_COMMAND_OK) {
                g_upper_controller_debug.reset_home_state =
                    UPPER_RESET_HOME_WAIT_BASE;
            } else if (result != ARM_COMMAND_BUSY &&
                       result != ARM_COMMAND_NOT_READY) {
                UpperControllerFailResetHome();
            }
        }
        break;

    case UPPER_RESET_HOME_WAIT_BASE:
        if (ArmGetHostStatus(&host) == 0u ||
            host.last_command_id !=
                g_upper_controller_debug.reset_home_command_id) {
            break;
        }
        if (host.last_command_state == ARM_COMMAND_STATE_COMPLETED &&
            host.last_command_result == ARM_COMMAND_OK) {
            g_upper_controller_debug.reset_home_completed_mask |=
                UPPER_RESET_HOME_DONE_BASE;
            g_upper_controller_debug.reset_home_state =
                UPPER_RESET_HOME_SUBMIT_PITCH;
        } else if (host.last_command_state == ARM_COMMAND_STATE_REJECTED ||
                   host.last_command_state == ARM_COMMAND_STATE_CANCELLED ||
                   host.last_command_state == ARM_COMMAND_STATE_FAULTED) {
            g_upper_controller_debug.reset_home_submit_result =
                host.last_command_result;
            UpperControllerFailResetHome();
        }
        break;

    case UPPER_RESET_HOME_SUBMIT_PITCH:
        if (ArmGetHostStatus(&host) == 0u ||
            host.ready == 0u || host.busy != 0u) {
            break;
        }
        {
            Arm_Command_s command;
            const float home_q_deg[3] = {
                ARM_SAFE_Q1_DEG, ARM_SAFE_Q2_DEG, ARM_SAFE_Q3_DEG
            };

            memset(&command, 0, sizeof(command));
            command.command_id = AppArmCommandIdNext();
            command.type = ARM_COMMAND_TYPE_TOOL;
            command.payload.tool.action = ARM_TOOL_ACTION_SET_PITCH;
            command.payload.tool.pitch_deg =
                ArmToolSmallLinkPitchFromJoint(home_q_deg);
            result = ArmSubmitCommand(&command);
            g_upper_controller_debug.reset_home_command_id =
                command.command_id;
            g_upper_controller_debug.reset_home_submit_result = result;
            if (result == ARM_COMMAND_OK) {
                g_upper_controller_debug.reset_home_state =
                    UPPER_RESET_HOME_WAIT_PITCH;
            } else if (result != ARM_COMMAND_BUSY &&
                       result != ARM_COMMAND_NOT_READY) {
                UpperControllerFailResetHome();
            }
        }
        break;

    case UPPER_RESET_HOME_WAIT_PITCH:
        if (ArmGetHostStatus(&host) == 0u ||
            host.last_command_id !=
                g_upper_controller_debug.reset_home_command_id) {
            break;
        }
        if (host.last_command_state == ARM_COMMAND_STATE_COMPLETED &&
            host.last_command_result == ARM_COMMAND_OK) {
            g_upper_controller_debug.reset_home_completed_mask |=
                UPPER_RESET_HOME_DONE_PITCH;
            g_upper_controller_debug.reset_home_state =
                UPPER_RESET_HOME_SUBMIT_GRIPPER;
        } else if (host.last_command_state == ARM_COMMAND_STATE_REJECTED ||
                   host.last_command_state == ARM_COMMAND_STATE_CANCELLED ||
                   host.last_command_state == ARM_COMMAND_STATE_FAULTED) {
            g_upper_controller_debug.reset_home_submit_result =
                host.last_command_result;
            UpperControllerFailResetHome();
        }
        break;

    case UPPER_RESET_HOME_SUBMIT_GRIPPER:
        if (ArmGetHostStatus(&host) == 0u ||
            host.ready == 0u || host.busy != 0u) {
            break;
        }
        {
            Arm_Command_s command;

            memset(&command, 0, sizeof(command));
            command.command_id = AppArmCommandIdNext();
            command.type = ARM_COMMAND_TYPE_TOOL;
            command.payload.tool.action = ARM_TOOL_ACTION_GRIPPER_READY;
            result = ArmSubmitCommand(&command);
            g_upper_controller_debug.reset_home_command_id =
                command.command_id;
            g_upper_controller_debug.reset_home_submit_result = result;
            if (result == ARM_COMMAND_OK) {
                g_upper_controller_debug.reset_home_state =
                    UPPER_RESET_HOME_WAIT_GRIPPER;
            } else if (result != ARM_COMMAND_BUSY &&
                       result != ARM_COMMAND_NOT_READY) {
                UpperControllerFailResetHome();
            }
        }
        break;

    case UPPER_RESET_HOME_WAIT_GRIPPER:
        if (ArmGetHostStatus(&host) == 0u ||
            host.last_command_id !=
                g_upper_controller_debug.reset_home_command_id) {
            break;
        }
        if (host.last_command_state == ARM_COMMAND_STATE_COMPLETED &&
            host.last_command_result == ARM_COMMAND_OK) {
            g_upper_controller_debug.reset_home_completed_mask |=
                UPPER_RESET_HOME_DONE_GRIPPER;
            (void)UpperControllerSendCallback(
                UPPER_CALLBACK_RETURN_INITIAL_POSE,
                UPPER_CALLBACK_COMPLETED);
            g_upper_controller_debug.reset_home_complete_count++;
            g_upper_controller_debug.discrete_complete_count++;
            g_upper_controller_debug.reset_home_state =
                UPPER_RESET_HOME_IDLE;
            g_upper_controller_debug.discrete_state =
                UPPER_DISCRETE_IDLE;
            g_upper_controller_debug.reset_home_command_id = 0u;
        } else if (host.last_command_state == ARM_COMMAND_STATE_REJECTED ||
                   host.last_command_state == ARM_COMMAND_STATE_CANCELLED ||
                   host.last_command_state == ARM_COMMAND_STATE_FAULTED) {
            g_upper_controller_debug.reset_home_submit_result =
                host.last_command_result;
            UpperControllerFailResetHome();
        }
        break;

    case UPPER_RESET_HOME_IDLE:
    case UPPER_RESET_HOME_FAILED:
    default:
        break;
    }
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

static void UpperControllerSubmitArmRetractPose(void)
{
    Arm_Joint_Command_s command;
    Arm_Command_Result_e result;

    if (g_upper_controller_debug.arm_retract_command_id == 0u) {
        g_upper_controller_debug.arm_retract_command_id =
            AppArmCommandIdNext();
    }
    memset(&command, 0, sizeof(command));
    command.command_id =
        g_upper_controller_debug.arm_retract_command_id;
    command.move_type = ARM_MOVE_LINEAR;
    command.q_deg[ARM_JOINT_BASE_YAW] =
        APP_ARM_CHASSIS_CLEARANCE_Q1_DEG;
    command.q_deg[ARM_JOINT_SHOULDER] =
        APP_ARM_CHASSIS_CLEARANCE_Q2_DEG;
    command.q_deg[ARM_JOINT_ELBOW] =
        APP_ARM_CHASSIS_CLEARANCE_Q3_DEG;
    command.tool_relative_pitch_valid = 1u;
    command.tool_relative_pitch_deg =
        APP_ARM_CHASSIS_CLEARANCE_TOOL_REL_PITCH_DEG;
    result = ArmSubmitJointCommand(&command);
    g_upper_controller_debug.arm_retract_submit_result = result;
    if (result == ARM_COMMAND_BUSY ||
        result == ARM_COMMAND_NOT_READY) {
        return;
    }
    if (result != ARM_COMMAND_OK) {
        g_upper_controller_debug.arm_retract_fail_count++;
        g_upper_controller_debug.discrete_invalid_count++;
        g_upper_controller_debug.discrete_state = UPPER_DISCRETE_IDLE;
        g_upper_controller_debug.arm_retract_command_id = 0u;
        return;
    }
    g_upper_controller_debug.arm_retract_start_count++;
    g_upper_controller_debug.discrete_state = UPPER_DISCRETE_RUNNING;
    (void)UpperControllerSendCallback(
        UPPER_CALLBACK_ARM_RETRACT_POSE, UPPER_CALLBACK_EXECUTING);
}

static void UpperControllerPollArmRetractPose(void)
{
    Arm_Host_Status_s status;
    uint32_t command_id =
        g_upper_controller_debug.arm_retract_command_id;

    if (ArmGetHostStatus(&status) == 0u || command_id == 0u) {
        return;
    }
    if (status.last_command_id != command_id) {
        return;
    }
    if (status.last_command_state == ARM_COMMAND_STATE_COMPLETED &&
        status.last_command_result == ARM_COMMAND_OK) {
        (void)UpperControllerSendCallback(
            UPPER_CALLBACK_ARM_RETRACT_POSE,
            UPPER_CALLBACK_COMPLETED);
        g_upper_controller_debug.arm_retract_complete_count++;
        g_upper_controller_debug.discrete_complete_count++;
    } else if (status.last_command_state != ARM_COMMAND_STATE_REJECTED &&
               status.last_command_state != ARM_COMMAND_STATE_CANCELLED &&
               status.last_command_state != ARM_COMMAND_STATE_FAULTED) {
        return;
    } else {
        g_upper_controller_debug.arm_retract_submit_result =
            status.last_command_result;
        g_upper_controller_debug.arm_retract_fail_count++;
        g_upper_controller_debug.discrete_invalid_count++;
    }
    g_upper_controller_debug.discrete_state = UPPER_DISCRETE_IDLE;
    g_upper_controller_debug.arm_retract_command_id = 0u;
}

static App_Fruit_Side_e UpperControllerAcSideFromStatus(uint8_t status)
{
    return status == UPPER_TASK_STATUS_SECONDARY ?
        APP_FRUIT_SIDE_RIGHT : APP_FRUIT_SIDE_LEFT;
}

static float UpperControllerObservationBaseQ1Deg(
    Upper_Observe_Area_Group_e group, App_Fruit_Side_e side)
{
    if (group == UPPER_OBSERVE_AREA_AC) {
        return side == APP_FRUIT_SIDE_RIGHT ?
            APP_ARM_AC_OBSERVATION_RIGHT_BASE_Q1_DEG :
            APP_ARM_AC_OBSERVATION_LEFT_BASE_Q1_DEG;
    }
    return side == APP_FRUIT_SIDE_RIGHT ?
        APP_ARM_BD_OBSERVATION_RIGHT_BASE_Q1_DEG :
        APP_ARM_BD_OBSERVATION_LEFT_BASE_Q1_DEG;
}

static float UpperControllerObservationStagingQ2Deg(
    Upper_Observe_Area_Group_e group)
{
    return group == UPPER_OBSERVE_AREA_AC ?
        APP_ARM_AC_OBSERVATION_STAGING_Q2_DEG :
        APP_ARM_BD_OBSERVATION_STAGING_Q2_DEG;
}

static float UpperControllerObservationStagingQ3Deg(
    Upper_Observe_Area_Group_e group)
{
    return group == UPPER_OBSERVE_AREA_AC ?
        APP_ARM_AC_OBSERVATION_STAGING_Q3_DEG :
        APP_ARM_BD_OBSERVATION_STAGING_Q3_DEG;
}

static float UpperControllerObservationToolPitchDeg(
    Upper_Observe_Area_Group_e group)
{
    return group == UPPER_OBSERVE_AREA_AC ?
        APP_ARM_AC_OBSERVATION_TOOL_PITCH_DEG :
        APP_ARM_BD_OBSERVATION_TOOL_PITCH_DEG;
}

static float UpperControllerObservationSpeedMmS(
    Upper_Observe_Area_Group_e group)
{
    return group == UPPER_OBSERVE_AREA_AC ?
        APP_ARM_AC_OBSERVATION_SPEED_MM_S :
        APP_ARM_BD_OBSERVATION_SPEED_MM_S;
}

static uint8_t UpperControllerLoadObservationTarget(App_Fruit_Side_e side)
{
    Upper_Controller_Area_e area = g_upper_controller_debug.current_area;
    Upper_Observe_Area_Group_e group;

    if (g_upper_controller_debug.current_area_valid == 0u) {
        g_upper_controller_debug.observe_area_group =
            UPPER_OBSERVE_AREA_UNKNOWN;
        g_upper_controller_debug.observe_area =
            UPPER_CONTROLLER_AREA_UNKNOWN;
        return 0u;
    }
    group = UpperControllerAreaGroup(area);
    if (group == UPPER_OBSERVE_AREA_UNKNOWN) {
        g_upper_controller_debug.observe_area_group = group;
        g_upper_controller_debug.observe_area = area;
        return 0u;
    }
    g_upper_controller_debug.observe_area_group = group;
    g_upper_controller_debug.observe_area = area;
    if (group == UPPER_OBSERVE_AREA_AC) {
        if (side == APP_FRUIT_SIDE_RIGHT) {
            g_upper_controller_debug.ac_observe_target_center_mm[0] =
                APP_ARM_AC_OBSERVATION_RIGHT_X_MM;
            g_upper_controller_debug.ac_observe_target_center_mm[1] =
                APP_ARM_AC_OBSERVATION_RIGHT_Y_MM;
            g_upper_controller_debug.ac_observe_target_center_mm[2] =
                APP_ARM_AC_OBSERVATION_RIGHT_Z_MM;
        } else {
            g_upper_controller_debug.ac_observe_target_center_mm[0] =
                APP_ARM_AC_OBSERVATION_LEFT_X_MM;
            g_upper_controller_debug.ac_observe_target_center_mm[1] =
                APP_ARM_AC_OBSERVATION_LEFT_Y_MM;
            g_upper_controller_debug.ac_observe_target_center_mm[2] =
                APP_ARM_AC_OBSERVATION_LEFT_Z_MM;
        }
    } else if (side == APP_FRUIT_SIDE_RIGHT) {
        g_upper_controller_debug.ac_observe_target_center_mm[0] =
            APP_ARM_BD_OBSERVATION_RIGHT_X_MM;
        g_upper_controller_debug.ac_observe_target_center_mm[1] =
            APP_ARM_BD_OBSERVATION_RIGHT_Y_MM;
        g_upper_controller_debug.ac_observe_target_center_mm[2] =
            APP_ARM_BD_OBSERVATION_RIGHT_Z_MM;
    } else {
        g_upper_controller_debug.ac_observe_target_center_mm[0] =
            APP_ARM_BD_OBSERVATION_LEFT_X_MM;
        g_upper_controller_debug.ac_observe_target_center_mm[1] =
            APP_ARM_BD_OBSERVATION_LEFT_Y_MM;
        g_upper_controller_debug.ac_observe_target_center_mm[2] =
            APP_ARM_BD_OBSERVATION_LEFT_Z_MM;
    }
    g_upper_controller_debug.ac_observe_target_tool_pitch_deg =
        UpperControllerObservationToolPitchDeg(group);
    return 1u;
}

static void UpperControllerFailAcObservation(void)
{
    g_upper_controller_debug.ac_fail_count++;
    g_upper_controller_debug.ac_observe_fail_count++;
    g_upper_controller_debug.discrete_invalid_count++;
    g_upper_controller_debug.ac_observe_state = UPPER_AC_OBSERVE_FAILED;
    g_upper_controller_debug.discrete_state = UPPER_DISCRETE_IDLE;
}

static void UpperControllerStartAcObservation(uint32_t now_ms)
{
    App_Fruit_Side_e side =
        UpperControllerAcSideFromStatus(upper_pending_discrete.task_status);

    (void)now_ms;
    if (AppArmSidePickPlaceGetStatus() ==
            APP_ARM_SIDE_PICK_PLACE_RUNNING ||
        g_upper_controller_debug.arm_target_pick_running != 0u) {
        g_upper_controller_debug.discrete_busy_count++;
        return;
    }
    g_upper_controller_debug.ac_right_pending = 0u;
    g_upper_controller_debug.ac_active_side = (uint8_t)side;
    g_upper_controller_debug.ac_operation_status =
        (uint8_t)UPPER_AC_OBSERVE_WAIT_READY;
    g_upper_controller_debug.ac_observe_base_command_id = 0u;
    g_upper_controller_debug.ac_observe_target_command_id = 0u;
    g_upper_controller_debug.ac_observe_command_id = 0u;
    g_upper_controller_debug.ac_observe_capture_id = 0u;
    if (UpperControllerLoadObservationTarget(side) == 0u) {
        g_upper_controller_debug.ac_start_result =
            (uint8_t)ARM_COMMAND_INVALID;
        UpperControllerFailAcObservation();
        return;
    }
    g_upper_controller_debug.ac_start_count++;
    g_upper_controller_debug.ac_observe_start_count++;
    g_upper_controller_debug.ac_observe_state =
        UPPER_AC_OBSERVE_WAIT_READY;
    (void)UpperControllerSendCallback(
        UPPER_CALLBACK_AC_SIDE_PICK, UPPER_CALLBACK_EXECUTING);
    g_upper_controller_debug.discrete_state = UPPER_DISCRETE_RUNNING;
}

static void UpperControllerPollAcObservation(uint32_t now_ms)
{
    Arm_Host_Status_s host;
    Arm_Command_Result_e result;
    App_Fruit_Side_e side =
        (App_Fruit_Side_e)g_upper_controller_debug.ac_active_side;
    Upper_Observe_Area_Group_e group =
        g_upper_controller_debug.observe_area_group;

    if (ArmGetHostStatus(&host) == 0u) {
        return;
    }
    if (host.state == ARM_HOST_STATE_FAULT ||
        host.state == ARM_HOST_STATE_ESTOP) {
        UpperControllerFailAcObservation();
        return;
    }
    switch (g_upper_controller_debug.ac_observe_state) {
    case UPPER_AC_OBSERVE_WAIT_READY:
        if (host.ready == 0u || host.busy != 0u) {
            break;
        }
        if (group == UPPER_OBSERVE_AREA_AC) {
            Arm_Joint_Command_s command;
            Arm_Position_s target_center;
            Arm_Tool_Center_IK_Result_s ik_result;
            float waypoint_q_deg[3];
            float tool_pitch_deg;
            Arm_IK_Status_e ik_status;

            memset(&command, 0, sizeof(command));
            memset(&ik_result, 0, sizeof(ik_result));
            target_center.x_mm =
                g_upper_controller_debug.ac_observe_target_center_mm[0];
            target_center.y_mm =
                g_upper_controller_debug.ac_observe_target_center_mm[1];
            target_center.z_mm =
                g_upper_controller_debug.ac_observe_target_center_mm[2];
            tool_pitch_deg = UpperControllerObservationToolPitchDeg(group);
            memcpy(waypoint_q_deg, host.q_feedback_deg,
                   sizeof(waypoint_q_deg));
            waypoint_q_deg[ARM_JOINT_BASE_YAW] =
                UpperControllerObservationBaseQ1Deg(group, side);
            waypoint_q_deg[ARM_JOINT_SHOULDER] =
                UpperControllerObservationStagingQ2Deg(group);
            waypoint_q_deg[ARM_JOINT_ELBOW] =
                UpperControllerObservationStagingQ3Deg(group);
            /*
             * 观察姿态原先拆成“关节安全位->工具中心目标”两条命令，会在
             * 中间安全位等待完整到位。这里先用安全位作为IK seed算出最终
             * 观察关节角，再把安全位合并为同一条route的waypoint，让轨迹
             * 层使用中间点宽松到位判定连续切段，减少HOME到观察位卡顿。
             */
            ik_status = ArmInverseKinematicsToolCenter(
                &target_center, tool_pitch_deg, waypoint_q_deg,
                &ik_result);
            if (ik_status != ARM_IK_OK) {
                g_upper_controller_debug.ac_start_result =
                    (uint8_t)ARM_COMMAND_PREFLIGHT_FAILED;
                UpperControllerFailAcObservation();
                break;
            }
            command.command_id = AppArmCommandIdNext();
            command.move_type = ARM_MOVE_LINEAR;
            memcpy(command.q_deg, ik_result.q_deg,
                   sizeof(command.q_deg));
            command.waypoint_valid = 1u;
            memcpy(command.waypoint_q_deg, waypoint_q_deg,
                   sizeof(command.waypoint_q_deg));
            command.tool_relative_pitch_valid = 1u;
            command.tool_relative_pitch_deg =
                tool_pitch_deg -
                ArmToolSmallLinkPitchFromJoint(command.q_deg);
            result = ArmSubmitJointCommand(&command);
            g_upper_controller_debug.ac_observe_command_id =
                command.command_id;
            g_upper_controller_debug.ac_observe_base_command_id =
                command.command_id;
            g_upper_controller_debug.ac_observe_target_command_id =
                command.command_id;
            if (result == ARM_COMMAND_OK) {
                g_upper_controller_debug.ac_observe_state =
                    UPPER_AC_OBSERVE_TARGET_SUBMITTED;
                g_upper_controller_debug.ac_operation_status =
                    (uint8_t)UPPER_AC_OBSERVE_TARGET_SUBMITTED;
            } else if (result != ARM_COMMAND_BUSY &&
                       result != ARM_COMMAND_NOT_READY) {
                g_upper_controller_debug.ac_start_result = (uint8_t)result;
                UpperControllerFailAcObservation();
            }
        } else {
            Arm_Joint_Command_s command;
            float q_deg[3];

            memset(&command, 0, sizeof(command));
            command.command_id = AppArmCommandIdNext();
            command.move_type = ARM_MOVE_LINEAR;
            memcpy(q_deg, host.q_feedback_deg, sizeof(q_deg));
            q_deg[ARM_JOINT_BASE_YAW] =
                UpperControllerObservationBaseQ1Deg(group, side);
            q_deg[ARM_JOINT_SHOULDER] =
                UpperControllerObservationStagingQ2Deg(group);
            q_deg[ARM_JOINT_ELBOW] =
                UpperControllerObservationStagingQ3Deg(group);
            memcpy(command.q_deg, q_deg, sizeof(command.q_deg));
            command.tool_relative_pitch_valid = 1u;
            command.tool_relative_pitch_deg =
                UpperControllerObservationToolPitchDeg(group) -
                ArmToolSmallLinkPitchFromJoint(q_deg);
            result = ArmSubmitJointCommand(&command);
            g_upper_controller_debug.ac_observe_command_id =
                command.command_id;
            g_upper_controller_debug.ac_observe_base_command_id =
                command.command_id;
            if (result == ARM_COMMAND_OK) {
                g_upper_controller_debug.ac_observe_state =
                    UPPER_AC_OBSERVE_BASE_SUBMITTED;
                g_upper_controller_debug.ac_operation_status =
                    (uint8_t)UPPER_AC_OBSERVE_BASE_SUBMITTED;
            } else if (result != ARM_COMMAND_BUSY &&
                       result != ARM_COMMAND_NOT_READY) {
                g_upper_controller_debug.ac_start_result = (uint8_t)result;
                UpperControllerFailAcObservation();
            }
        }
        break;

    case UPPER_AC_OBSERVE_BASE_SUBMITTED:
        if (host.last_command_id !=
            g_upper_controller_debug.ac_observe_base_command_id) {
            break;
        }
        if (host.last_command_state == ARM_COMMAND_STATE_COMPLETED &&
            host.last_command_result == ARM_COMMAND_OK) {
            g_upper_controller_debug.ac_observe_state =
                UPPER_AC_OBSERVE_SUBMIT_TARGET;
            g_upper_controller_debug.ac_operation_status =
                (uint8_t)UPPER_AC_OBSERVE_SUBMIT_TARGET;
        } else if (host.last_command_state == ARM_COMMAND_STATE_REJECTED ||
                   host.last_command_state == ARM_COMMAND_STATE_CANCELLED ||
                   host.last_command_state == ARM_COMMAND_STATE_FAULTED) {
            UpperControllerFailAcObservation();
        }
        break;

    case UPPER_AC_OBSERVE_SUBMIT_TARGET:
        if (host.ready == 0u || host.busy != 0u) {
            break;
        }
        {
            Arm_Tool_Center_Command_s command;

            memset(&command, 0, sizeof(command));
            command.command_id = AppArmCommandIdNext();
            command.move_type = ARM_MOVE_LINEAR;
            command.target_center_mm.x_mm =
                g_upper_controller_debug.ac_observe_target_center_mm[0];
            command.target_center_mm.y_mm =
                g_upper_controller_debug.ac_observe_target_center_mm[1];
            command.target_center_mm.z_mm =
                g_upper_controller_debug.ac_observe_target_center_mm[2];
            command.max_speed_mm_s =
                UpperControllerObservationSpeedMmS(group);
            command.tool_pitch_valid = 1u;
            command.tool_pitch_deg =
                UpperControllerObservationToolPitchDeg(group);
            result = ArmSubmitToolCenterCommand(&command);
            g_upper_controller_debug.ac_observe_command_id =
                command.command_id;
            g_upper_controller_debug.ac_observe_target_command_id =
                command.command_id;
            if (result == ARM_COMMAND_OK) {
                g_upper_controller_debug.ac_observe_state =
                    UPPER_AC_OBSERVE_TARGET_SUBMITTED;
                g_upper_controller_debug.ac_operation_status =
                    (uint8_t)UPPER_AC_OBSERVE_TARGET_SUBMITTED;
            } else if (result != ARM_COMMAND_BUSY &&
                       result != ARM_COMMAND_NOT_READY) {
                g_upper_controller_debug.ac_start_result = (uint8_t)result;
                UpperControllerFailAcObservation();
            }
        }
        break;

    case UPPER_AC_OBSERVE_TARGET_SUBMITTED:
        if (host.last_command_id !=
            g_upper_controller_debug.ac_observe_target_command_id) {
            break;
        }
        if (host.last_command_state == ARM_COMMAND_STATE_COMPLETED &&
            host.last_command_result == ARM_COMMAND_OK) {
            uint32_t capture_id = UpperControllerNextAcCaptureId();
            Camera_Target_Transform_Status_e status =
                UpperControllerCaptureCameraPose(capture_id, now_ms);

            if (status == CAMERA_TARGET_STATUS_OK) {
                g_upper_controller_debug.ac_observe_capture_id =
                    capture_id;
                g_upper_controller_debug.ac_observe_state =
                    UPPER_AC_OBSERVE_HOLDING;
                g_upper_controller_debug.ac_operation_status =
                    (uint8_t)UPPER_AC_OBSERVE_HOLDING;
                g_upper_controller_debug.ac_observe_complete_count++;
                g_upper_controller_debug.ac_complete_count++;
                g_upper_controller_debug.discrete_complete_count++;
                (void)UpperControllerSendCallback(
                    UPPER_CALLBACK_AC_SIDE_PICK,
                    UPPER_CALLBACK_COMPLETED);
                g_upper_controller_debug.discrete_state =
                    UPPER_DISCRETE_IDLE;
            } else {
                UpperControllerFailAcObservation();
            }
        } else if (host.last_command_state == ARM_COMMAND_STATE_REJECTED ||
                   host.last_command_state == ARM_COMMAND_STATE_CANCELLED ||
                   host.last_command_state == ARM_COMMAND_STATE_FAULTED) {
            UpperControllerFailAcObservation();
        }
        break;

    case UPPER_AC_OBSERVE_HOLDING:
    case UPPER_AC_OBSERVE_FAILED:
    case UPPER_AC_OBSERVE_IDLE:
    default:
        break;
    }
}

static void UpperControllerPollArmTargetPick(void)
{
    App_Arm_Flow_Status_e status;

    if (g_upper_controller_debug.arm_target_pick_running == 0u) {
        return;
    }
    status = AppArmFlowGetStatus();
    g_upper_controller_debug.arm_target_pick_flow_status = status;
    if (status == APP_ARM_FLOW_RUNNING) {
        return;
    }
    if (status == APP_ARM_FLOW_DONE) {
        if (upper_arm_target_flow_state == UPPER_ARM_TARGET_FLOW_PICK) {
            App_Arm_Flow_Start_Result_e start_result =
                AppArmFlowStartPlace(&upper_arm_target_place_profile,
                                     HAL_GetTick());

            g_arm_target_debug.stage =
                UPPER_ARM_TARGET_DEBUG_PICK_DONE;
            if (start_result == APP_ARM_FLOW_START_ACCEPTED) {
                upper_arm_target_flow_state =
                    UPPER_ARM_TARGET_FLOW_PLACE;
                g_upper_controller_debug.arm_target_pick_flow_status =
                    APP_ARM_FLOW_RUNNING;
                g_arm_target_debug.stage =
                    UPPER_ARM_TARGET_DEBUG_PLACE_STARTED;
                return;
            }
            if (start_result == APP_ARM_FLOW_START_BUSY) {
                return;
            }
            UpperControllerHandleArmTargetFailure(
                UPPER_ARM_TARGET_DEBUG_PLACE_REJECTED);
            return;
        }
        g_upper_controller_debug.arm_target_pick_running = 0u;
        upper_arm_target_flow_state = UPPER_ARM_TARGET_FLOW_IDLE;
        (void)UpperControllerSendCallback(
            UPPER_CALLBACK_ARM_TARGET, UPPER_CALLBACK_COMPLETED);
        g_arm_target_debug.stage = UPPER_ARM_TARGET_DEBUG_PLACE_DONE;
        g_upper_controller_debug.arm_target_pick_complete_count++;
        g_upper_controller_debug.discrete_complete_count++;
    } else {
        UpperControllerHandleArmTargetFailure(
            upper_arm_target_flow_state == UPPER_ARM_TARGET_FLOW_PLACE ?
                UPPER_ARM_TARGET_DEBUG_PLACE_REJECTED :
                UPPER_ARM_TARGET_DEBUG_PICK_FAILED);
    }
}

void UpperControllerBridgeInit(void)
{
    memset(&g_upper_controller_debug, 0,
           sizeof(g_upper_controller_debug));
    memset(&g_arm_target_debug, 0, sizeof(g_arm_target_debug));
    memset(&g_upper_arm_reject_diagnostic, 0,
           sizeof(g_upper_arm_reject_diagnostic));
    memset(&upper_pending_discrete, 0,
           sizeof(upper_pending_discrete));
    memset(&upper_arm_target_place_profile, 0,
           sizeof(upper_arm_target_place_profile));
    upper_next_chassis_command_id = UPPER_CHASSIS_COMMAND_ID_SEED;
    upper_next_ac_capture_id = UPPER_AC_CAPTURE_ID_SEED;
    upper_current_area_callback_pending = 0u;
    upper_arm_target_flow_state = UPPER_ARM_TARGET_FLOW_IDLE;
    g_upper_controller_debug.current_area =
        UPPER_CONTROLLER_AREA_UNKNOWN;
    g_upper_controller_debug.ac_observe_state = UPPER_AC_OBSERVE_IDLE;
    CameraTargetTransformInit();
    g_upper_controller_debug.initialized = 1u;
}

void UpperControllerBridgeTask(uint32_t now_ms)
{
    if (g_upper_controller_debug.initialized == 0u) {
        return;
    }
    if (UpperControllerResetHomeActive() != 0u) {
        UpperControllerServiceResetHome();
        return;
    }
    UpperControllerServiceCurrentAreaCallback();
    UpperControllerPollArmTargetPick();
    if (g_upper_controller_debug.discrete_state ==
            UPPER_DISCRETE_PENDING) {
        if (upper_pending_discrete.task_id ==
            UPPER_TASK_CAMERA_GIMBAL) {
            UpperControllerRunCameraCommand(now_ms);
        } else if (upper_pending_discrete.task_id ==
                   UPPER_TASK_AC_SIDE_PICK) {
            UpperControllerStartAcObservation(now_ms);
        } else if (upper_pending_discrete.task_id ==
                   UPPER_TASK_QR_RECOGNITION_POSE) {
            UpperControllerSubmitQrPose();
        } else if (upper_pending_discrete.task_id ==
                   UPPER_TASK_ARM_RETRACT_POSE) {
            UpperControllerSubmitArmRetractPose();
        } else {
            UpperControllerSubmitGripperCommand();
        }
    } else if (g_upper_controller_debug.discrete_state ==
               UPPER_DISCRETE_RUNNING) {
        if (upper_pending_discrete.task_id ==
            UPPER_TASK_CAMERA_GIMBAL) {
            UpperControllerPollCameraCommand(now_ms);
        } else if (upper_pending_discrete.task_id ==
                   UPPER_TASK_AC_SIDE_PICK) {
            UpperControllerPollAcObservation(now_ms);
        } else if (upper_pending_discrete.task_id ==
                   UPPER_TASK_QR_RECOGNITION_POSE) {
            UpperControllerPollQrPose();
        } else if (upper_pending_discrete.task_id ==
                   UPPER_TASK_ARM_RETRACT_POSE) {
            UpperControllerPollArmRetractPose();
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
        UpperControllerDiscreteCommandValid(packet) == 0u) {
        g_upper_controller_debug.discrete_invalid_count++;
        return;
    }
    g_upper_controller_debug.discrete_rx_count++;
    if (packet->task_id == UPPER_TASK_RETURN_INITIAL_POSE) {
        UpperControllerRequestResetHome(packet, HAL_GetTick());
        return;
    }
    if (packet->task_id == UPPER_TASK_CURRENT_AREA) {
        UpperControllerApplyCurrentArea(packet);
        return;
    }
    if (g_upper_controller_debug.arm_target_pick_running != 0u) {
        g_upper_controller_debug.discrete_busy_count++;
        return;
    }
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
    if (packet->task_id == UPPER_TASK_QR_RECOGNITION_POSE) {
        g_upper_controller_debug.qr_pose_request_count++;
    }
    g_upper_controller_debug.discrete_state = UPPER_DISCRETE_PENDING;
}

void on_receive_ArmTarget(const Packet_ArmTarget *packet)
{
    Camera_Target_Transform_Result_s transform_result;
    Camera_Target_Transform_Status_e transform_status;
    const Camera_Arm_Pose_Snapshot_s *snapshot;
    uint32_t now_ms;
    uint32_t capture_id;
    uint8_t gate_flags;
    uint8_t valid;

    ProtocolRuntimeNotifyApplicationRx();
    g_upper_controller_debug.arm_target_rx_count++;
    g_arm_target_debug.rx_count++;
    g_arm_target_debug.stage = UPPER_ARM_TARGET_DEBUG_RX;
    g_arm_target_debug.transform_status = CAMERA_TARGET_STATUS_OK;
    g_arm_target_debug.pose_age_ms = UPPER_ARM_TARGET_POSE_AGE_UNKNOWN;
    g_arm_target_debug.gate_flags = 0u;
    g_arm_target_debug.pick_start_result = 0u;
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
        g_arm_target_debug.stage = UPPER_ARM_TARGET_DEBUG_INVALID;
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
    g_upper_controller_debug.arm_target_reference_mm[0] = NAN;
    g_upper_controller_debug.arm_target_reference_mm[1] = NAN;
    g_upper_controller_debug.arm_target_reference_mm[2] = NAN;
    g_upper_controller_debug.arm_target_base_mm[0] = NAN;
    g_upper_controller_debug.arm_target_base_mm[1] = NAN;
    g_upper_controller_debug.arm_target_base_mm[2] = NAN;
    g_upper_controller_debug.arm_target_approach_y_raw_mm = NAN;
    g_upper_controller_debug.arm_target_approach_y_command_mm = NAN;
    g_upper_controller_debug.arm_target_near_y_min_mm =
        APP_ARM_AC_CLOSED_LOOP_NEAR_Y_MIN_MM;
    g_upper_controller_debug.arm_target_near_y_shortfall_mm = NAN;
    g_upper_controller_debug.arm_target_near_y_clamped = 0u;
    g_upper_controller_debug.arm_target_advance_requested_mm =
        APP_ARM_AC_CLOSED_LOOP_ADVANCE_MM;
    g_upper_controller_debug.arm_target_advance_selected_mm = NAN;
    g_upper_controller_debug.arm_target_advance_reduced = 0u;
    g_upper_controller_debug.arm_target_advance_approach_failed = 0u;
    g_upper_controller_debug.arm_target_advance_reject_reason =
        APP_ARM_ADVANCE_REJECT_NONE;
    g_upper_controller_debug.arm_target_advance_planner_status = 0u;
    g_upper_controller_debug.arm_target_advance_ik_status = 0u;
    g_upper_controller_debug.arm_target_advance_workspace_result = 0u;
    g_upper_controller_debug.arm_target_advance_failed_check_mask = 0u;
    g_upper_controller_debug.arm_target_advance_failed_sample = 0u;
    g_upper_controller_debug.arm_target_advance_failed_center_mm[0] = NAN;
    g_upper_controller_debug.arm_target_advance_failed_center_mm[1] = NAN;
    g_upper_controller_debug.arm_target_advance_failed_center_mm[2] = NAN;
    now_ms = HAL_GetTick();

    /*
     * AC closed-loop picking uses the arm pose at the moment the upper
     * computer reports the camera target. Refresh the pose here instead of
     * reusing the observation-complete snapshot, because perception or manual
     * debug may take longer than the pose-age guard.
     */
    capture_id = UpperControllerNextAcCaptureId();
    transform_status = UpperControllerCaptureCameraPose(capture_id, now_ms);
    g_arm_target_debug.transform_status = transform_status;
    snapshot = CameraTargetGetPoseSnapshot();
    if (snapshot != NULL) {
        g_upper_controller_debug.arm_target_pose_capture_id =
            snapshot->capture_id;
        g_upper_controller_debug.arm_target_pose_capture_tick_ms =
            snapshot->capture_tick_ms;
        if (snapshot->valid != 0u) {
            g_arm_target_debug.pose_age_ms =
                (uint32_t)(now_ms - snapshot->capture_tick_ms);
        }
    }
    if (transform_status != CAMERA_TARGET_STATUS_OK) {
        g_arm_target_debug.stage =
            UPPER_ARM_TARGET_DEBUG_TRANSFORM_FAILED;
        g_upper_controller_debug.arm_target_transform_fail_count++;
        g_upper_controller_debug.arm_target_deferred_count++;
        return;
    }

    transform_status = CameraTargetTransformLatest(
        capture_id, now_ms, CAMERA_TARGET_DEFAULT_MAX_POSE_AGE_MS,
        g_upper_controller_debug.arm_target_camera_mm,
        &transform_result);
    g_upper_controller_debug.arm_target_transform_status =
        transform_status;
    g_arm_target_debug.transform_status = transform_status;
    snapshot = CameraTargetGetPoseSnapshot();
    if (snapshot != NULL) {
        g_upper_controller_debug.arm_target_pose_capture_id =
            snapshot->capture_id;
        g_upper_controller_debug.arm_target_pose_capture_tick_ms =
            snapshot->capture_tick_ms;
        if (snapshot->valid != 0u) {
            g_arm_target_debug.pose_age_ms =
                (uint32_t)(now_ms - snapshot->capture_tick_ms);
        }
    }
    if (transform_status == CAMERA_TARGET_STATUS_OK) {
        memcpy(g_upper_controller_debug.arm_target_reference_mm,
               transform_result.reference_point_mm,
               sizeof(g_upper_controller_debug.arm_target_reference_mm));
        memcpy(g_upper_controller_debug.arm_target_base_mm,
               transform_result.base_point_mm,
               sizeof(g_upper_controller_debug.arm_target_base_mm));
        g_upper_controller_debug.arm_target_transform_success_count++;
        gate_flags = UpperControllerArmTargetGateFlags();
        g_arm_target_debug.gate_flags = gate_flags;
        if (gate_flags != 0u) {
            g_arm_target_debug.stage = UPPER_ARM_TARGET_DEBUG_DEFERRED;
            g_upper_controller_debug.arm_target_deferred_count++;
            return;
        }
        {
            App_Arm_Pick_Target_s target;
            App_Fruit_Side_e side =
                (App_Fruit_Side_e)g_upper_controller_debug.ac_active_side;
            App_Arm_Advance_Result_s advance_result;
            float advance_sign;
            float pick_x_bias_mm;
            uint8_t start_ok;

            if (side != APP_FRUIT_SIDE_LEFT &&
                side != APP_FRUIT_SIDE_RIGHT) {
                UpperControllerHandleArmTargetFailure(
                    UPPER_ARM_TARGET_DEBUG_PICK_REJECTED);
                return;
            }
            if (AppArmSidePickPlaceBuildPlaceProfile(
                    side, &upper_arm_target_place_profile) == 0u) {
                UpperControllerHandleArmTargetFailure(
                    UPPER_ARM_TARGET_DEBUG_PLACE_REJECTED);
                return;
            }
            advance_sign = side == APP_FRUIT_SIDE_RIGHT ? -1.0f : 1.0f;
            pick_x_bias_mm = side == APP_FRUIT_SIDE_RIGHT ?
                APP_ARM_AC_CLOSED_LOOP_RIGHT_PICK_X_BIAS_MM :
                APP_ARM_AC_CLOSED_LOOP_LEFT_PICK_X_BIAS_MM;
            memset(&target, 0, sizeof(target));
            target.approach_valid = 1u;
            target.approach_x_mm =
                transform_result.base_point_mm[0] + pick_x_bias_mm;
            if (UpperControllerClampAcNearY(
                    side, transform_result.base_point_mm[1],
                    &target.approach_y_mm) == 0u) {
                UpperControllerHandleArmTargetFailure(
                    UPPER_ARM_TARGET_DEBUG_NEAR_LIMIT_REJECTED);
                return;
            }
            target.approach_z_mm = APP_ARM_AC_CLOSED_LOOP_PICK_Z_MM;
            target.x_mm = target.approach_x_mm;
            target.z_mm = APP_ARM_AC_CLOSED_LOOP_PICK_Z_MM;
            target.tool_pitch_deg =
                APP_ARM_AC_CLOSED_LOOP_PICK_TOOL_PITCH_DEG;
            if (AppArmFlowSelectReachablePickAdvance(
                    &target, advance_sign,
                    APP_ARM_AC_CLOSED_LOOP_ADVANCE_MM,
                    APP_ARM_AC_CLOSED_LOOP_ADVANCE_SEARCH_STEP_MM,
                    &advance_result) == 0u) {
                UpperControllerRecordAcAdvanceResult(&advance_result);
                UpperControllerHandleArmTargetFailure(
                    UPPER_ARM_TARGET_DEBUG_ADVANCE_REJECTED);
                return;
            }
            UpperControllerRecordAcAdvanceResult(&advance_result);
            /*
             * AC闭环抓后放置沿用开环profile，但Y峰值按本次视觉目标动态
             * 收紧：只允许比最终抓取点再向当前侧前方多配置余量。
             */
            upper_arm_target_place_profile.transfer_path_y_max_mm =
                fabsf(target.y_mm) +
                APP_ARM_AC_CLOSED_LOOP_PLACE_FORWARD_MARGIN_MM;
            g_upper_controller_debug.arm_target_pick_center_mm[0] =
                target.x_mm;
            g_upper_controller_debug.arm_target_pick_center_mm[1] =
                target.y_mm;
            g_upper_controller_debug.arm_target_pick_center_mm[2] =
                target.z_mm;
            g_upper_controller_debug.arm_target_pick_tool_pitch_deg =
                target.tool_pitch_deg;
            start_ok = AppArmFlowStartPick(&target, now_ms);
            g_upper_controller_debug.arm_target_pick_start_result =
                start_ok;
            g_arm_target_debug.pick_start_result = start_ok;
            if (start_ok != 0u) {
                g_upper_controller_debug.arm_target_pick_running = 1u;
                upper_arm_target_flow_state = UPPER_ARM_TARGET_FLOW_PICK;
                g_upper_controller_debug.arm_target_pick_flow_status =
                    APP_ARM_FLOW_RUNNING;
                g_upper_controller_debug.arm_target_pick_start_count++;
                g_arm_target_debug.stage =
                    UPPER_ARM_TARGET_DEBUG_PICK_STARTED;
                g_upper_controller_debug.ac_observe_state =
                    UPPER_AC_OBSERVE_IDLE;
                g_upper_controller_debug.ac_operation_status =
                    (uint8_t)UPPER_AC_OBSERVE_IDLE;
                (void)UpperControllerSendCallback(
                    UPPER_CALLBACK_ARM_TARGET,
                    UPPER_CALLBACK_EXECUTING);
            } else {
                UpperControllerHandleArmTargetFailure(
                    UPPER_ARM_TARGET_DEBUG_PICK_REJECTED);
            }
        }
    } else {
        g_arm_target_debug.stage =
            UPPER_ARM_TARGET_DEBUG_TRANSFORM_FAILED;
        g_upper_controller_debug.arm_target_transform_fail_count++;
        g_upper_controller_debug.arm_target_deferred_count++;
    }
}

Camera_Target_Transform_Status_e UpperControllerCaptureCameraPose(
    uint32_t capture_id, uint32_t now_ms)
{
    const Arm_State_s *arm = ArmGetState();
    const Arm_Tool_State_s *tool = ArmToolGetState();
    const Camera_Target_Extrinsic_s *extrinsic =
        CameraTargetGetExtrinsic();
    Camera_Arm_Pose_Snapshot_s snapshot;
    Camera_Target_Transform_Status_e status;
    float wrist_origin_b_mm[3];
    float tool_center_b_mm[3];
    uint8_t axis;

    if (arm == NULL || tool == NULL || extrinsic == NULL ||
        arm->kinematics_valid == 0u) {
        status = CAMERA_TARGET_STATUS_INVALID_POSE;
        g_upper_controller_debug.arm_pose_capture_fail_count++;
        g_upper_controller_debug.arm_target_transform_status = status;
        return status;
    }
    for (axis = 0u; axis < 3u; ++axis) {
        if (arm->motor_online[axis] == 0u) {
            status = CAMERA_TARGET_STATUS_INVALID_POSE;
            g_upper_controller_debug.arm_pose_capture_fail_count++;
            g_upper_controller_debug.arm_target_transform_status = status;
            return status;
        }
    }
    wrist_origin_b_mm[0] = arm->wrist_center.x_mm;
    wrist_origin_b_mm[1] = arm->wrist_center.y_mm;
    wrist_origin_b_mm[2] = arm->wrist_center.z_mm;
    tool_center_b_mm[0] = arm->tool_tip.x_mm;
    tool_center_b_mm[1] = arm->tool_tip.y_mm;
    tool_center_b_mm[2] = arm->tool_tip.z_mm;
    status = CameraTargetBuildArmPoseSnapshot(
        extrinsic->reference_frame, capture_id, now_ms,
        arm->q_feedback_deg, wrist_origin_b_mm, tool_center_b_mm,
        arm->small_link_pitch_deg, tool->tool_pitch_feedback_deg,
        &snapshot);
    if (status == CAMERA_TARGET_STATUS_OK) {
        status = CameraTargetStorePoseSnapshot(&snapshot);
    }
    g_upper_controller_debug.arm_target_transform_status = status;
    if (status == CAMERA_TARGET_STATUS_OK) {
        g_upper_controller_debug.arm_target_pose_capture_id = capture_id;
        g_upper_controller_debug.arm_target_pose_capture_tick_ms = now_ms;
        g_upper_controller_debug.arm_pose_capture_success_count++;
    } else {
        g_upper_controller_debug.arm_pose_capture_fail_count++;
    }
    return status;
}

uint8_t UpperControllerGetCurrentArea(Upper_Controller_Area_e *area)
{
    if (area == NULL ||
        g_upper_controller_debug.current_area_valid == 0u) {
        return 0u;
    }
    *area = g_upper_controller_debug.current_area;
    return 1u;
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
