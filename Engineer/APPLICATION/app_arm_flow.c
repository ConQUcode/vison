/**
 * @file app_arm_flow.c
 * @brief 工具中心坐标抓取和固定角度放置两个机械臂子流程的状态机实现。
 *
 * 抓取目标由夹爪中心世界坐标和绝对俯仰定义；放置仍使用已实测的
 * 显式关节profile，两个流程共享命令终态和故障锁存逻辑。
 */

#include "app_arm_flow.h"

#include "app_config.h"

/* Watch符号沿用旧抓放测试名称；打点模式下也保留定义便于统一观察。 */
App_Arm_Pick_Place_Test_Debug_s g_app_arm_pick_place_test_debug;

#if (APP_ARM_ENABLED || APP_ARM_POSTURE_TEST_ENABLED || \
     APP_HOST_CONTROL_ENABLED) && \
    APP_ARM_TOOL_CENTER_TEST_ENABLE

#include <math.h>
#include <string.h>

#include "app_arm_command_id.h"
#include "arm.h"
#include "arm_config.h"
#include "arm_tool.h"
#include "arm_trajectory.h"

#define APP_ARM_FLOW_RAD_TO_DEG 57.29577951308232f
#define APP_ARM_FLOW_TRANSFER_SAMPLE_STEP_DEG 1.0f

/* 子流程运行时状态；同一时刻最多一个子流程活动。 */
static App_Arm_Flow_Active_e app_flow_active;
static App_Arm_Flow_Status_e app_flow_status;
static App_Arm_Pick_Step_e app_pick_step;
static App_Arm_Place_Step_e app_place_step;
static App_Arm_Pick_Target_s app_pick_target;
static App_Arm_Place_Profile_s app_place_profile;
static uint32_t app_flow_step_tick;    /* 当前步骤进入时刻ms。 */
static uint32_t app_pitch_stable_tick; /* 俯仰反馈连续稳定的起始时刻。 */

static void AppArmFlowSetPickStep(App_Arm_Pick_Step_e step, uint32_t now_ms)
{
    app_pick_step = step;
    g_app_arm_pick_place_test_debug.state_elapsed_ms = 0u;
    app_flow_step_tick = now_ms;
}

static void AppArmFlowSetPlaceStep(App_Arm_Place_Step_e step, uint32_t now_ms)
{
    app_place_step = step;
    g_app_arm_pick_place_test_debug.state_elapsed_ms = 0u;
    app_flow_step_tick = now_ms;
}

/** 锁存失败来源并置当前子流程为FAILED；不自动重试。 */
static void AppArmFlowFail(App_Arm_Pick_Place_Failure_Source_e source,
                           uint32_t code, uint32_t now_ms)
{
    g_app_arm_pick_place_test_debug.failure_source = source;
    g_app_arm_pick_place_test_debug.fault = code;
    app_flow_status = APP_ARM_FLOW_FAILED;
    if (app_flow_active == APP_ARM_FLOW_PICK) {
        AppArmFlowSetPickStep(APP_ARM_PICK_STEP_FAILED, now_ms);
    } else if (app_flow_active == APP_ARM_FLOW_PLACE) {
        AppArmFlowSetPlaceStep(APP_ARM_PLACE_STEP_FAILED, now_ms);
    }
}

/** 刷新夹爪中心反馈、误差、双舵机通信和轨迹安全Watch状态。 */
static void AppArmFlowUpdateWatch(const Arm_State_s *arm, uint32_t now_ms)
{
    const Arm_Tool_State_s *tool = ArmToolGetState();
    const Arm_Motion_Debug_s *motion = ArmGetMotionState();
    float dx;
    float dy;
    float dz;

    g_app_arm_pick_place_test_debug.active_flow = (uint8_t)app_flow_active;
    g_app_arm_pick_place_test_debug.flow_status = (uint8_t)app_flow_status;
    g_app_arm_pick_place_test_debug.pick_step = (uint8_t)app_pick_step;
    g_app_arm_pick_place_test_debug.place_step = (uint8_t)app_place_step;
    if (arm == NULL) {
        return;
    }
    g_app_arm_pick_place_test_debug.state_elapsed_ms =
        now_ms - app_flow_step_tick;
    g_app_arm_pick_place_test_debug.feedback_center_mm[0] =
        arm->tool_tip.x_mm;
    g_app_arm_pick_place_test_debug.feedback_center_mm[1] =
        arm->tool_tip.y_mm;
    g_app_arm_pick_place_test_debug.feedback_center_mm[2] =
        arm->tool_tip.z_mm;
    dx = g_app_arm_pick_place_test_debug.target_center_mm[0] -
        arm->tool_tip.x_mm;
    dy = g_app_arm_pick_place_test_debug.target_center_mm[1] -
        arm->tool_tip.y_mm;
    dz = g_app_arm_pick_place_test_debug.target_center_mm[2] -
        arm->tool_tip.z_mm;
    g_app_arm_pick_place_test_debug.center_error_mm[0] = dx;
    g_app_arm_pick_place_test_debug.center_error_mm[1] = dy;
    g_app_arm_pick_place_test_debug.center_error_mm[2] = dz;
    g_app_arm_pick_place_test_debug.center_error_norm_mm =
        sqrtf(dx * dx + dy * dy + dz * dz);
    if (tool != NULL) {
        g_app_arm_pick_place_test_debug.servo1_communication_ok =
            tool->servo_online[0] != 0u &&
            tool->servo_feedback_valid[0] != 0u;
        g_app_arm_pick_place_test_debug.servo2_communication_ok =
            tool->servo_online[1] != 0u &&
            tool->servo_feedback_valid[1] != 0u;
        g_app_arm_pick_place_test_debug.gripper_state =
            (uint8_t)tool->gripper_state;
        g_app_arm_pick_place_test_debug.pitch_feedback_deg =
            tool->tool_pitch_feedback_deg;
        g_app_arm_pick_place_test_debug.pitch_error_deg =
            g_app_arm_pick_place_test_debug.pitch_target_deg -
            tool->tool_pitch_feedback_deg;
        g_app_arm_pick_place_test_debug.tool_error_code =
            tool->error_code;
    }
    if (motion != NULL) {
        g_app_arm_pick_place_test_debug.motion_state =
            (uint32_t)motion->motion_state;
        g_app_arm_pick_place_test_debug.motion_fault =
            (uint32_t)motion->fault_code;
        g_app_arm_pick_place_test_debug.safety_route_enabled =
            motion->safety_route_enabled;
        g_app_arm_pick_place_test_debug.safety_route_segment =
            motion->safety_route_active_segment;
        g_app_arm_pick_place_test_debug.workspace_safety_result =
            (uint32_t)motion->workspace_safety_result;
        g_app_arm_pick_place_test_debug.ik_status =
            (uint32_t)motion->ik_status;
        g_app_arm_pick_place_test_debug.path_sample_count =
            motion->path_sample_count;
        g_app_arm_pick_place_test_debug.preflight_duration_ms =
            motion->preflight_duration_ms;
        g_app_arm_pick_place_test_debug.preflight_motor_service_count =
            motion->preflight_motor_service_count;
        g_app_arm_pick_place_test_debug.preflight_tool_service_count =
            motion->preflight_tool_service_count;
        g_app_arm_pick_place_test_debug.preflight_failed_segment =
            motion->preflight_failed_segment;
        g_app_arm_pick_place_test_debug.preflight_failed_sample =
            motion->preflight_failed_sample;
        g_app_arm_pick_place_test_debug.preflight_failed_check_mask =
            motion->preflight_failed_check_mask;
        g_app_arm_pick_place_test_debug.preflight_failed_center_mm[0] =
            motion->preflight_failed_center_mm.x_mm;
        g_app_arm_pick_place_test_debug.preflight_failed_center_mm[1] =
            motion->preflight_failed_center_mm.y_mm;
        g_app_arm_pick_place_test_debug.preflight_failed_center_mm[2] =
            motion->preflight_failed_center_mm.z_mm;
        memcpy(g_app_arm_pick_place_test_debug.preflight_failed_q_deg,
               motion->preflight_failed_q_deg,
               sizeof(motion->preflight_failed_q_deg));
        memcpy(g_app_arm_pick_place_test_debug.target_q_deg,
               motion->target_q_deg, sizeof(motion->target_q_deg));
    }
    for (uint8_t axis = 0u; axis < 3u; ++axis) {
        g_app_arm_pick_place_test_debug.dm_online[axis] =
            g_arm_dm_debug.axis[axis].online;
        g_app_arm_pick_place_test_debug.dm_rx_count[axis] =
            g_arm_dm_debug.axis[axis].rx_count;
        g_app_arm_pick_place_test_debug.dm_feedback_age_ms[axis] =
            g_arm_dm_debug.axis[axis].feedback_age_ms;
    }
}

static uint8_t AppArmFlowCommandFinished(
    const Arm_Host_Status_s *host, uint32_t command_id, uint32_t now_ms)
{
    g_app_arm_pick_place_test_debug.command_state =
        (uint32_t)host->last_command_state;
    g_app_arm_pick_place_test_debug.command_result =
        (uint32_t)host->last_command_result;
    if (host->last_command_id != command_id) {
        return 0u;
    }
    if (host->last_command_state == ARM_COMMAND_STATE_COMPLETED &&
        host->last_command_result == ARM_COMMAND_OK) {
        return 1u;
    }
    /* QUEUED/RUNNING不是失败；只有终态才结束流程并锁存失败来源。 */
    if (host->last_command_state != ARM_COMMAND_STATE_REJECTED &&
        host->last_command_state != ARM_COMMAND_STATE_CANCELLED &&
        host->last_command_state != ARM_COMMAND_STATE_FAULTED) {
        return 0u;
    }
    AppArmFlowFail(APP_ARM_PICK_PLACE_FAILURE_COMMAND_EXECUTION,
                   (uint32_t)host->last_command_result, now_ms);
    return 0u;
}

static uint8_t AppArmFlowSubmitTool(Arm_Tool_Action_e action,
                                    float pitch_deg,
                                    uint32_t now_ms)
{
    Arm_Command_s command;
    Arm_Command_Result_e result;

    memset(&command, 0, sizeof(command));
    command.command_id = AppArmCommandIdNext();
    command.type = ARM_COMMAND_TYPE_TOOL;
    command.payload.tool.action = action;
    command.payload.tool.pitch_deg = pitch_deg;
    if (action == ARM_TOOL_ACTION_SET_PITCH) {
        g_app_arm_pick_place_test_debug.pitch_target_deg = pitch_deg;
    }
    result = ArmSubmitCommand(&command);
    g_app_arm_pick_place_test_debug.active_command_id = command.command_id;
    g_app_arm_pick_place_test_debug.submit_result = (uint32_t)result;
    if (result != ARM_COMMAND_OK) {
        g_app_arm_pick_place_test_debug.command_result = (uint32_t)result;
        AppArmFlowFail(APP_ARM_PICK_PLACE_FAILURE_COMMAND_SUBMIT,
                       (uint32_t)result, now_ms);
        return 0u;
    }
    return 1u;
}

/**
 * 提交抓放过渡关节命令。未被指定的关节沿用当前反馈值；子流程可
 * 在一条命令内同步设置多个关节，也可只调整底座等单个关节。
 */
static uint8_t AppArmFlowSubmitJoint(uint8_t set_base,
                                     float base_q1_deg,
                                     uint8_t set_shoulder,
                                     float shoulder_q2_deg,
                                     uint8_t set_elbow,
                                     float elbow_q3_deg,
                                     uint32_t now_ms)
{
    const Arm_State_s *arm = ArmGetState();
    Arm_Joint_Command_s command;
    Arm_Command_Result_e result;

    if (arm == NULL) {
        AppArmFlowFail(APP_ARM_PICK_PLACE_FAILURE_COMMAND_SUBMIT,
                       (uint32_t)ARM_COMMAND_NOT_READY, now_ms);
        return 0u;
    }
    memset(&command, 0, sizeof(command));
    command.command_id = AppArmCommandIdNext();
    command.move_type = ARM_MOVE_LINEAR;
    memcpy(command.q_deg, arm->q_feedback_deg, sizeof(command.q_deg));
    if (set_base != 0u) {
        command.q_deg[ARM_JOINT_BASE_YAW] = base_q1_deg;
    }
    if (set_shoulder != 0u) {
        command.q_deg[ARM_JOINT_SHOULDER] = shoulder_q2_deg;
    }
    if (set_elbow != 0u) {
        command.q_deg[ARM_JOINT_ELBOW] = elbow_q3_deg;
    }
    memcpy(g_app_arm_pick_place_test_debug.target_q_deg, command.q_deg,
           sizeof(command.q_deg));
    result = ArmSubmitJointCommand(&command);
    g_app_arm_pick_place_test_debug.active_command_id = command.command_id;
    g_app_arm_pick_place_test_debug.submit_result = (uint32_t)result;
    if (result != ARM_COMMAND_OK) {
        g_app_arm_pick_place_test_debug.command_result = (uint32_t)result;
        AppArmFlowFail(APP_ARM_PICK_PLACE_FAILURE_COMMAND_SUBMIT,
                       (uint32_t)result, now_ms);
        return 0u;
    }
    return 1u;
}

/**
 * 提交三轴关节与ID1相对俯仰联合命令。DM三轴按同一轨迹插值，ID1从
 * 轨迹启动时即更新到指定的相对小臂角度，避免放置姿态逐轴串行等待。
 */
static uint8_t AppArmFlowSubmitJointWithRelativePitch(
    uint8_t set_base,
    float base_q1_deg,
    uint8_t set_shoulder,
    float shoulder_q2_deg,
    uint8_t set_elbow,
    float elbow_q3_deg,
    float relative_pitch_deg,
    uint32_t now_ms)
{
    const Arm_State_s *arm = ArmGetState();
    Arm_Joint_Command_s command;
    Arm_Command_Result_e result;

    if (arm == NULL) {
        AppArmFlowFail(APP_ARM_PICK_PLACE_FAILURE_COMMAND_SUBMIT,
                       (uint32_t)ARM_COMMAND_NOT_READY, now_ms);
        return 0u;
    }
    memset(&command, 0, sizeof(command));
    command.command_id = AppArmCommandIdNext();
    command.move_type = ARM_MOVE_LINEAR;
    memcpy(command.q_deg, arm->q_feedback_deg, sizeof(command.q_deg));
    if (set_base != 0u) {
        command.q_deg[ARM_JOINT_BASE_YAW] = base_q1_deg;
    }
    if (set_shoulder != 0u) {
        command.q_deg[ARM_JOINT_SHOULDER] = shoulder_q2_deg;
    }
    if (set_elbow != 0u) {
        command.q_deg[ARM_JOINT_ELBOW] = elbow_q3_deg;
    }
    command.tool_relative_pitch_valid = 1u;
    command.tool_relative_pitch_deg = relative_pitch_deg;
    g_app_arm_pick_place_test_debug.pitch_target_deg =
        ArmToolSmallLinkPitchFromJoint(command.q_deg) + relative_pitch_deg;
    memcpy(g_app_arm_pick_place_test_debug.target_q_deg, command.q_deg,
           sizeof(command.q_deg));
    result = ArmSubmitJointCommand(&command);
    g_app_arm_pick_place_test_debug.active_command_id = command.command_id;
    g_app_arm_pick_place_test_debug.submit_result = (uint32_t)result;
    if (result != ARM_COMMAND_OK) {
        g_app_arm_pick_place_test_debug.command_result = (uint32_t)result;
        AppArmFlowFail(APP_ARM_PICK_PLACE_FAILURE_COMMAND_SUBMIT,
                       (uint32_t)result, now_ms);
        return 0u;
    }
    return 1u;
}

/** 提交以夹爪中心世界坐标和世界绝对俯仰定义的抓取运动。 */
static uint8_t AppArmFlowSubmitPickCenter(float x_mm, float y_mm,
                                          float z_mm,
                                          float tool_pitch_deg,
                                          float max_speed_mm_s,
                                          uint32_t now_ms)
{
    Arm_Tool_Center_Command_s command;
    Arm_Command_Result_e result;

    memset(&command, 0, sizeof(command));
    command.command_id = AppArmCommandIdNext();
    command.move_type = ARM_MOVE_LINEAR;
    command.target_center_mm.x_mm = x_mm;
    command.target_center_mm.y_mm = y_mm;
    command.target_center_mm.z_mm = z_mm;
    command.max_speed_mm_s = max_speed_mm_s;
    command.tool_pitch_valid = 1u;
    command.tool_pitch_deg = tool_pitch_deg;
    /* 仅AC抓取接近/推进允许低位侧向q1进入+/-115deg。 */
    command.safety_profile = ARM_CARTESIAN_SAFETY_AC_SIDE_PICK;
    g_app_arm_pick_place_test_debug.pitch_target_deg =
        tool_pitch_deg;
    result = ArmSubmitToolCenterCommand(&command);
    g_app_arm_pick_place_test_debug.active_command_id = command.command_id;
    g_app_arm_pick_place_test_debug.submit_result = (uint32_t)result;
    if (result != ARM_COMMAND_OK) {
        g_app_arm_pick_place_test_debug.command_result = (uint32_t)result;
        AppArmFlowFail(APP_ARM_PICK_PLACE_FAILURE_COMMAND_SUBMIT,
                       (uint32_t)result, now_ms);
        return 0u;
    }
    return 1u;
}

static uint8_t AppArmFlowSubmitPickApproach(uint32_t now_ms)
{
    return AppArmFlowSubmitPickCenter(
        app_pick_target.approach_x_mm,
        app_pick_target.approach_y_mm,
        app_pick_target.approach_z_mm,
        app_pick_target.tool_pitch_deg,
        APP_ARM_POSTURE_TEST_APPROACH_SPEED_MM_S, now_ms);
}

static uint8_t AppArmFlowSubmitPickTarget(uint32_t now_ms)
{
    float speed_mm_s = app_pick_target.approach_valid != 0u ?
        APP_ARM_POSTURE_TEST_GRIP_ADVANCE_SPEED_MM_S :
        APP_ARM_TOOL_CENTER_TEST_SPEED_MM_S;

    return AppArmFlowSubmitPickCenter(
        app_pick_target.x_mm, app_pick_target.y_mm,
        app_pick_target.z_mm, app_pick_target.tool_pitch_deg,
        speed_mm_s, now_ms);
}

/** 提交带指定方向引导点的底座连续旋转；引导点不触发到位等待。 */
static uint8_t AppArmFlowSubmitDirectedBaseRotation(
    float waypoint_q1_deg, float target_q1_deg, uint32_t now_ms)
{
    const Arm_State_s *arm = ArmGetState();
    Arm_Joint_Command_s command;
    Arm_Command_Result_e result;

    if (arm == NULL) {
        AppArmFlowFail(APP_ARM_PICK_PLACE_FAILURE_COMMAND_SUBMIT,
                       (uint32_t)ARM_COMMAND_NOT_READY, now_ms);
        return 0u;
    }
    memset(&command, 0, sizeof(command));
    command.command_id = AppArmCommandIdNext();
    command.move_type = ARM_MOVE_LINEAR;
    memcpy(command.q_deg, arm->q_feedback_deg, sizeof(command.q_deg));
    command.q_deg[ARM_JOINT_BASE_YAW] = target_q1_deg;
    command.waypoint_valid = 1u;
    memcpy(command.waypoint_q_deg, arm->q_feedback_deg,
           sizeof(command.waypoint_q_deg));
    command.waypoint_q_deg[ARM_JOINT_BASE_YAW] = waypoint_q1_deg;
    memcpy(g_app_arm_pick_place_test_debug.target_q_deg, command.q_deg,
           sizeof(command.q_deg));
    result = ArmSubmitJointCommand(&command);
    g_app_arm_pick_place_test_debug.active_command_id = command.command_id;
    g_app_arm_pick_place_test_debug.submit_result = (uint32_t)result;
    if (result != ARM_COMMAND_OK) {
        g_app_arm_pick_place_test_debug.command_result = (uint32_t)result;
        AppArmFlowFail(APP_ARM_PICK_PLACE_FAILURE_COMMAND_SUBMIT,
                       (uint32_t)result, now_ms);
        return 0u;
    }
    return 1u;
}

/**
 * AC抓后去后方放置前，先保持底座在当前侧，只把q2/q3和ID1收拢到
 * 后方旋转安全姿态。这样后续q1进入后方区域时，不会从低抓取姿态扫过。
 */
static uint8_t AppArmFlowSubmitRearRotateStaging(uint32_t now_ms)
{
    return AppArmFlowSubmitJointWithRelativePitch(
        0u, 0.0f,
        1u, app_place_profile.safe_q_deg[ARM_JOINT_SHOULDER],
        1u, app_place_profile.safe_q_deg[ARM_JOINT_ELBOW],
        app_place_profile.release_tool_relative_pitch_deg,
        now_ms);
}

/**
 * AC抓后专用：底座转到后方放置点，同时q2/q3进入释放姿态。
 * 受约束profile下，把原独立REAR_STAGING安全位合并为本命令的中间waypoint，
 * 减少“到安全位停住 -> 再发底座旋转”的硬停顿。
 */
static uint8_t AppArmFlowSubmitDirectedReleaseRotation(uint32_t now_ms)
{
    Arm_Joint_Command_s command;
    Arm_Command_Result_e result;

    memset(&command, 0, sizeof(command));
    command.command_id = AppArmCommandIdNext();
    command.move_type = ARM_MOVE_LINEAR;
    memcpy(command.q_deg, app_place_profile.release_q_deg,
           sizeof(command.q_deg));
    command.tool_relative_pitch_valid = 1u;
    command.tool_relative_pitch_deg =
        app_place_profile.release_tool_relative_pitch_deg;
    command.waypoint_valid = 1u;
    if (app_place_profile.transfer_path_constraints_enabled != 0u) {
        memcpy(command.waypoint_q_deg, app_place_profile.safe_q_deg,
               sizeof(command.waypoint_q_deg));
    } else {
        memcpy(command.waypoint_q_deg, app_place_profile.release_q_deg,
               sizeof(command.waypoint_q_deg));
        command.waypoint_q_deg[ARM_JOINT_BASE_YAW] =
            app_place_profile.rotate_to_place_waypoint_q1_deg;
    }
    memcpy(g_app_arm_pick_place_test_debug.target_q_deg, command.q_deg,
           sizeof(command.q_deg));
    g_app_arm_pick_place_test_debug.pitch_target_deg =
        ArmToolSmallLinkPitchFromJoint(command.q_deg) +
        command.tool_relative_pitch_deg;
    result = ArmSubmitJointCommand(&command);
    g_app_arm_pick_place_test_debug.active_command_id = command.command_id;
    g_app_arm_pick_place_test_debug.submit_result = (uint32_t)result;
    if (result != ARM_COMMAND_OK) {
        g_app_arm_pick_place_test_debug.command_result = (uint32_t)result;
        AppArmFlowFail(APP_ARM_PICK_PLACE_FAILURE_COMMAND_SUBMIT,
                       (uint32_t)result, now_ms);
        return 0u;
    }
    return 1u;
}

/**
 * 释放后回前方：先在后方保持q1不变，经release_clearance抬臂作为
 * 安全waypoint，再在同一条关节命令内转回前方。中间waypoint使用
 * 轨迹层宽松到位判定，避免release_clearance作为独立命令硬停顿。
 */
static uint8_t AppArmFlowSubmitFrontRotationViaReleaseClearance(
    uint32_t now_ms)
{
    Arm_Joint_Command_s command;
    Arm_Command_Result_e result;

    memset(&command, 0, sizeof(command));
    command.command_id = AppArmCommandIdNext();
    command.move_type = ARM_MOVE_LINEAR;
    memcpy(command.q_deg, app_place_profile.release_clearance_q_deg,
           sizeof(command.q_deg));
    command.q_deg[ARM_JOINT_BASE_YAW] =
        app_place_profile.rotate_to_front_target_q1_deg;
    command.waypoint_valid = 1u;
    memcpy(command.waypoint_q_deg, app_place_profile.release_clearance_q_deg,
           sizeof(command.waypoint_q_deg));
    memcpy(g_app_arm_pick_place_test_debug.target_q_deg, command.q_deg,
           sizeof(command.q_deg));
    result = ArmSubmitJointCommand(&command);
    g_app_arm_pick_place_test_debug.active_command_id = command.command_id;
    g_app_arm_pick_place_test_debug.submit_result = (uint32_t)result;
    if (result != ARM_COMMAND_OK) {
        g_app_arm_pick_place_test_debug.command_result = (uint32_t)result;
        AppArmFlowFail(APP_ARM_PICK_PLACE_FAILURE_COMMAND_SUBMIT,
                       (uint32_t)result, now_ms);
        return 0u;
    }
    return 1u;
}

static uint8_t AppArmFlowValueInRange(float value, float min_value,
                                      float max_value)
{
    return isfinite(value) && value >= min_value && value <= max_value;
}

static uint8_t AppArmFlowPoseValid(const float q_deg[3])
{
    return q_deg != NULL &&
        AppArmFlowValueInRange(q_deg[ARM_JOINT_BASE_YAW],
                               ARM_Q1_SOFT_MIN_DEG,
                               ARM_Q1_SOFT_MAX_DEG) &&
        AppArmFlowValueInRange(q_deg[ARM_JOINT_SHOULDER],
                               ARM_Q2_SOFT_MIN_DEG,
                               ARM_Q2_SOFT_MAX_DEG) &&
        AppArmFlowValueInRange(q_deg[ARM_JOINT_ELBOW],
                               ARM_Q3_SOFT_MIN_DEG,
                               ARM_Q3_SOFT_MAX_DEG);
}

static uint8_t AppArmFlowPlaceProfileValid(
    const App_Arm_Place_Profile_s *profile)
{
    float to_waypoint_delta;
    float to_target_delta;
    float front_waypoint_delta;
    float front_target_delta;

    if (profile == NULL || profile->profile_id == 0u ||
        profile->release_pitch_wait_timeout_ms == 0u ||
        profile->transfer_waypoint_valid > 1u ||
        profile->transfer_path_constraints_enabled > 1u ||
        !AppArmFlowPoseValid(profile->safe_q_deg) ||
        !AppArmFlowPoseValid(profile->release_q_deg) ||
        !AppArmFlowPoseValid(profile->release_clearance_q_deg)) {
        return 0u;
    }
    if ((profile->transfer_waypoint_valid != 0u &&
         !AppArmFlowPoseValid(profile->transfer_waypoint_q_deg)) ||
        (profile->transfer_path_constraints_enabled != 0u &&
         (profile->transfer_waypoint_valid == 0u ||
          !isfinite(profile->transfer_path_y_max_mm) ||
          profile->transfer_path_y_max_mm <= 0.0f ||
          !isfinite(profile->transfer_waypoint_z_raise_mm) ||
          profile->transfer_waypoint_z_raise_mm < 0.0f ||
          !isfinite(profile->transfer_waypoint_z_tolerance_mm) ||
          profile->transfer_waypoint_z_tolerance_mm < 0.0f))) {
        return 0u;
    }
    if (!AppArmFlowValueInRange(
               profile->rotate_to_place_waypoint_q1_deg,
               ARM_Q1_SOFT_MIN_DEG, ARM_Q1_SOFT_MAX_DEG) ||
        !AppArmFlowValueInRange(profile->rotate_to_place_target_q1_deg,
                                ARM_Q1_SOFT_MIN_DEG,
                                ARM_Q1_SOFT_MAX_DEG) ||
        !AppArmFlowValueInRange(profile->release_tool_relative_pitch_deg,
                                ARM_TOOL_PITCH_RELATIVE_MIN_DEG,
                                ARM_TOOL_PITCH_RELATIVE_MAX_DEG) ||
        !AppArmFlowValueInRange(
               profile->rotate_to_front_waypoint_q1_deg,
               ARM_Q1_SOFT_MIN_DEG, ARM_Q1_SOFT_MAX_DEG) ||
        !AppArmFlowValueInRange(profile->rotate_to_front_target_q1_deg,
                                ARM_Q1_SOFT_MIN_DEG,
                                ARM_Q1_SOFT_MAX_DEG)) {
        return 0u;
    }
    if ((profile->transfer_waypoint_valid != 0u &&
         fabsf(profile->transfer_waypoint_q_deg[ARM_JOINT_BASE_YAW] -
               profile->safe_q_deg[ARM_JOINT_BASE_YAW]) > 0.01f) ||
        fabsf(profile->release_q_deg[ARM_JOINT_BASE_YAW] -
              profile->rotate_to_place_target_q1_deg) > 0.01f ||
        fabsf(profile->release_clearance_q_deg[ARM_JOINT_BASE_YAW] -
              profile->rotate_to_place_target_q1_deg) > 0.01f) {
        return 0u;
    }
    to_waypoint_delta = profile->rotate_to_place_waypoint_q1_deg -
        profile->safe_q_deg[ARM_JOINT_BASE_YAW];
    to_target_delta = profile->rotate_to_place_target_q1_deg -
        profile->rotate_to_place_waypoint_q1_deg;
    front_waypoint_delta = profile->rotate_to_front_waypoint_q1_deg -
        profile->release_clearance_q_deg[ARM_JOINT_BASE_YAW];
    front_target_delta = profile->rotate_to_front_target_q1_deg -
        profile->rotate_to_front_waypoint_q1_deg;
    return fabsf(to_waypoint_delta) > 0.01f &&
        fabsf(to_target_delta) > 0.01f &&
        to_waypoint_delta * to_target_delta > 0.0f &&
        fabsf(front_waypoint_delta) > 0.01f &&
        fabsf(front_target_delta) > 0.01f &&
        front_waypoint_delta * front_target_delta > 0.0f;
}

uint8_t AppArmFlowBuildPickStaging(float target_x_mm, float target_y_mm,
                                   App_Arm_Pick_Staging_s *staging)
{
    float base_q1_deg;

    if (staging == NULL || !isfinite(target_x_mm) ||
        !isfinite(target_y_mm) ||
        (fabsf(target_x_mm) < 0.001f && fabsf(target_y_mm) < 0.001f)) {
        return 0u;
    }
    base_q1_deg = atan2f(target_y_mm, target_x_mm) *
        APP_ARM_FLOW_RAD_TO_DEG;
    if (base_q1_deg > APP_ARM_PICK_BASE_AIM_MAX_ABS_Q1_DEG) {
        base_q1_deg = APP_ARM_PICK_BASE_AIM_MAX_ABS_Q1_DEG;
    } else if (base_q1_deg < -APP_ARM_PICK_BASE_AIM_MAX_ABS_Q1_DEG) {
        base_q1_deg = -APP_ARM_PICK_BASE_AIM_MAX_ABS_Q1_DEG;
    }
    staging->q_deg[ARM_JOINT_BASE_YAW] = base_q1_deg;
    staging->q_deg[ARM_JOINT_SHOULDER] = APP_ARM_PICK_STAGING_Q2_DEG;
    staging->q_deg[ARM_JOINT_ELBOW] = APP_ARM_PICK_STAGING_Q3_DEG;
    staging->tool_relative_pitch_deg =
        APP_ARM_PICK_STAGING_TOOL_RELATIVE_PITCH_DEG;
    return 1u;
}

static App_Arm_Advance_Reject_Reason_e AppArmFlowAdvanceRejectReason(
    const Arm_Path_Plan_Result_s *plan)
{
    if (plan == NULL || plan->status == ARM_PATH_PLAN_INVALID) {
        return APP_ARM_ADVANCE_REJECT_INVALID;
    }
    if (plan->status == ARM_PATH_PLAN_SAMPLE_CAPACITY) {
        return APP_ARM_ADVANCE_REJECT_SAMPLE_CAPACITY;
    }
    if ((plan->failed_check_mask &
         ARM_PATH_PREFLIGHT_FAIL_TOOL_PITCH) != 0u) {
        return APP_ARM_ADVANCE_REJECT_TOOL_PITCH;
    }
    if ((plan->failed_check_mask &
         ARM_PATH_PREFLIGHT_FAIL_WORKSPACE) != 0u) {
        return APP_ARM_ADVANCE_REJECT_WORKSPACE;
    }
    if ((plan->failed_check_mask &
         ARM_PATH_PREFLIGHT_FAIL_JOINT_STEP) != 0u) {
        return APP_ARM_ADVANCE_REJECT_CONTINUITY;
    }
    if ((plan->failed_check_mask &
         (ARM_PATH_PREFLIGHT_FAIL_JOINT_LIMIT |
          ARM_PATH_PREFLIGHT_FAIL_AUTO_REGION)) != 0u) {
        return APP_ARM_ADVANCE_REJECT_JOINT_LIMIT;
    }
    return APP_ARM_ADVANCE_REJECT_IK;
}

static void AppArmFlowStoreAdvancePlanResult(
    App_Arm_Advance_Result_s *result,
    const Arm_Path_Plan_Result_s *plan)
{
    result->planner_status = (uint32_t)plan->status;
    result->ik_status = (uint32_t)plan->ik_status;
    result->workspace_safety_result =
        (uint32_t)plan->workspace_safety_result;
    result->failed_check_mask = plan->failed_check_mask;
    result->failed_sample = plan->failed_sample;
    result->failed_center_mm[0] = plan->failed_center_mm.x_mm;
    result->failed_center_mm[1] = plan->failed_center_mm.y_mm;
    result->failed_center_mm[2] = plan->failed_center_mm.z_mm;
}

uint8_t AppArmFlowSelectReachablePickAdvance(
    App_Arm_Pick_Target_s *target, float advance_sign,
    float requested_advance_mm, float sample_step_mm,
    App_Arm_Advance_Result_s *result)
{
    App_Arm_Pick_Staging_s staging;
    Arm_Path_Advance_Request_s advance_request;
    Arm_Path_Advance_Result_s advance_result;
    float step_count;

    if (result == NULL) {
        return 0u;
    }
    memset(result, 0, sizeof(*result));
    result->requested_mm = requested_advance_mm;
    result->reject_reason = APP_ARM_ADVANCE_REJECT_INVALID;
    if (target == NULL || target->approach_valid == 0u ||
        !isfinite(advance_sign) || fabsf(fabsf(advance_sign) - 1.0f) >
            0.001f ||
        !isfinite(requested_advance_mm) || requested_advance_mm <= 0.0f ||
        !isfinite(sample_step_mm) || sample_step_mm <= 0.0f ||
        !isfinite(target->approach_x_mm) ||
        !isfinite(target->approach_y_mm) ||
        !isfinite(target->approach_z_mm) ||
        !isfinite(target->tool_pitch_deg)) {
        return 0u;
    }
    step_count = requested_advance_mm / sample_step_mm;
    if (!isfinite(step_count) ||
        fabsf(step_count - floorf(step_count + 0.5f)) > 0.0001f) {
        return 0u;
    }
    if (ArmTrajectoryIsBusy()) {
        result->reject_reason = APP_ARM_ADVANCE_REJECT_BUSY;
        return 0u;
    }
    if (!AppArmFlowBuildPickStaging(
            target->approach_x_mm, target->approach_y_mm, &staging)) {
        return 0u;
    }

    memset(&advance_request, 0, sizeof(advance_request));
    memcpy(advance_request.staging_q_deg, staging.q_deg,
           sizeof(advance_request.staging_q_deg));
    advance_request.approach_center_mm.x_mm = target->approach_x_mm;
    advance_request.approach_center_mm.y_mm = target->approach_y_mm;
    advance_request.approach_center_mm.z_mm = target->approach_z_mm;
    advance_request.tool_pitch_deg = target->tool_pitch_deg;
    advance_request.advance_sign = advance_sign;
    advance_request.requested_advance_mm = requested_advance_mm;
    advance_request.sample_step_mm = sample_step_mm;
    advance_request.safety_profile = ARM_CARTESIAN_SAFETY_AC_SIDE_PICK;
    if (!ArmTrajectorySelectReachableToolCenterAdvance(
            &advance_request, &advance_result)) {
        result->approach_failed = advance_result.approach_failed;
        AppArmFlowStoreAdvancePlanResult(
            result, &advance_result.plan_result);
        result->reject_reason = advance_result.approach_failed != 0u ?
            APP_ARM_ADVANCE_REJECT_APPROACH :
            AppArmFlowAdvanceRejectReason(&advance_result.plan_result);
        return 0u;
    }

    target->x_mm = target->approach_x_mm;
    target->y_mm = target->approach_y_mm +
        advance_sign * advance_result.selected_advance_mm;
    target->z_mm = target->approach_z_mm;
    result->selected_mm = advance_result.selected_advance_mm;
    result->reject_reason = APP_ARM_ADVANCE_REJECT_NONE;
    AppArmFlowStoreAdvancePlanResult(
        result, advance_result.advance_limited != 0u ?
            &advance_result.limiting_plan_result :
            &advance_result.plan_result);
    return 1u;
}

static uint8_t AppArmFlowBuildActivePickStaging(
    App_Arm_Pick_Staging_s *staging)
{
    float aim_x_mm = app_pick_target.x_mm;
    float aim_y_mm = app_pick_target.y_mm;

    if (app_pick_target.approach_valid != 0u) {
        aim_x_mm = app_pick_target.approach_x_mm;
        aim_y_mm = app_pick_target.approach_y_mm;
    }
    return AppArmFlowBuildPickStaging(aim_x_mm, aim_y_mm, staging);
}

/**
 * 抓取准备：一条关节轨迹先经过“底座保持当前角、q2/q3/ID1收拢”的
 * 安全waypoint，再连续转到底座对准目标的准备姿态。这样保留后方框避障
 * 顺序，但不再把安全姿态作为独立命令等待，减少抓取前明显停顿。
 */
static uint8_t AppArmFlowSubmitBaseAim(uint32_t now_ms)
{
    const Arm_State_s *arm = ArmGetState();
    App_Arm_Pick_Staging_s staging;
    Arm_Joint_Command_s command;
    Arm_Command_Result_e result;

    if (arm == NULL) {
        AppArmFlowFail(APP_ARM_PICK_PLACE_FAILURE_COMMAND_SUBMIT,
                       (uint32_t)ARM_COMMAND_NOT_READY, now_ms);
        return 0u;
    }
    if (!AppArmFlowBuildActivePickStaging(&staging)) {
        AppArmFlowFail(APP_ARM_PICK_PLACE_FAILURE_COMMAND_SUBMIT,
                       (uint32_t)ARM_COMMAND_INVALID, now_ms);
        return 0u;
    }
    memset(&command, 0, sizeof(command));
    command.command_id = AppArmCommandIdNext();
    command.move_type = ARM_MOVE_LINEAR;
    memcpy(command.q_deg, staging.q_deg, sizeof(command.q_deg));
    command.waypoint_valid = 1u;
    memcpy(command.waypoint_q_deg, arm->q_feedback_deg,
           sizeof(command.waypoint_q_deg));
    command.waypoint_q_deg[ARM_JOINT_SHOULDER] =
        staging.q_deg[ARM_JOINT_SHOULDER];
    command.waypoint_q_deg[ARM_JOINT_ELBOW] =
        staging.q_deg[ARM_JOINT_ELBOW];
    command.tool_relative_pitch_valid = 1u;
    command.tool_relative_pitch_deg = staging.tool_relative_pitch_deg;
    g_app_arm_pick_place_test_debug.pitch_target_deg =
        ArmToolSmallLinkPitchFromJoint(command.q_deg) +
        command.tool_relative_pitch_deg;
    memcpy(g_app_arm_pick_place_test_debug.target_q_deg, command.q_deg,
           sizeof(command.q_deg));
    result = ArmSubmitJointCommand(&command);
    g_app_arm_pick_place_test_debug.active_command_id = command.command_id;
    g_app_arm_pick_place_test_debug.submit_result = (uint32_t)result;
    if (result != ARM_COMMAND_OK) {
        g_app_arm_pick_place_test_debug.command_result = (uint32_t)result;
        AppArmFlowFail(APP_ARM_PICK_PLACE_FAILURE_COMMAND_SUBMIT,
                       (uint32_t)result, now_ms);
        return 0u;
    }
    return 1u;
}

static uint8_t AppArmFlowTransferSegmentWithinLimits(
    const float start_q_deg[3], const float target_q_deg[3],
    float relative_pitch_deg, float y_limit_mm,
    uint8_t require_z_non_decreasing, float *peak_abs_y_mm,
    float *start_z_mm, float *end_z_mm)
{
    float max_delta_deg = 0.0f;
    float previous_z_mm = 0.0f;
    uint16_t intervals;

    for (uint8_t joint = 0u; joint < 3u; ++joint) {
        float delta_deg = joint == ARM_JOINT_BASE_YAW ?
            fabsf(remainderf(target_q_deg[joint] - start_q_deg[joint],
                             360.0f)) :
            fabsf(target_q_deg[joint] - start_q_deg[joint]);

        max_delta_deg = fmaxf(max_delta_deg, delta_deg);
    }
    intervals = (uint16_t)ceilf(
        max_delta_deg / APP_ARM_FLOW_TRANSFER_SAMPLE_STEP_DEG);
    if (intervals < 1u) {
        intervals = 1u;
    }
    if ((uint32_t)intervals + 1u > ARM_LINEAR_MAX_SAMPLES) {
        return 0u;
    }

    for (uint16_t i = 0u; i <= intervals; ++i) {
        float ratio = (float)i / (float)intervals;
        float q_deg[3];
        float absolute_pitch_deg;
        Arm_Position_s wrist;
        Arm_Position_s center;

        q_deg[0] = start_q_deg[0] + ratio *
            remainderf(target_q_deg[0] - start_q_deg[0], 360.0f);
        q_deg[1] = start_q_deg[1] + ratio *
            (target_q_deg[1] - start_q_deg[1]);
        q_deg[2] = start_q_deg[2] + ratio *
            (target_q_deg[2] - start_q_deg[2]);
        absolute_pitch_deg = ArmToolSmallLinkPitchFromJoint(q_deg) +
            relative_pitch_deg;
        ArmForwardKinematics3DOF(q_deg[0], q_deg[1], q_deg[2], &wrist);
        if (!isfinite(absolute_pitch_deg) ||
            !ArmToolGetCenterFromWrist(&wrist, q_deg[0],
                                      absolute_pitch_deg, &center) ||
            !isfinite(center.y_mm) || !isfinite(center.z_mm)) {
            return 0u;
        }
        if (i == 0u && start_z_mm != NULL) {
            *start_z_mm = center.z_mm;
        }
        if (i > 0u && require_z_non_decreasing != 0u &&
            center.z_mm < previous_z_mm - 0.01f) {
            return 0u;
        }
        previous_z_mm = center.z_mm;
        *peak_abs_y_mm = fmaxf(*peak_abs_y_mm, fabsf(center.y_mm));
        if (fabsf(center.y_mm) > y_limit_mm) {
            return 0u;
        }
        if (i == intervals && end_z_mm != NULL) {
            *end_z_mm = center.z_mm;
        }
    }
    return 1u;
}

/** 按实际关节与ID1反馈复核AC放置准备的Y上限和第一段抬高量。 */
static uint8_t AppArmFlowTransferPathWithinLimits(
    float *relative_pitch_deg_out, uint8_t require_full_safe_pose)
{
    const Arm_State_s *arm = ArmGetState();
    const Arm_Tool_State_s *tool = ArmToolGetState();
    float relative_pitch_deg;
    float relative_pitch_over_deg;
    float peak_abs_y_mm = 0.0f;
    float start_z_mm = 0.0f;
    float waypoint_z_mm = 0.0f;
    float unused_z_mm = 0.0f;
    float z_raise_mm;
    uint8_t y_safe;
    uint8_t z_safe;

    g_app_arm_pick_place_test_debug.transfer_path_y_check_passed = 0u;
    g_app_arm_pick_place_test_debug.transfer_path_peak_abs_y_mm = 0.0f;
    g_app_arm_pick_place_test_debug.transfer_path_z_check_passed = 0u;
    g_app_arm_pick_place_test_debug.transfer_path_start_z_mm = 0.0f;
    g_app_arm_pick_place_test_debug.transfer_path_waypoint_z_mm = 0.0f;
    g_app_arm_pick_place_test_debug.transfer_path_z_raise_mm = 0.0f;
    g_app_arm_pick_place_test_debug.transfer_reject_reason =
        APP_ARM_TRANSFER_REJECT_NONE;
    if (arm == NULL || tool == NULL ||
        tool->servo_feedback_valid[0] == 0u ||
        !isfinite(tool->tool_pitch_feedback_deg) ||
        relative_pitch_deg_out == NULL) {
        g_app_arm_pick_place_test_debug.transfer_reject_reason =
            APP_ARM_TRANSFER_REJECT_FEEDBACK_INVALID;
        return 0u;
    }
    relative_pitch_deg = tool->tool_pitch_feedback_deg -
        ArmToolSmallLinkPitchFromJoint(arm->q_feedback_deg);
    if (!isfinite(relative_pitch_deg)) {
        g_app_arm_pick_place_test_debug.transfer_reject_reason =
            APP_ARM_TRANSFER_REJECT_RELATIVE_PITCH_RANGE;
        return 0u;
    }
    if (relative_pitch_deg < ARM_TOOL_PITCH_RELATIVE_MIN_DEG) {
        relative_pitch_over_deg =
            ARM_TOOL_PITCH_RELATIVE_MIN_DEG - relative_pitch_deg;
        if (relative_pitch_over_deg >
            APP_ARM_POSTURE_TEST_TRANSFER_PITCH_CLAMP_TOL_DEG) {
            g_app_arm_pick_place_test_debug.transfer_reject_reason =
                APP_ARM_TRANSFER_REJECT_RELATIVE_PITCH_RANGE;
            return 0u;
        }
        relative_pitch_deg = ARM_TOOL_PITCH_RELATIVE_MIN_DEG;
    } else if (relative_pitch_deg > ARM_TOOL_PITCH_RELATIVE_MAX_DEG) {
        relative_pitch_over_deg =
            relative_pitch_deg - ARM_TOOL_PITCH_RELATIVE_MAX_DEG;
        if (relative_pitch_over_deg >
            APP_ARM_POSTURE_TEST_TRANSFER_PITCH_CLAMP_TOL_DEG) {
            g_app_arm_pick_place_test_debug.transfer_reject_reason =
                APP_ARM_TRANSFER_REJECT_RELATIVE_PITCH_RANGE;
            return 0u;
        }
        relative_pitch_deg = ARM_TOOL_PITCH_RELATIVE_MAX_DEG;
    }
    *relative_pitch_deg_out = relative_pitch_deg;
    g_app_arm_pick_place_test_debug.pitch_target_deg =
        ArmToolSmallLinkPitchFromJoint(app_place_profile.safe_q_deg) +
        relative_pitch_deg;
    if (!isfinite(g_app_arm_pick_place_test_debug.pitch_target_deg)) {
        g_app_arm_pick_place_test_debug.transfer_reject_reason =
            APP_ARM_TRANSFER_REJECT_RELATIVE_PITCH_RANGE;
        return 0u;
    }
    y_safe = AppArmFlowTransferSegmentWithinLimits(
        arm->q_feedback_deg, app_place_profile.transfer_waypoint_q_deg,
        relative_pitch_deg, app_place_profile.transfer_path_y_max_mm,
        1u, &peak_abs_y_mm, &start_z_mm, &waypoint_z_mm);
    if (y_safe == 0u) {
        g_app_arm_pick_place_test_debug.transfer_reject_reason =
            peak_abs_y_mm > app_place_profile.transfer_path_y_max_mm ?
                APP_ARM_TRANSFER_REJECT_Y_LIMIT :
                APP_ARM_TRANSFER_REJECT_PATH_INVALID;
    }
    z_raise_mm = waypoint_z_mm - start_z_mm;
    /*
     * AC抓后收拢只要求过渡点相对当前抓取姿态至少抬高配置值。
     * 不要求精确等于配置值，否则不同视觉目标/反馈姿态会被误拒。
     */
    z_safe = y_safe != 0u &&
        z_raise_mm + app_place_profile.transfer_waypoint_z_tolerance_mm >=
            app_place_profile.transfer_waypoint_z_raise_mm;
    if (y_safe != 0u && z_safe == 0u) {
        g_app_arm_pick_place_test_debug.transfer_reject_reason =
            APP_ARM_TRANSFER_REJECT_Z_RAISE;
    }
    if (y_safe != 0u && z_safe != 0u &&
        require_full_safe_pose != 0u) {
        y_safe = AppArmFlowTransferSegmentWithinLimits(
            app_place_profile.transfer_waypoint_q_deg,
            app_place_profile.safe_q_deg, relative_pitch_deg,
            app_place_profile.transfer_path_y_max_mm, 0u,
            &peak_abs_y_mm, &unused_z_mm, &unused_z_mm);
        if (y_safe == 0u) {
            g_app_arm_pick_place_test_debug.transfer_reject_reason =
                peak_abs_y_mm > app_place_profile.transfer_path_y_max_mm ?
                    APP_ARM_TRANSFER_REJECT_Y_LIMIT :
                    APP_ARM_TRANSFER_REJECT_PATH_INVALID;
        }
    }
    g_app_arm_pick_place_test_debug.transfer_path_peak_abs_y_mm =
        peak_abs_y_mm;
    g_app_arm_pick_place_test_debug.transfer_path_y_check_passed = y_safe;
    g_app_arm_pick_place_test_debug.transfer_path_z_check_passed = z_safe;
    g_app_arm_pick_place_test_debug.transfer_path_start_z_mm = start_z_mm;
    g_app_arm_pick_place_test_debug.transfer_path_waypoint_z_mm =
        waypoint_z_mm;
    g_app_arm_pick_place_test_debug.transfer_path_z_raise_mm = z_raise_mm;
    return y_safe != 0u && z_safe != 0u;
}

/**
 * AC抓后先独立到达抬升waypoint。普通关节命令锁存动作开始时的
 * ID1相对角；只有该命令完成后，状态机才允许提交释放俯仰和后转命令。
 */
static uint8_t AppArmFlowSubmitTransferClearance(uint32_t now_ms)
{
    Arm_Joint_Command_s command;
    Arm_Command_Result_e result;
    float relative_pitch_deg = 0.0f;

    if (!AppArmFlowTransferPathWithinLimits(&relative_pitch_deg, 0u)) {
        AppArmFlowFail(APP_ARM_PICK_PLACE_FAILURE_COMMAND_SUBMIT,
                       (uint32_t)ARM_COMMAND_PREFLIGHT_FAILED, now_ms);
        return 0u;
    }
    memset(&command, 0, sizeof(command));
    command.command_id = AppArmCommandIdNext();
    command.move_type = ARM_MOVE_LINEAR;
    memcpy(command.q_deg, app_place_profile.transfer_waypoint_q_deg,
           sizeof(command.q_deg));
    command.waypoint_valid = 0u;
    command.tool_relative_pitch_valid = 0u;
    memcpy(g_app_arm_pick_place_test_debug.target_q_deg, command.q_deg,
           sizeof(command.q_deg));
    g_app_arm_pick_place_test_debug.pitch_target_deg =
        ArmToolSmallLinkPitchFromJoint(command.q_deg) + relative_pitch_deg;
    result = ArmSubmitJointCommand(&command);
    g_app_arm_pick_place_test_debug.active_command_id = command.command_id;
    g_app_arm_pick_place_test_debug.submit_result = (uint32_t)result;
    if (result != ARM_COMMAND_OK) {
        g_app_arm_pick_place_test_debug.command_result = (uint32_t)result;
        g_app_arm_pick_place_test_debug.transfer_reject_reason =
            APP_ARM_TRANSFER_REJECT_COMMAND_SUBMIT;
        AppArmFlowFail(APP_ARM_PICK_PLACE_FAILURE_COMMAND_SUBMIT,
                       (uint32_t)result, now_ms);
        return 0u;
    }
    return 1u;
}

/**
 * 放置准备阶段按profile选择直接到安全姿态，或经显式关节过渡点到达。
 * 轨迹层对每段逐1deg预检；AC约束路径显式带入钳位后的ID1相对俯仰。
 */
static uint8_t AppArmFlowSubmitTransferViaWaypoint(uint32_t now_ms)
{
    Arm_Joint_Command_s command;
    Arm_Command_Result_e result;
    float relative_pitch_deg = 0.0f;
    uint8_t relative_pitch_valid = 0u;

    if (app_place_profile.transfer_path_constraints_enabled != 0u &&
        !AppArmFlowTransferPathWithinLimits(&relative_pitch_deg, 1u)) {
        AppArmFlowFail(APP_ARM_PICK_PLACE_FAILURE_COMMAND_SUBMIT,
                       (uint32_t)ARM_COMMAND_PREFLIGHT_FAILED, now_ms);
        return 0u;
    }
    if (app_place_profile.transfer_path_constraints_enabled != 0u) {
        relative_pitch_valid = 1u;
    }
    memset(&command, 0, sizeof(command));
    command.command_id = AppArmCommandIdNext();
    command.move_type = ARM_MOVE_LINEAR;
    memcpy(command.q_deg, app_place_profile.safe_q_deg,
           sizeof(command.q_deg));
    command.waypoint_valid = app_place_profile.transfer_waypoint_valid;
    if (command.waypoint_valid != 0u) {
        memcpy(command.waypoint_q_deg,
               app_place_profile.transfer_waypoint_q_deg,
               sizeof(command.waypoint_q_deg));
    }
    command.tool_relative_pitch_valid = relative_pitch_valid;
    command.tool_relative_pitch_deg = relative_pitch_deg;
    memcpy(g_app_arm_pick_place_test_debug.target_q_deg, command.q_deg,
           sizeof(command.q_deg));
    result = ArmSubmitJointCommand(&command);
    g_app_arm_pick_place_test_debug.active_command_id = command.command_id;
    g_app_arm_pick_place_test_debug.submit_result = (uint32_t)result;
    if (result != ARM_COMMAND_OK) {
        g_app_arm_pick_place_test_debug.command_result = (uint32_t)result;
        g_app_arm_pick_place_test_debug.transfer_reject_reason =
            APP_ARM_TRANSFER_REJECT_COMMAND_SUBMIT;
        AppArmFlowFail(APP_ARM_PICK_PLACE_FAILURE_COMMAND_SUBMIT,
                       (uint32_t)result, now_ms);
        return 0u;
    }
    return 1u;
}

/** 坐标抓取子流程：底座对准->工具中心轨迹->等俯仰稳定->停留->闭合。 */
static void AppArmFlowPollPick(const Arm_Host_Status_s *host,
                               const Arm_Tool_State_s *tool,
                               uint32_t now_ms)
{
    switch (app_pick_step) {
    case APP_ARM_PICK_STEP_SUBMIT_BASE_AIM:
        if (AppArmFlowSubmitBaseAim(now_ms)) {
            AppArmFlowSetPickStep(APP_ARM_PICK_STEP_WAIT_BASE_AIM, now_ms);
        }
        break;

    case APP_ARM_PICK_STEP_WAIT_BASE_AIM:
        if (AppArmFlowCommandFinished(
                host, g_app_arm_pick_place_test_debug.active_command_id,
                now_ms)) {
            if (app_pick_target.approach_valid != 0u) {
                AppArmFlowSetPickStep(
                    APP_ARM_PICK_STEP_SUBMIT_APPROACH, now_ms);
            } else {
                AppArmFlowSetPickStep(
                    APP_ARM_PICK_STEP_SUBMIT_TARGET, now_ms);
            }
        }
        break;

    case APP_ARM_PICK_STEP_SUBMIT_APPROACH:
        if (AppArmFlowSubmitPickApproach(now_ms)) {
            AppArmFlowSetPickStep(
                APP_ARM_PICK_STEP_WAIT_APPROACH, now_ms);
        }
        break;

    case APP_ARM_PICK_STEP_WAIT_APPROACH:
        if (AppArmFlowCommandFinished(
                host, g_app_arm_pick_place_test_debug.active_command_id,
                now_ms)) {
            AppArmFlowSetPickStep(APP_ARM_PICK_STEP_SUBMIT_TARGET, now_ms);
        }
        break;

    case APP_ARM_PICK_STEP_SUBMIT_TARGET:
        if (AppArmFlowSubmitPickTarget(now_ms)) {
            AppArmFlowSetPickStep(APP_ARM_PICK_STEP_WAIT_TARGET, now_ms);
        }
        break;

    case APP_ARM_PICK_STEP_WAIT_TARGET:
        if (AppArmFlowCommandFinished(
                host, g_app_arm_pick_place_test_debug.active_command_id,
                now_ms)) {
            app_pitch_stable_tick = 0u;
            AppArmFlowSetPickStep(
                APP_ARM_PICK_STEP_WAIT_PITCH_STABLE, now_ms);
        }
        break;

    case APP_ARM_PICK_STEP_WAIT_PITCH_STABLE:
        /* 工具中心命令直接指定世界绝对俯仰。 */
        if (tool == NULL || tool->servo_feedback_valid[0] == 0u ||
            !isfinite(tool->tool_pitch_feedback_deg) ||
            fabsf(tool->tool_pitch_feedback_deg -
                   g_app_arm_pick_place_test_debug.pitch_target_deg) >
                APP_ARM_TOOL_CENTER_PITCH_TOLERANCE_DEG) {
            app_pitch_stable_tick = 0u;
            return;
        }
        if (app_pitch_stable_tick == 0u) {
            app_pitch_stable_tick = now_ms;
        }
        if ((uint32_t)(now_ms - app_pitch_stable_tick) >=
            APP_ARM_TOOL_CENTER_PITCH_STABLE_MS) {
            AppArmFlowSetPickStep(APP_ARM_PICK_STEP_PICK_DWELL, now_ms);
        }
        break;

    case APP_ARM_PICK_STEP_PICK_DWELL:
        if ((uint32_t)(now_ms - app_flow_step_tick) >=
            APP_ARM_PICK_DWELL_MS) {
            AppArmFlowSetPickStep(APP_ARM_PICK_STEP_SUBMIT_CLOSE, now_ms);
        }
        break;

    case APP_ARM_PICK_STEP_SUBMIT_CLOSE:
        if (AppArmFlowSubmitTool(
                ARM_TOOL_ACTION_GRIPPER_CLOSE, 0.0f, now_ms)) {
            AppArmFlowSetPickStep(APP_ARM_PICK_STEP_WAIT_CLOSE, now_ms);
        }
        break;

    case APP_ARM_PICK_STEP_WAIT_CLOSE:
        /* FORCED_HELD after four contact-relief attempts completes the grip. */
        /*
         * 提前受阻并完成分级卸力后进入HELD_CONTACT，或无阻挡正常到660
         * 进入CLOSED_EMPTY，
         * 底层都会把工具命令标记完成；两种结果均视为抓取步骤成功。
         */
        if (AppArmFlowCommandFinished(
                host, g_app_arm_pick_place_test_debug.active_command_id,
                now_ms)) {
            AppArmFlowSetPickStep(
                APP_ARM_PICK_STEP_POST_GRIP_DWELL, now_ms);
        }
        break;

    case APP_ARM_PICK_STEP_POST_GRIP_DWELL:
        if ((uint32_t)(now_ms - app_flow_step_tick) >=
            APP_ARM_POST_GRIP_DWELL_MS) {
            AppArmFlowSetPickStep(APP_ARM_PICK_STEP_DONE, now_ms);
            app_flow_status = APP_ARM_FLOW_DONE;
            app_flow_active = APP_ARM_FLOW_NONE;
        }
        break;

    case APP_ARM_PICK_STEP_DONE:
    case APP_ARM_PICK_STEP_FAILED:
    case APP_ARM_PICK_STEP_IDLE:
    default:
        break;
    }
}

/** 对应侧安全点放置子流程：受约束收拢->同侧转后方->释放->转回。 */
static void AppArmFlowPollPlace(const Arm_Host_Status_s *host,
                                const Arm_Tool_State_s *tool,
                                uint32_t now_ms)
{
    switch (app_place_step) {
    case APP_ARM_PLACE_STEP_SUBMIT_TRANSFER:
        if (app_place_profile.transfer_path_constraints_enabled != 0u) {
            if (AppArmFlowSubmitTransferClearance(now_ms)) {
                AppArmFlowSetPlaceStep(
                    APP_ARM_PLACE_STEP_WAIT_TRANSFER, now_ms);
            }
        } else if (AppArmFlowSubmitTransferViaWaypoint(now_ms)) {
            AppArmFlowSetPlaceStep(
                APP_ARM_PLACE_STEP_WAIT_TRANSFER, now_ms);
        }
        break;

    case APP_ARM_PLACE_STEP_WAIT_TRANSFER:
        if (AppArmFlowCommandFinished(
                host, g_app_arm_pick_place_test_debug.active_command_id,
                now_ms)) {
            AppArmFlowSetPlaceStep(
                APP_ARM_PLACE_STEP_SUBMIT_ROTATE_TO_PLACE, now_ms);
        }
        break;

    case APP_ARM_PLACE_STEP_SUBMIT_REAR_STAGING:
        if (AppArmFlowSubmitRearRotateStaging(now_ms)) {
            AppArmFlowSetPlaceStep(
                APP_ARM_PLACE_STEP_WAIT_REAR_STAGING, now_ms);
        }
        break;

    case APP_ARM_PLACE_STEP_WAIT_REAR_STAGING:
        if (AppArmFlowCommandFinished(
                host, g_app_arm_pick_place_test_debug.active_command_id,
                now_ms)) {
            AppArmFlowSetPlaceStep(
                APP_ARM_PLACE_STEP_SUBMIT_ROTATE_TO_PLACE, now_ms);
        }
        break;

    case APP_ARM_PLACE_STEP_SUBMIT_ROTATE_TO_PLACE:
        if ((app_place_profile.transfer_path_constraints_enabled != 0u &&
             AppArmFlowSubmitDirectedReleaseRotation(now_ms)) ||
            (app_place_profile.transfer_path_constraints_enabled == 0u &&
             AppArmFlowSubmitDirectedBaseRotation(
                 app_place_profile.rotate_to_place_waypoint_q1_deg,
                 app_place_profile.rotate_to_place_target_q1_deg, now_ms))) {
            AppArmFlowSetPlaceStep(
                APP_ARM_PLACE_STEP_WAIT_ROTATE_TO_PLACE, now_ms);
        }
        break;

    case APP_ARM_PLACE_STEP_WAIT_ROTATE_TO_PLACE:
        if (AppArmFlowCommandFinished(
                host, g_app_arm_pick_place_test_debug.active_command_id,
                now_ms)) {
            AppArmFlowSetPlaceStep(
                app_place_profile.transfer_path_constraints_enabled != 0u ?
                    APP_ARM_PLACE_STEP_WAIT_RELEASE_PITCH :
                    APP_ARM_PLACE_STEP_SUBMIT_RELEASE_POSE,
                now_ms);
        }
        break;

    case APP_ARM_PLACE_STEP_SUBMIT_RELEASE_POSE:
        /* 显式保持profile后方q1，禁止用瞬时反馈重锁定底座目标。 */
        if (AppArmFlowSubmitJointWithRelativePitch(
                1u, app_place_profile.release_q_deg[ARM_JOINT_BASE_YAW],
                1u, app_place_profile.release_q_deg[ARM_JOINT_SHOULDER],
                1u, app_place_profile.release_q_deg[ARM_JOINT_ELBOW],
                app_place_profile.release_tool_relative_pitch_deg,
                now_ms)) {
            AppArmFlowSetPlaceStep(
                APP_ARM_PLACE_STEP_WAIT_RELEASE_POSE, now_ms);
        }
        break;

    case APP_ARM_PLACE_STEP_WAIT_RELEASE_POSE:
        if (AppArmFlowCommandFinished(
                host, g_app_arm_pick_place_test_debug.active_command_id,
                now_ms)) {
            AppArmFlowSetPlaceStep(
                APP_ARM_PLACE_STEP_WAIT_RELEASE_PITCH, now_ms);
        }
        break;

    case APP_ARM_PLACE_STEP_WAIT_RELEASE_PITCH:
        /* 联合关节动作已经完成；确认ID1反馈到位后才允许释放。 */
        if (tool != NULL && tool->servo_feedback_valid[0] != 0u &&
            tool->servo_arrived[0] != 0u) {
            AppArmFlowSetPlaceStep(APP_ARM_PLACE_STEP_SUBMIT_OPEN, now_ms);
        } else if ((uint32_t)(now_ms - app_flow_step_tick) >=
                   app_place_profile.release_pitch_wait_timeout_ms) {
            AppArmFlowFail(APP_ARM_PICK_PLACE_FAILURE_COMMAND_EXECUTION,
                           (uint32_t)ARM_COMMAND_NOT_READY, now_ms);
        }
        break;

    case APP_ARM_PLACE_STEP_SUBMIT_OPEN:
        if (AppArmFlowSubmitTool(
                ARM_TOOL_ACTION_GRIPPER_OPEN, 0.0f, now_ms)) {
            AppArmFlowSetPlaceStep(APP_ARM_PLACE_STEP_WAIT_OPEN, now_ms);
        }
        break;

    case APP_ARM_PLACE_STEP_WAIT_OPEN:
        if (AppArmFlowCommandFinished(
                host, g_app_arm_pick_place_test_debug.active_command_id,
                now_ms)) {
            AppArmFlowSetPlaceStep(
                app_place_profile.transfer_path_constraints_enabled != 0u ?
                    APP_ARM_PLACE_STEP_SUBMIT_ROTATE_TO_FRONT :
                    APP_ARM_PLACE_STEP_SUBMIT_RELEASE_CLEARANCE,
                now_ms);
        }
        break;

    case APP_ARM_PLACE_STEP_SUBMIT_RELEASE_CLEARANCE:
        /* 释放后抬臂时继续保持profile后方q1，不采样瞬时反馈。 */
        if (AppArmFlowSubmitJoint(
                1u, app_place_profile.release_clearance_q_deg[
                    ARM_JOINT_BASE_YAW],
                1u, app_place_profile.release_clearance_q_deg[
                    ARM_JOINT_SHOULDER],
                1u, app_place_profile.release_clearance_q_deg[
                    ARM_JOINT_ELBOW],
                now_ms)) {
            AppArmFlowSetPlaceStep(
                APP_ARM_PLACE_STEP_WAIT_RELEASE_CLEARANCE, now_ms);
        }
        break;

    case APP_ARM_PLACE_STEP_WAIT_RELEASE_CLEARANCE:
        if (AppArmFlowCommandFinished(
                host, g_app_arm_pick_place_test_debug.active_command_id,
                now_ms)) {
            AppArmFlowSetPlaceStep(
                APP_ARM_PLACE_STEP_SUBMIT_ROTATE_TO_FRONT, now_ms);
        }
        break;

    case APP_ARM_PLACE_STEP_SUBMIT_ROTATE_TO_FRONT:
        if ((app_place_profile.transfer_path_constraints_enabled != 0u &&
             AppArmFlowSubmitFrontRotationViaReleaseClearance(now_ms)) ||
            (app_place_profile.transfer_path_constraints_enabled == 0u &&
             AppArmFlowSubmitDirectedBaseRotation(
                 app_place_profile.rotate_to_front_waypoint_q1_deg,
                 app_place_profile.rotate_to_front_target_q1_deg, now_ms))) {
            AppArmFlowSetPlaceStep(
                APP_ARM_PLACE_STEP_WAIT_ROTATE_TO_FRONT, now_ms);
        }
        break;

    case APP_ARM_PLACE_STEP_WAIT_ROTATE_TO_FRONT:
        if (AppArmFlowCommandFinished(
                host, g_app_arm_pick_place_test_debug.active_command_id,
                now_ms)) {
            AppArmFlowSetPlaceStep(APP_ARM_PLACE_STEP_DONE, now_ms);
            app_flow_status = APP_ARM_FLOW_DONE;
            app_flow_active = APP_ARM_FLOW_NONE;
        }
        break;

    case APP_ARM_PLACE_STEP_DONE:
    case APP_ARM_PLACE_STEP_FAILED:
    case APP_ARM_PLACE_STEP_IDLE:
    default:
        break;
    }
}

void AppArmFlowInit(void)
{
    memset(&g_app_arm_pick_place_test_debug, 0,
           sizeof(g_app_arm_pick_place_test_debug));
    app_flow_active = APP_ARM_FLOW_NONE;
    app_flow_status = APP_ARM_FLOW_IDLE;
    app_pick_step = APP_ARM_PICK_STEP_IDLE;
    app_place_step = APP_ARM_PLACE_STEP_IDLE;
    app_flow_step_tick = 0u;
    app_pitch_stable_tick = 0u;
}

uint8_t AppArmFlowStartPick(const App_Arm_Pick_Target_s *target,
                            uint32_t now_ms)
{
    if (target == NULL || app_flow_active != APP_ARM_FLOW_NONE ||
        app_flow_status == APP_ARM_FLOW_FAILED ||
        !isfinite(target->x_mm) || !isfinite(target->y_mm) ||
        !isfinite(target->z_mm) ||
        (fabsf(target->x_mm) < 0.001f &&
         fabsf(target->y_mm) < 0.001f) ||
        target->approach_valid > 1u ||
        (target->approach_valid != 0u &&
         (!isfinite(target->approach_x_mm) ||
          !isfinite(target->approach_y_mm) ||
          !isfinite(target->approach_z_mm) ||
          (fabsf(target->approach_x_mm) < 0.001f &&
           fabsf(target->approach_y_mm) < 0.001f))) ||
        !AppArmFlowValueInRange(target->tool_pitch_deg,
                                ARM_USB_TOOL_PITCH_MIN_DEG,
                                ARM_USB_TOOL_PITCH_MAX_DEG)) {
        return 0u;
    }
    app_pick_target = *target;
    app_flow_active = APP_ARM_FLOW_PICK;
    app_flow_status = APP_ARM_FLOW_RUNNING;
    g_app_arm_pick_place_test_debug.target_center_mm[0] = target->x_mm;
    g_app_arm_pick_place_test_debug.target_center_mm[1] = target->y_mm;
    g_app_arm_pick_place_test_debug.target_center_mm[2] = target->z_mm;
    g_app_arm_pick_place_test_debug.pitch_target_deg =
        target->tool_pitch_deg;
    {
        Arm_Position_s center;
        Arm_Position_s wrist;
        float base_q1_deg = atan2f(target->y_mm, target->x_mm) *
            APP_ARM_FLOW_RAD_TO_DEG;

        center.x_mm = target->x_mm;
        center.y_mm = target->y_mm;
        center.z_mm = target->z_mm;
        if (ArmToolGetWristFromCenter(&center, base_q1_deg,
                                     target->tool_pitch_deg, &wrist)) {
            g_app_arm_pick_place_test_debug.target_wrist_mm[0] = wrist.x_mm;
            g_app_arm_pick_place_test_debug.target_wrist_mm[1] = wrist.y_mm;
            g_app_arm_pick_place_test_debug.target_wrist_mm[2] = wrist.z_mm;
        }
    }
    AppArmFlowSetPickStep(APP_ARM_PICK_STEP_SUBMIT_BASE_AIM, now_ms);
    return 1u;
}

App_Arm_Flow_Start_Result_e AppArmFlowStartPlace(
    const App_Arm_Place_Profile_s *profile, uint32_t now_ms)
{
    App_Arm_Flow_Start_Result_e result;

    if (app_flow_status == APP_ARM_FLOW_FAILED) {
        result = APP_ARM_FLOW_START_FAILED;
    } else if (app_flow_active != APP_ARM_FLOW_NONE) {
        result = APP_ARM_FLOW_START_BUSY;
    } else if (profile == NULL || profile->configured == 0u) {
        result = APP_ARM_FLOW_START_NOT_CONFIGURED;
    } else if (!AppArmFlowPlaceProfileValid(profile)) {
        result = APP_ARM_FLOW_START_INVALID;
    } else {
        app_place_profile = *profile;
        app_flow_active = APP_ARM_FLOW_PLACE;
        app_flow_status = APP_ARM_FLOW_RUNNING;
        g_app_arm_pick_place_test_debug.place_profile_id =
            profile->profile_id;
        g_app_arm_pick_place_test_debug.transfer_path_y_limit_mm =
            profile->transfer_path_y_max_mm;
        g_app_arm_pick_place_test_debug.transfer_path_peak_abs_y_mm = 0.0f;
        g_app_arm_pick_place_test_debug.transfer_path_y_check_passed = 0u;
        g_app_arm_pick_place_test_debug.transfer_path_z_check_passed = 0u;
        g_app_arm_pick_place_test_debug.transfer_path_start_z_mm = 0.0f;
        g_app_arm_pick_place_test_debug.transfer_path_waypoint_z_mm = 0.0f;
        g_app_arm_pick_place_test_debug.transfer_path_z_raise_mm = 0.0f;
        g_app_arm_pick_place_test_debug.transfer_reject_reason =
            APP_ARM_TRANSFER_REJECT_NONE;
        AppArmFlowSetPlaceStep(APP_ARM_PLACE_STEP_SUBMIT_TRANSFER, now_ms);
        result = APP_ARM_FLOW_START_ACCEPTED;
    }
    g_app_arm_pick_place_test_debug.last_start_result = result;
    return result;
}

App_Arm_Flow_Status_e AppArmFlowPoll(uint32_t now_ms)
{
    const Arm_State_s *arm = ArmGetState();
    const Arm_Tool_State_s *tool = ArmToolGetState();
    Arm_Host_Status_s host;

    AppArmFlowUpdateWatch(arm, now_ms);
    if (app_flow_status == APP_ARM_FLOW_FAILED) {
        return app_flow_status;
    }
    if (arm == NULL || ArmGetHostStatus(&host) == 0u) {
        return app_flow_status;
    }
    /* 机械臂主机故障/急停时立即锁存失败，与旧抓放测试行为一致。 */
    if (host.state == ARM_HOST_STATE_FAULT ||
        host.state == ARM_HOST_STATE_ESTOP) {
        g_app_arm_pick_place_test_debug.arm_fault_code = host.fault_code;
        AppArmFlowFail(APP_ARM_PICK_PLACE_FAILURE_ARM,
                       host.fault_code, now_ms);
        return app_flow_status;
    }
    if (app_flow_active == APP_ARM_FLOW_PICK) {
        AppArmFlowPollPick(&host, tool, now_ms);
    } else if (app_flow_active == APP_ARM_FLOW_PLACE) {
        AppArmFlowPollPlace(&host, tool, now_ms);
    }
    return app_flow_status;
}

App_Arm_Flow_Status_e AppArmFlowGetStatus(void)
{
    return app_flow_status;
}

void AppArmFlowAbort(uint32_t now_ms)
{
    app_flow_active = APP_ARM_FLOW_NONE;
    app_flow_status = APP_ARM_FLOW_IDLE;
    app_pick_step = APP_ARM_PICK_STEP_IDLE;
    app_place_step = APP_ARM_PLACE_STEP_IDLE;
    app_flow_step_tick = now_ms;
    app_pitch_stable_tick = 0u;
    g_app_arm_pick_place_test_debug.active_flow =
        (uint8_t)APP_ARM_FLOW_NONE;
    g_app_arm_pick_place_test_debug.flow_status =
        (uint8_t)APP_ARM_FLOW_IDLE;
    g_app_arm_pick_place_test_debug.pick_step =
        (uint8_t)APP_ARM_PICK_STEP_IDLE;
    g_app_arm_pick_place_test_debug.place_step =
        (uint8_t)APP_ARM_PLACE_STEP_IDLE;
    g_app_arm_pick_place_test_debug.active_command_id = 0u;
    g_app_arm_pick_place_test_debug.submit_result = 0u;
    g_app_arm_pick_place_test_debug.command_state =
        (uint32_t)ARM_COMMAND_STATE_NONE;
    g_app_arm_pick_place_test_debug.command_result =
        (uint32_t)ARM_COMMAND_OK;
    g_app_arm_pick_place_test_debug.failure_source =
        APP_ARM_PICK_PLACE_FAILURE_NONE;
    g_app_arm_pick_place_test_debug.fault = 0u;
    g_app_arm_pick_place_test_debug.arm_fault_code = 0u;
    g_app_arm_pick_place_test_debug.transfer_reject_reason =
        APP_ARM_TRANSFER_REJECT_NONE;
    g_app_arm_pick_place_test_debug.state_elapsed_ms = 0u;
}

#else

uint8_t AppArmFlowBuildPickStaging(float target_x_mm, float target_y_mm,
                                   App_Arm_Pick_Staging_s *staging)
{
    (void)target_x_mm;
    (void)target_y_mm;
    (void)staging;
    return 0u;
}

uint8_t AppArmFlowSelectReachablePickAdvance(
    App_Arm_Pick_Target_s *target, float advance_sign,
    float requested_advance_mm, float sample_step_mm,
    App_Arm_Advance_Result_s *result)
{
    (void)target;
    (void)advance_sign;
    (void)requested_advance_mm;
    (void)sample_step_mm;
    if (result != NULL) {
        memset(result, 0, sizeof(*result));
        result->requested_mm = requested_advance_mm;
        result->reject_reason = APP_ARM_ADVANCE_REJECT_INVALID;
    }
    return 0u;
}

void AppArmFlowInit(void)
{
}

uint8_t AppArmFlowStartPick(const App_Arm_Pick_Target_s *target,
                            uint32_t now_ms)
{
    (void)target;
    (void)now_ms;
    return 0u;
}

App_Arm_Flow_Start_Result_e AppArmFlowStartPlace(
    const App_Arm_Place_Profile_s *profile, uint32_t now_ms)
{
    (void)profile;
    (void)now_ms;
    return APP_ARM_FLOW_START_FAILED;
}

App_Arm_Flow_Status_e AppArmFlowPoll(uint32_t now_ms)
{
    (void)now_ms;
    return APP_ARM_FLOW_IDLE;
}

App_Arm_Flow_Status_e AppArmFlowGetStatus(void)
{
    return APP_ARM_FLOW_IDLE;
}

void AppArmFlowAbort(uint32_t now_ms)
{
    (void)now_ms;
}

#endif /* arm, posture test or host control with tool-center flow */
