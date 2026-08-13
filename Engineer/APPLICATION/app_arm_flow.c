/**
 * @file app_arm_flow.c
 * @brief 教导位姿抓取和固定角度放置两个机械臂子流程的状态机实现。
 *
 * 提交函数与状态顺序从旧 app_runtime.c 抓放大状态机原样平移；
 * 所有实机验证过的角度、方向锚点和俯仰换算逻辑保持不变。
 */

#include "app_arm_flow.h"

#include "app_config.h"

/* Watch符号沿用旧抓放测试名称；打点模式下也保留定义便于统一观察。 */
App_Arm_Pick_Place_Test_Debug_s g_app_arm_pick_place_test_debug;

#if APP_ARM_ENABLED && APP_ARM_TOOL_CENTER_TEST_ENABLE

#include <math.h>
#include <string.h>

#include "arm.h"
#include "arm_config.h"
#include "arm_tool.h"

/* 子流程运行时状态；同一时刻最多一个子流程活动。 */
static App_Arm_Flow_Active_e app_flow_active;
static App_Arm_Flow_Status_e app_flow_status;
static App_Arm_Pick_Step_e app_pick_step;
static App_Arm_Place_Step_e app_place_step;
static App_Arm_Pick_Target_s app_pick_target;
static App_Arm_Place_Profile_s app_place_profile;
static uint32_t app_flow_step_tick;    /* 当前步骤进入时刻ms。 */
static uint32_t app_flow_command_seq;  /* 递增命令ID；邮箱按ID去重。 */
static uint32_t app_pitch_stable_tick; /* 俯仰反馈连续稳定的起始时刻。 */

static uint32_t AppArmFlowNextCommandId(void)
{
    app_flow_command_seq++;
    if (app_flow_command_seq == 0u) {
        app_flow_command_seq = APP_ARM_PICK_PLACE_COMMAND_ID_BASE + 1u;
    }
    return app_flow_command_seq;
}

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
    command.command_id = AppArmFlowNextCommandId();
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
    command.command_id = AppArmFlowNextCommandId();
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
    command.command_id = AppArmFlowNextCommandId();
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
    command.command_id = AppArmFlowNextCommandId();
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
        !AppArmFlowPoseValid(profile->safe_q_deg) ||
        !AppArmFlowPoseValid(profile->release_q_deg) ||
        !AppArmFlowPoseValid(profile->release_clearance_q_deg) ||
        !AppArmFlowPoseValid(profile->restore_q_deg)) {
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
    if (fabsf(profile->release_q_deg[ARM_JOINT_BASE_YAW] -
              profile->rotate_to_place_target_q1_deg) > 0.01f ||
        fabsf(profile->release_clearance_q_deg[ARM_JOINT_BASE_YAW] -
              profile->rotate_to_place_target_q1_deg) > 0.01f ||
        fabsf(profile->restore_q_deg[ARM_JOINT_BASE_YAW] -
              profile->rotate_to_front_target_q1_deg) > 0.01f) {
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

/**
 * 底座对准：仅提交底座关节旋转到教导q1。对准在低位进行（工具中心
 * z约203mm），q1到+/-90deg会使工具中心跨过X=0并触发210mm跨区高度
 * 拒绝；限幅保持X>0，剩余角度由随后的教导位姿联合命令完成（该段
 * 跨X=0时工具中心已接近教导高度，高于跨区线）。
 */
static uint8_t AppArmFlowSubmitBaseAim(uint32_t now_ms)
{
    float base_q1_deg = app_pick_target.q_deg[ARM_JOINT_BASE_YAW];

    if (base_q1_deg > APP_ARM_PICK_BASE_AIM_MAX_ABS_Q1_DEG) {
        base_q1_deg = APP_ARM_PICK_BASE_AIM_MAX_ABS_Q1_DEG;
    } else if (base_q1_deg < -APP_ARM_PICK_BASE_AIM_MAX_ABS_Q1_DEG) {
        base_q1_deg = -APP_ARM_PICK_BASE_AIM_MAX_ABS_Q1_DEG;
    }
    return AppArmFlowSubmitJoint(
        1u, base_q1_deg,
        0u, 0.0f,
        0u, 0.0f, now_ms);
}

/** 教导位姿抓取子流程：底座对准->联合位姿->等俯仰稳定->停留->闭合->停留。 */
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
            AppArmFlowSetPickStep(APP_ARM_PICK_STEP_SUBMIT_POSE, now_ms);
        }
        break;

    case APP_ARM_PICK_STEP_SUBMIT_POSE:
        if (AppArmFlowSubmitJointWithRelativePitch(
                1u, app_pick_target.q_deg[ARM_JOINT_BASE_YAW],
                1u, app_pick_target.q_deg[ARM_JOINT_SHOULDER],
                1u, app_pick_target.q_deg[ARM_JOINT_ELBOW],
                app_pick_target.tool_relative_pitch_deg, now_ms)) {
            AppArmFlowSetPickStep(APP_ARM_PICK_STEP_WAIT_POSE, now_ms);
        }
        break;

    case APP_ARM_PICK_STEP_WAIT_POSE:
        if (AppArmFlowCommandFinished(
                host, g_app_arm_pick_place_test_debug.active_command_id,
                now_ms)) {
            app_pitch_stable_tick = 0u;
            AppArmFlowSetPickStep(
                APP_ARM_PICK_STEP_WAIT_PITCH_STABLE, now_ms);
        }
        break;

    case APP_ARM_PICK_STEP_WAIT_PITCH_STABLE:
        /* 目标绝对俯仰由联合命令按教导关节角+相对角换算并已写入Watch。 */
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

/** 对应侧安全点放置子流程：抬臂->同侧转后方->释放->同侧转回前方。 */
static void AppArmFlowPollPlace(const Arm_Host_Status_s *host,
                                const Arm_Tool_State_s *tool,
                                uint32_t now_ms)
{
    switch (app_place_step) {
    case APP_ARM_PLACE_STEP_SUBMIT_TRANSFER:
        if (AppArmFlowSubmitJoint(
                1u, app_place_profile.safe_q_deg[ARM_JOINT_BASE_YAW],
                1u, app_place_profile.safe_q_deg[ARM_JOINT_SHOULDER],
                1u, app_place_profile.safe_q_deg[ARM_JOINT_ELBOW],
                now_ms)) {
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

    case APP_ARM_PLACE_STEP_SUBMIT_ROTATE_TO_PLACE:
        if (AppArmFlowSubmitDirectedBaseRotation(
                app_place_profile.rotate_to_place_waypoint_q1_deg,
                app_place_profile.rotate_to_place_target_q1_deg, now_ms)) {
            AppArmFlowSetPlaceStep(
                APP_ARM_PLACE_STEP_WAIT_ROTATE_TO_PLACE, now_ms);
        }
        break;

    case APP_ARM_PLACE_STEP_WAIT_ROTATE_TO_PLACE:
        if (AppArmFlowCommandFinished(
                host, g_app_arm_pick_place_test_debug.active_command_id,
                now_ms)) {
            AppArmFlowSetPlaceStep(
                APP_ARM_PLACE_STEP_SUBMIT_RELEASE_POSE, now_ms);
        }
        break;

    case APP_ARM_PLACE_STEP_SUBMIT_RELEASE_POSE:
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
                APP_ARM_PLACE_STEP_SUBMIT_RELEASE_CLEARANCE, now_ms);
        }
        break;

    case APP_ARM_PLACE_STEP_SUBMIT_RELEASE_CLEARANCE:
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
        if (AppArmFlowSubmitDirectedBaseRotation(
                app_place_profile.rotate_to_front_waypoint_q1_deg,
                app_place_profile.rotate_to_front_target_q1_deg, now_ms)) {
            AppArmFlowSetPlaceStep(
                APP_ARM_PLACE_STEP_WAIT_ROTATE_TO_FRONT, now_ms);
        }
        break;

    case APP_ARM_PLACE_STEP_WAIT_ROTATE_TO_FRONT:
        if (AppArmFlowCommandFinished(
                host, g_app_arm_pick_place_test_debug.active_command_id,
                now_ms)) {
            AppArmFlowSetPlaceStep(
                APP_ARM_PLACE_STEP_SUBMIT_RESTORE_TRANSFER, now_ms);
        }
        break;

    case APP_ARM_PLACE_STEP_SUBMIT_RESTORE_TRANSFER:
        if (AppArmFlowSubmitJoint(
                1u, app_place_profile.restore_q_deg[ARM_JOINT_BASE_YAW],
                1u, app_place_profile.restore_q_deg[ARM_JOINT_SHOULDER],
                1u, app_place_profile.restore_q_deg[ARM_JOINT_ELBOW],
                now_ms)) {
            AppArmFlowSetPlaceStep(
                APP_ARM_PLACE_STEP_WAIT_RESTORE_TRANSFER, now_ms);
        }
        break;

    case APP_ARM_PLACE_STEP_WAIT_RESTORE_TRANSFER:
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
    app_flow_command_seq = APP_ARM_PICK_PLACE_COMMAND_ID_BASE;
    app_flow_step_tick = 0u;
    app_pitch_stable_tick = 0u;
}

uint8_t AppArmFlowStartPick(const App_Arm_Pick_Target_s *target,
                            uint32_t now_ms)
{
    if (target == NULL || app_flow_active != APP_ARM_FLOW_NONE ||
        app_flow_status == APP_ARM_FLOW_FAILED) {
        return 0u;
    }
    app_pick_target = *target;
    app_flow_active = APP_ARM_FLOW_PICK;
    app_flow_status = APP_ARM_FLOW_RUNNING;
    g_app_arm_pick_place_test_debug.target_center_mm[0] = target->x_mm;
    g_app_arm_pick_place_test_debug.target_center_mm[1] = target->y_mm;
    g_app_arm_pick_place_test_debug.target_center_mm[2] = target->z_mm;
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

#endif /* APP_ARM_ENABLED && APP_ARM_TOOL_CENTER_TEST_ENABLE */
