/**
 * @file app_arm_side_pick_place.c
 * @brief 单次左侧或右侧水果抓取、释放及回正的非阻塞状态机。
 */

#include "app_arm_side_pick_place.h"

#include <math.h>
#include <string.h>

#include "app_arm_command_id.h"
#include "app_config.h"
#include "arm.h"
#include "arm_tool.h"

App_Arm_Posture_Test_Debug_s g_app_arm_posture_test_debug;

#if APP_ARM_ENABLED || APP_ARM_POSTURE_TEST_ENABLED || \
    APP_HOST_CONTROL_ENABLED

static uint32_t app_arm_side_pitch_stable_tick;
static uint32_t app_arm_side_state_tick;
static App_Arm_Place_Profile_s app_arm_side_place_profile;
static App_Arm_Side_Pick_Place_Status_e app_arm_side_status;

static void AppArmSidePickPlaceSetState(App_Arm_Posture_Test_State_e state,
                                        uint32_t now_ms)
{
    g_app_arm_posture_test_debug.state = state;
    g_app_arm_posture_test_debug.state_elapsed_ms = 0u;
    app_arm_side_state_tick = now_ms;
}

static void AppArmSidePickPlaceSetStatus(
    App_Arm_Side_Pick_Place_Status_e status)
{
    app_arm_side_status = status;
    g_app_arm_posture_test_debug.operation_status = (uint8_t)status;
}

static void AppArmSidePickPlaceFail(uint32_t now_ms)
{
    AppArmSidePickPlaceSetState(APP_ARM_POSTURE_TEST_FAILED, now_ms);
    AppArmSidePickPlaceSetStatus(APP_ARM_SIDE_PICK_PLACE_FAILED);
}

uint8_t AppArmSidePickPlaceBuildPlaceProfile(
    App_Fruit_Side_e side, App_Arm_Place_Profile_s *profile)
{
    if (profile == NULL) {
        return 0u;
    }
    memset(profile, 0, sizeof(*profile));
    if (side != APP_FRUIT_SIDE_LEFT && side != APP_FRUIT_SIDE_RIGHT) {
        return 0u;
    }
    /* AC区当前复用已验证的A区左右放置profile；BD区必须独立配置。 */
    if (AppFruitGetPlaceProfile(APP_FRUIT_AREA_A, side, profile) == 0u) {
        return 0u;
    }

    /* AC抓后专用：向内收时至少抬高约20mm，公共A区profile保持原语义。 */
    profile->transfer_waypoint_valid = 1u;
    profile->transfer_path_constraints_enabled = 1u;
    profile->transfer_waypoint_q_deg[ARM_JOINT_BASE_YAW] =
        profile->safe_q_deg[ARM_JOINT_BASE_YAW];
    profile->transfer_waypoint_q_deg[ARM_JOINT_SHOULDER] =
        APP_ARM_POSTURE_TEST_TRANSFER_WAYPOINT_Q2_DEG;
    profile->transfer_waypoint_q_deg[ARM_JOINT_ELBOW] =
        APP_ARM_POSTURE_TEST_TRANSFER_WAYPOINT_Q3_DEG;
    profile->transfer_path_y_max_mm =
        APP_ARM_POSTURE_TEST_TRANSFER_PATH_Y_MAX_MM;
    profile->transfer_waypoint_z_raise_mm =
        APP_ARM_POSTURE_TEST_TRANSFER_Z_RAISE_MM;
    profile->transfer_waypoint_z_tolerance_mm =
        APP_ARM_POSTURE_TEST_TRANSFER_Z_TOLERANCE_MM;
    return 1u;
}

/** 装载AC区一侧镜像抓放参数，并清空本轮ID供提交时重新分配。 */
static uint8_t AppArmSidePickPlacePrepare(App_Fruit_Side_e side)
{
    App_Arm_Place_Profile_s profile;
    float target_y_mm;
    float advance_y_mm;

    if (side == APP_FRUIT_SIDE_LEFT) {
        target_y_mm = APP_ARM_POSTURE_TEST_LEFT_Y_MM;
        advance_y_mm = APP_ARM_POSTURE_TEST_LEFT_ADVANCE_Y_MM;
    } else if (side == APP_FRUIT_SIDE_RIGHT) {
        target_y_mm = APP_ARM_POSTURE_TEST_RIGHT_Y_MM;
        advance_y_mm = APP_ARM_POSTURE_TEST_RIGHT_ADVANCE_Y_MM;
    } else {
        return 0u;
    }
    if (AppArmSidePickPlaceBuildPlaceProfile(side, &profile) == 0u) {
        return 0u;
    }

    app_arm_side_place_profile = profile;
    g_app_arm_posture_test_debug.active_side = (uint8_t)side;
    g_app_arm_posture_test_debug.place_profile_id = profile.profile_id;
    g_app_arm_posture_test_debug.target_center_mm[0] =
        APP_ARM_POSTURE_TEST_X_MM;
    g_app_arm_posture_test_debug.target_center_mm[1] = target_y_mm;
    g_app_arm_posture_test_debug.target_center_mm[2] =
        APP_ARM_POSTURE_TEST_Z_MM;
    g_app_arm_posture_test_debug.advance_center_mm[0] =
        APP_ARM_POSTURE_TEST_ADVANCE_X_MM;
    g_app_arm_posture_test_debug.advance_center_mm[1] = advance_y_mm;
    g_app_arm_posture_test_debug.advance_center_mm[2] =
        APP_ARM_POSTURE_TEST_ADVANCE_Z_MM;
    g_app_arm_posture_test_debug.target_tool_pitch_deg =
        APP_ARM_POSTURE_TEST_TOOL_PITCH_DEG;

    g_app_arm_posture_test_debug.base_command_id = 0u;
    g_app_arm_posture_test_debug.target_command_id = 0u;
    g_app_arm_posture_test_debug.advance_command_id = 0u;
    g_app_arm_posture_test_debug.active_command_id = 0u;
    g_app_arm_posture_test_debug.submit_result = 0u;
    g_app_arm_posture_test_debug.command_state = 0u;
    g_app_arm_posture_test_debug.fault_code = 0u;
    g_app_arm_posture_test_debug.place_start_result = 0u;
    app_arm_side_pitch_stable_tick = 0u;
    return 1u;
}

static Arm_Command_Result_e AppArmSidePickPlaceSubmitCenter(
    uint32_t command_id, const float center_mm[3], float max_speed_mm_s)
{
    Arm_Tool_Center_Command_s command;

    memset(&command, 0, sizeof(command));
    command.command_id = command_id;
    command.move_type = ARM_MOVE_LINEAR;
    command.target_center_mm.x_mm = center_mm[0];
    command.target_center_mm.y_mm = center_mm[1];
    command.target_center_mm.z_mm = center_mm[2];
    command.max_speed_mm_s = max_speed_mm_s;
    command.tool_pitch_valid = 1u;
    command.tool_pitch_deg =
        g_app_arm_posture_test_debug.target_tool_pitch_deg;
    return ArmSubmitToolCenterCommand(&command);
}

static uint8_t AppArmSidePickPlaceCommandFinished(
    const Arm_Host_Status_s *host, uint32_t command_id, uint32_t now_ms)
{
    if (host->pending_command_id == command_id) {
        g_app_arm_posture_test_debug.command_state =
            (uint32_t)ARM_COMMAND_STATE_QUEUED;
        return 0u;
    }
    if (host->active_command_id == command_id) {
        g_app_arm_posture_test_debug.command_state =
            (uint32_t)host->active_command_state;
        return 0u;
    }
    if (host->last_command_id != command_id) {
        return 0u;
    }
    g_app_arm_posture_test_debug.command_state =
        (uint32_t)host->last_command_state;
    g_app_arm_posture_test_debug.submit_result =
        (uint32_t)host->last_command_result;
    if (host->last_command_state == ARM_COMMAND_STATE_COMPLETED &&
        host->last_command_result == ARM_COMMAND_OK) {
        return 1u;
    }
    if (host->last_command_state == ARM_COMMAND_STATE_REJECTED ||
        host->last_command_state == ARM_COMMAND_STATE_CANCELLED ||
        host->last_command_state == ARM_COMMAND_STATE_FAULTED) {
        AppArmSidePickPlaceFail(now_ms);
    }
    return 0u;
}

static void AppArmSidePickPlaceUpdateWatch(const Arm_Host_Status_s *host,
                                           uint32_t now_ms)
{
    const Arm_State_s *arm = ArmGetState();
    const Arm_Tool_State_s *tool = ArmToolGetState();
    const Arm_Motion_Debug_s *motion = ArmGetMotionState();
    const float *active_center_mm =
        g_app_arm_posture_test_debug.state >=
            APP_ARM_POSTURE_TEST_SUBMIT_ADVANCE ?
        g_app_arm_posture_test_debug.advance_center_mm :
        g_app_arm_posture_test_debug.target_center_mm;
    uint8_t axis;

    g_app_arm_posture_test_debug.host_state = (uint32_t)host->state;
    g_app_arm_posture_test_debug.fault_code = host->fault_code;
    if (motion != NULL) {
        memcpy(g_app_arm_posture_test_debug.target_q_deg,
               motion->target_q_deg, sizeof(motion->target_q_deg));
    }
    for (axis = 0u; axis < 3u; ++axis) {
        g_app_arm_posture_test_debug.feedback_q_deg[axis] =
            host->q_feedback_deg[axis];
        g_app_arm_posture_test_debug.error_q_deg[axis] =
            g_app_arm_posture_test_debug.target_q_deg[axis] -
            host->q_feedback_deg[axis];
    }
    if (arm != NULL) {
        g_app_arm_posture_test_debug.feedback_center_mm[0] =
            arm->tool_tip.x_mm;
        g_app_arm_posture_test_debug.feedback_center_mm[1] =
            arm->tool_tip.y_mm;
        g_app_arm_posture_test_debug.feedback_center_mm[2] =
            arm->tool_tip.z_mm;
        for (axis = 0u; axis < 3u; ++axis) {
            g_app_arm_posture_test_debug.center_error_mm[axis] =
                active_center_mm[axis] -
                g_app_arm_posture_test_debug.feedback_center_mm[axis];
        }
    }
    g_app_arm_posture_test_debug.small_link_pitch_deg =
        ArmToolSmallLinkPitchFromJoint(host->q_feedback_deg);
    if (tool != NULL) {
        g_app_arm_posture_test_debug.gripper_state =
            (uint8_t)tool->gripper_state;
        g_app_arm_posture_test_debug.tool_pitch_feedback_deg =
            tool->tool_pitch_feedback_deg;
        g_app_arm_posture_test_debug.tool_pitch_error_deg =
            g_app_arm_posture_test_debug.target_tool_pitch_deg -
            tool->tool_pitch_feedback_deg;
    }
    g_app_arm_posture_test_debug.arm_flow_status =
        g_app_arm_pick_place_test_debug.flow_status;
    g_app_arm_posture_test_debug.arm_place_step =
        g_app_arm_pick_place_test_debug.place_step;
    g_app_arm_posture_test_debug.state_elapsed_ms =
        (uint32_t)(now_ms - app_arm_side_state_tick);
    g_app_arm_posture_test_debug.update_count++;
}

void AppArmSidePickPlaceInit(void)
{
    memset(&g_app_arm_posture_test_debug, 0,
           sizeof(g_app_arm_posture_test_debug));
    memset(&app_arm_side_place_profile, 0,
           sizeof(app_arm_side_place_profile));
    app_arm_side_pitch_stable_tick = 0u;
    app_arm_side_state_tick = 0u;
    g_app_arm_posture_test_debug.state = APP_ARM_POSTURE_TEST_WAIT_READY;
    g_app_arm_posture_test_debug.active_side = (uint8_t)APP_FRUIT_SIDE_NONE;
    AppArmSidePickPlaceSetStatus(APP_ARM_SIDE_PICK_PLACE_IDLE);
}

App_Arm_Side_Pick_Place_Start_Result_e AppArmSidePickPlaceStart(
    App_Fruit_Side_e side, uint32_t now_ms)
{
    App_Arm_Side_Pick_Place_Start_Result_e result;

    if (app_arm_side_status == APP_ARM_SIDE_PICK_PLACE_RUNNING) {
        result = APP_ARM_SIDE_PICK_PLACE_START_BUSY;
    } else if (side != APP_FRUIT_SIDE_LEFT &&
               side != APP_FRUIT_SIDE_RIGHT) {
        result = APP_ARM_SIDE_PICK_PLACE_START_INVALID_SIDE;
    } else if (app_arm_side_status == APP_ARM_SIDE_PICK_PLACE_FAILED) {
        result = APP_ARM_SIDE_PICK_PLACE_START_FAILED;
    } else if (AppArmSidePickPlacePrepare(side) == 0u) {
        AppArmSidePickPlaceFail(now_ms);
        result = APP_ARM_SIDE_PICK_PLACE_START_FAILED;
    } else {
        AppArmSidePickPlaceSetState(APP_ARM_POSTURE_TEST_WAIT_READY, now_ms);
        AppArmSidePickPlaceSetStatus(APP_ARM_SIDE_PICK_PLACE_RUNNING);
        g_app_arm_posture_test_debug.start_count++;
        result = APP_ARM_SIDE_PICK_PLACE_START_ACCEPTED;
    }
    g_app_arm_posture_test_debug.last_start_result = (uint8_t)result;
    return result;
}

App_Arm_Side_Pick_Place_Status_e AppArmSidePickPlacePoll(uint32_t now_ms)
{
    Arm_Host_Status_s host;
    Arm_Command_Result_e result;
    App_Arm_Flow_Status_e flow_status;

    if (app_arm_side_status != APP_ARM_SIDE_PICK_PLACE_RUNNING) {
        return app_arm_side_status;
    }
    if (ArmGetHostStatus(&host) == 0u) {
        return app_arm_side_status;
    }
    flow_status = AppArmFlowPoll(now_ms);
    AppArmSidePickPlaceUpdateWatch(&host, now_ms);

    if (host.state == ARM_HOST_STATE_FAULT ||
        host.state == ARM_HOST_STATE_ESTOP ||
        flow_status == APP_ARM_FLOW_FAILED) {
        AppArmSidePickPlaceFail(now_ms);
        return app_arm_side_status;
    }

    switch (g_app_arm_posture_test_debug.state) {
    case APP_ARM_POSTURE_TEST_WAIT_READY:
    {
        Arm_Joint_Command_s command;
        App_Arm_Pick_Staging_s staging;

        if (host.ready == 0u || host.busy != 0u) {
            break;
        }
        if (!AppArmFlowBuildPickStaging(
                g_app_arm_posture_test_debug.target_center_mm[0],
                g_app_arm_posture_test_debug.target_center_mm[1],
                &staging)) {
            g_app_arm_posture_test_debug.submit_result =
                (uint32_t)ARM_COMMAND_INVALID;
            AppArmSidePickPlaceFail(now_ms);
            break;
        }
        if (g_app_arm_posture_test_debug.base_command_id == 0u) {
            g_app_arm_posture_test_debug.base_command_id =
                AppArmCommandIdNext();
        }
        memset(&command, 0, sizeof(command));
        command.command_id = g_app_arm_posture_test_debug.base_command_id;
        command.move_type = ARM_MOVE_LINEAR;
        memcpy(command.q_deg, staging.q_deg, sizeof(command.q_deg));
        command.tool_relative_pitch_valid = 1u;
        command.tool_relative_pitch_deg = staging.tool_relative_pitch_deg;
        result = ArmSubmitJointCommand(&command);
        g_app_arm_posture_test_debug.active_command_id = command.command_id;
        g_app_arm_posture_test_debug.submit_result = (uint32_t)result;
        if (result == ARM_COMMAND_OK) {
            AppArmSidePickPlaceSetState(
                APP_ARM_POSTURE_TEST_WAIT_BASE_AIM, now_ms);
        } else if (result != ARM_COMMAND_BUSY &&
                   result != ARM_COMMAND_NOT_READY) {
            AppArmSidePickPlaceFail(now_ms);
        }
        break;
    }

    case APP_ARM_POSTURE_TEST_WAIT_BASE_AIM:
        if (AppArmSidePickPlaceCommandFinished(
                &host, g_app_arm_posture_test_debug.base_command_id,
                now_ms)) {
            AppArmSidePickPlaceSetState(
                APP_ARM_POSTURE_TEST_SUBMIT_TARGET, now_ms);
        }
        break;

    case APP_ARM_POSTURE_TEST_SUBMIT_TARGET:
        if (host.ready == 0u || host.busy != 0u) {
            break;
        }
        if (g_app_arm_posture_test_debug.target_command_id == 0u) {
            g_app_arm_posture_test_debug.target_command_id =
                AppArmCommandIdNext();
        }
        result = AppArmSidePickPlaceSubmitCenter(
            g_app_arm_posture_test_debug.target_command_id,
            g_app_arm_posture_test_debug.target_center_mm,
            APP_ARM_POSTURE_TEST_APPROACH_SPEED_MM_S);
        g_app_arm_posture_test_debug.active_command_id =
            g_app_arm_posture_test_debug.target_command_id;
        g_app_arm_posture_test_debug.submit_result = (uint32_t)result;
        if (result == ARM_COMMAND_OK) {
            AppArmSidePickPlaceSetState(
                APP_ARM_POSTURE_TEST_WAIT_TARGET, now_ms);
        } else if (result != ARM_COMMAND_BUSY &&
                   result != ARM_COMMAND_NOT_READY) {
            AppArmSidePickPlaceFail(now_ms);
        }
        break;

    case APP_ARM_POSTURE_TEST_WAIT_TARGET:
        if (AppArmSidePickPlaceCommandFinished(
                &host, g_app_arm_posture_test_debug.target_command_id,
                now_ms)) {
            AppArmSidePickPlaceSetState(
                APP_ARM_POSTURE_TEST_SUBMIT_ADVANCE, now_ms);
        }
        break;

    case APP_ARM_POSTURE_TEST_SUBMIT_ADVANCE:
        if (host.ready == 0u || host.busy != 0u) {
            break;
        }
        if (g_app_arm_posture_test_debug.advance_command_id == 0u) {
            g_app_arm_posture_test_debug.advance_command_id =
                AppArmCommandIdNext();
        }
        result = AppArmSidePickPlaceSubmitCenter(
            g_app_arm_posture_test_debug.advance_command_id,
            g_app_arm_posture_test_debug.advance_center_mm,
            APP_ARM_POSTURE_TEST_GRIP_ADVANCE_SPEED_MM_S);
        g_app_arm_posture_test_debug.active_command_id =
            g_app_arm_posture_test_debug.advance_command_id;
        g_app_arm_posture_test_debug.submit_result = (uint32_t)result;
        if (result == ARM_COMMAND_OK) {
            AppArmSidePickPlaceSetState(
                APP_ARM_POSTURE_TEST_WAIT_ADVANCE, now_ms);
        } else if (result != ARM_COMMAND_BUSY &&
                   result != ARM_COMMAND_NOT_READY) {
            AppArmSidePickPlaceFail(now_ms);
        }
        break;

    case APP_ARM_POSTURE_TEST_WAIT_ADVANCE:
        if (AppArmSidePickPlaceCommandFinished(
                &host, g_app_arm_posture_test_debug.advance_command_id,
                now_ms)) {
            app_arm_side_pitch_stable_tick = 0u;
            AppArmSidePickPlaceSetState(
                APP_ARM_POSTURE_TEST_WAIT_PITCH_STABLE, now_ms);
        }
        break;

    case APP_ARM_POSTURE_TEST_WAIT_PITCH_STABLE:
    {
        const Arm_Tool_State_s *tool = ArmToolGetState();

        if (tool == NULL || tool->servo_feedback_valid[0] == 0u ||
            !isfinite(tool->tool_pitch_feedback_deg) ||
            fabsf(tool->tool_pitch_feedback_deg -
                   g_app_arm_posture_test_debug.target_tool_pitch_deg) >
                APP_ARM_TOOL_CENTER_PITCH_TOLERANCE_DEG) {
            app_arm_side_pitch_stable_tick = 0u;
            break;
        }
        if (app_arm_side_pitch_stable_tick == 0u) {
            app_arm_side_pitch_stable_tick = now_ms;
        }
        if ((uint32_t)(now_ms - app_arm_side_pitch_stable_tick) >=
            APP_ARM_TOOL_CENTER_PITCH_STABLE_MS) {
            AppArmSidePickPlaceSetState(
                APP_ARM_POSTURE_TEST_PICK_DWELL, now_ms);
        }
        break;
    }

    case APP_ARM_POSTURE_TEST_PICK_DWELL:
        if ((uint32_t)(now_ms - app_arm_side_state_tick) >=
            APP_ARM_PICK_DWELL_MS) {
            AppArmSidePickPlaceSetState(
                APP_ARM_POSTURE_TEST_SUBMIT_CLOSE, now_ms);
        }
        break;

    case APP_ARM_POSTURE_TEST_SUBMIT_CLOSE:
        result = ArmToolSetGripper(ARM_GRIPPER_COMMAND_CLOSE);
        g_app_arm_posture_test_debug.submit_result = (uint32_t)result;
        if (result == ARM_COMMAND_OK) {
            AppArmSidePickPlaceSetState(
                APP_ARM_POSTURE_TEST_WAIT_CLOSE, now_ms);
        } else if (result != ARM_COMMAND_BUSY) {
            AppArmSidePickPlaceFail(now_ms);
        }
        break;

    case APP_ARM_POSTURE_TEST_WAIT_CLOSE:
    {
        const Arm_Tool_State_s *tool = ArmToolGetState();

        if (tool == NULL || ArmToolGripperFaulted() != 0u) {
            AppArmSidePickPlaceFail(now_ms);
        } else if (tool->gripper_state == ARM_GRIPPER_HELD_CONTACT ||
                   tool->gripper_state == ARM_GRIPPER_CLOSED_EMPTY ||
                   tool->gripper_state == ARM_GRIPPER_FORCED_HELD) {
            AppArmSidePickPlaceSetState(
                APP_ARM_POSTURE_TEST_POST_GRIP_DWELL, now_ms);
        }
        break;
    }

    case APP_ARM_POSTURE_TEST_POST_GRIP_DWELL:
        if ((uint32_t)(now_ms - app_arm_side_state_tick) >=
            APP_ARM_POST_GRIP_DWELL_MS) {
            AppArmSidePickPlaceSetState(
                APP_ARM_POSTURE_TEST_START_PLACE, now_ms);
        }
        break;

    case APP_ARM_POSTURE_TEST_START_PLACE:
    {
        App_Arm_Flow_Start_Result_e start_result =
            AppArmFlowStartPlace(&app_arm_side_place_profile, now_ms);

        g_app_arm_posture_test_debug.place_start_result =
            (uint32_t)start_result;
        if (start_result == APP_ARM_FLOW_START_ACCEPTED) {
            AppArmSidePickPlaceSetState(
                APP_ARM_POSTURE_TEST_WAIT_PLACE, now_ms);
        } else if (start_result != APP_ARM_FLOW_START_BUSY) {
            AppArmSidePickPlaceFail(now_ms);
        }
        break;
    }

    case APP_ARM_POSTURE_TEST_WAIT_PLACE:
        if (flow_status == APP_ARM_FLOW_DONE) {
            App_Fruit_Side_e completed_side =
                (App_Fruit_Side_e)g_app_arm_posture_test_debug.active_side;

            if (completed_side == APP_FRUIT_SIDE_LEFT) {
                g_app_arm_posture_test_debug.left_completed_count++;
            } else if (completed_side == APP_FRUIT_SIDE_RIGHT) {
                g_app_arm_posture_test_debug.right_completed_count++;
            } else {
                AppArmSidePickPlaceFail(now_ms);
                break;
            }
            g_app_arm_pick_place_test_debug.cycle_count++;
            g_app_arm_posture_test_debug.completed_count++;
            AppArmSidePickPlaceSetState(APP_ARM_POSTURE_TEST_DONE, now_ms);
            AppArmSidePickPlaceSetStatus(APP_ARM_SIDE_PICK_PLACE_DONE);
        }
        break;

    case APP_ARM_POSTURE_TEST_DONE:
    case APP_ARM_POSTURE_TEST_FAILED:
    default:
        break;
    }
    return app_arm_side_status;
}

App_Arm_Side_Pick_Place_Status_e AppArmSidePickPlaceGetStatus(void)
{
    return app_arm_side_status;
}

void AppArmSidePickPlaceAbort(uint32_t now_ms)
{
    AppArmFlowAbort(now_ms);
    app_arm_side_pitch_stable_tick = 0u;
    app_arm_side_state_tick = now_ms;
    g_app_arm_posture_test_debug.base_command_id = 0u;
    g_app_arm_posture_test_debug.target_command_id = 0u;
    g_app_arm_posture_test_debug.advance_command_id = 0u;
    g_app_arm_posture_test_debug.active_command_id = 0u;
    g_app_arm_posture_test_debug.submit_result = 0u;
    g_app_arm_posture_test_debug.command_state =
        (uint32_t)ARM_COMMAND_STATE_NONE;
    g_app_arm_posture_test_debug.place_start_result = 0u;
    g_app_arm_posture_test_debug.active_side =
        (uint8_t)APP_FRUIT_SIDE_NONE;
    AppArmSidePickPlaceSetState(APP_ARM_POSTURE_TEST_WAIT_READY, now_ms);
    AppArmSidePickPlaceSetStatus(APP_ARM_SIDE_PICK_PLACE_IDLE);
}

#else

void AppArmSidePickPlaceInit(void)
{
    memset(&g_app_arm_posture_test_debug, 0,
           sizeof(g_app_arm_posture_test_debug));
    g_app_arm_posture_test_debug.operation_status =
        (uint8_t)APP_ARM_SIDE_PICK_PLACE_IDLE;
}

uint8_t AppArmSidePickPlaceBuildPlaceProfile(
    App_Fruit_Side_e side, App_Arm_Place_Profile_s *profile)
{
    (void)side;
    if (profile != NULL) {
        memset(profile, 0, sizeof(*profile));
    }
    return 0u;
}

App_Arm_Side_Pick_Place_Start_Result_e AppArmSidePickPlaceStart(
    App_Fruit_Side_e side, uint32_t now_ms)
{
    (void)side;
    (void)now_ms;
    g_app_arm_posture_test_debug.last_start_result =
        (uint8_t)APP_ARM_SIDE_PICK_PLACE_START_FAILED;
    return APP_ARM_SIDE_PICK_PLACE_START_FAILED;
}

App_Arm_Side_Pick_Place_Status_e AppArmSidePickPlacePoll(uint32_t now_ms)
{
    (void)now_ms;
    return APP_ARM_SIDE_PICK_PLACE_IDLE;
}

App_Arm_Side_Pick_Place_Status_e AppArmSidePickPlaceGetStatus(void)
{
    return APP_ARM_SIDE_PICK_PLACE_IDLE;
}

void AppArmSidePickPlaceAbort(uint32_t now_ms)
{
    (void)now_ms;
    g_app_arm_posture_test_debug.operation_status =
        (uint8_t)APP_ARM_SIDE_PICK_PLACE_IDLE;
}

#endif
