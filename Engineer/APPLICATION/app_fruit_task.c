/**
 * @file app_fruit_task.c
 * @brief 解释静态 A 区任务表并串联底盘、抓取和显式放置 profile。
 */

#include "app_fruit_task.h"
#include "app_config.h"
#include "app_fruit_task_config.h"

#include <string.h>

/* 非水果模式也保留Watch符号，任务实现只在完整机械臂模式参与链接。 */
App_Fruit_Task_Debug_s g_app_fruit_task_debug;

#define APP_FRUIT_PLACE_PROFILE_A_LEFT  0xA001u
#define APP_FRUIT_PLACE_PROFILE_A_RIGHT 0xA002u

typedef struct {
    App_Fruit_Point_Id_e point_id;
    App_Fruit_Area_e area;
    App_Fruit_Side_e side;
    App_Arm_Pick_Target_s pick_target;
} App_Fruit_Point_s;

typedef struct {
    float move_before_pick_mm;
    const App_Fruit_Point_s *point;
    const App_Arm_Place_Profile_s *place_profile;
} App_Fruit_Task_Item_s;

static const App_Arm_Place_Profile_s app_fruit_place_a_left = {
    APP_FRUIT_PLACE_PROFILE_A_LEFT,
    1u,
    0u, /* 公共A左profile直接过渡，不携带AC专用waypoint。 */
    0u, /* 公共A左profile不启用AC专用Y/Z约束。 */
    { APP_FRUIT_A_LEFT_SAFE_Q1_DEG, APP_FRUIT_A_TRANSFER_Q2_DEG,
      APP_FRUIT_A_TRANSFER_Q3_DEG },
    0.0f,
    0.0f,
    0.0f,
    { APP_FRUIT_A_LEFT_SAFE_Q1_DEG, APP_FRUIT_A_TRANSFER_Q2_DEG,
      APP_FRUIT_A_TRANSFER_Q3_DEG },
    APP_FRUIT_A_LEFT_PLACE_WAYPOINT_Q1_DEG,
    APP_FRUIT_A_LEFT_PLACE_Q1_DEG,
    { APP_FRUIT_A_LEFT_PLACE_Q1_DEG, APP_FRUIT_A_RELEASE_Q2_DEG,
      APP_FRUIT_A_RELEASE_Q3_DEG },
    APP_FRUIT_A_RELEASE_TOOL_RELATIVE_PITCH_DEG,
    APP_FRUIT_A_RELEASE_PITCH_WAIT_TIMEOUT_MS,
    { APP_FRUIT_A_LEFT_PLACE_Q1_DEG, APP_FRUIT_A_RELEASE_Q2_DEG,
      APP_FRUIT_A_RELEASE_CLEARANCE_Q3_DEG },
    APP_FRUIT_A_LEFT_FRONT_WAYPOINT_Q1_DEG,
    APP_FRUIT_A_LEFT_FRONT_Q1_DEG,
};

static const App_Arm_Place_Profile_s app_fruit_place_a_right = {
    APP_FRUIT_PLACE_PROFILE_A_RIGHT,
    1u,
    0u, /* 公共A右profile直接过渡，不携带AC专用waypoint。 */
    0u, /* 公共A右profile不启用AC专用Y/Z约束。 */
    { APP_FRUIT_A_RIGHT_SAFE_Q1_DEG, APP_FRUIT_A_TRANSFER_Q2_DEG,
      APP_FRUIT_A_TRANSFER_Q3_DEG },
    0.0f,
    0.0f,
    0.0f,
    { APP_FRUIT_A_RIGHT_SAFE_Q1_DEG, APP_FRUIT_A_TRANSFER_Q2_DEG,
      APP_FRUIT_A_TRANSFER_Q3_DEG },
    APP_FRUIT_A_RIGHT_PLACE_WAYPOINT_Q1_DEG,
    APP_FRUIT_A_RIGHT_PLACE_Q1_DEG,
    { APP_FRUIT_A_RIGHT_PLACE_Q1_DEG, APP_FRUIT_A_RELEASE_Q2_DEG,
      APP_FRUIT_A_RELEASE_Q3_DEG },
    APP_FRUIT_A_RELEASE_TOOL_RELATIVE_PITCH_DEG,
    APP_FRUIT_A_RELEASE_PITCH_WAIT_TIMEOUT_MS,
    { APP_FRUIT_A_RIGHT_PLACE_Q1_DEG, APP_FRUIT_A_RELEASE_Q2_DEG,
      APP_FRUIT_A_RELEASE_CLEARANCE_Q3_DEG },
    APP_FRUIT_A_RIGHT_FRONT_WAYPOINT_Q1_DEG,
    APP_FRUIT_A_RIGHT_FRONT_Q1_DEG,
};

uint8_t AppFruitGetPlaceProfile(App_Fruit_Area_e area,
                                App_Fruit_Side_e side,
                                App_Arm_Place_Profile_s *profile)
{
    const App_Arm_Place_Profile_s *source = NULL;

    if (profile == NULL) {
        return 0u;
    }
    memset(profile, 0, sizeof(*profile));
    if (area != APP_FRUIT_AREA_A) {
        return 0u;
    }
    if (side == APP_FRUIT_SIDE_LEFT) {
        source = &app_fruit_place_a_left;
    } else if (side == APP_FRUIT_SIDE_RIGHT) {
        source = &app_fruit_place_a_right;
    }
    if (source == NULL || source->configured == 0u) {
        return 0u;
    }
    *profile = *source;
    return 1u;
}

#if APP_ARM_ENABLED && APP_ARM_TOOL_CENTER_TEST_ENABLE

#include "arm.h"
#include "chassis.h"

static uint32_t app_fruit_next_chassis_command_id;

static const App_Fruit_Point_s app_fruit_point_a_left = {
    APP_FRUIT_POINT_A_LEFT,
    APP_FRUIT_AREA_A,
    APP_FRUIT_SIDE_LEFT,
    {
        APP_FRUIT_A_LEFT_X_MM,
        APP_FRUIT_A_LEFT_Y_MM,
        APP_FRUIT_A_LEFT_Z_MM,
        APP_FRUIT_A_PICK_TOOL_PITCH_DEG,
    },
};

static const App_Fruit_Point_s app_fruit_point_a_right = {
    APP_FRUIT_POINT_A_RIGHT,
    APP_FRUIT_AREA_A,
    APP_FRUIT_SIDE_RIGHT,
    {
        APP_FRUIT_A_RIGHT_X_MM,
        APP_FRUIT_A_RIGHT_Y_MM,
        APP_FRUIT_A_RIGHT_Z_MM,
        APP_FRUIT_A_PICK_TOOL_PITCH_DEG,
    },
};

/*
 * 原地模式仅用于机械臂硬件联调，不代表A区完整场地路线。
 * 关闭测试开关后恢复585/点1、500/点2、500/点1并最终DONE。
 */
static const App_Fruit_Task_Item_s app_fruit_test_tasks[] = {
#if APP_FRUIT_TEST_IN_PLACE_ALTERNATING_ENABLE
    { 0.0f, &app_fruit_point_a_left, &app_fruit_place_a_left },
    { 0.0f, &app_fruit_point_a_right, &app_fruit_place_a_right },
#else
    { APP_FRUIT_AREA_A_FIRST_MOVE_MM,
      &app_fruit_point_a_left, &app_fruit_place_a_left },
    { APP_FRUIT_AREA_A_GROUP_SPACING_MM,
      &app_fruit_point_a_right, &app_fruit_place_a_right },
    { APP_FRUIT_AREA_A_GROUP_SPACING_MM,
      &app_fruit_point_a_left, &app_fruit_place_a_left },
#endif
};

static const App_Fruit_Task_Item_s *AppFruitCurrentTask(void)
{
    if (g_app_fruit_task_debug.task_index >=
        (uint8_t)(sizeof(app_fruit_test_tasks) /
                  sizeof(app_fruit_test_tasks[0]))) {
        return NULL;
    }
    return &app_fruit_test_tasks[g_app_fruit_task_debug.task_index];
}

static void AppFruitFail(App_Fruit_Failure_Source_e source, uint32_t code)
{
    if (g_app_fruit_task_debug.state == APP_FRUIT_TASK_FAILED) {
        return;
    }
    g_app_fruit_task_debug.failure_source = source;
    g_app_fruit_task_debug.failure_code = code;
    g_app_fruit_task_debug.state = APP_FRUIT_TASK_FAILED;
}

static void AppFruitRefreshWatch(const Chassis_Status_s *chassis)
{
    const App_Fruit_Task_Item_s *task = AppFruitCurrentTask();

    if (task != NULL && task->point != NULL &&
        task->place_profile != NULL) {
        g_app_fruit_task_debug.area = task->point->area;
        g_app_fruit_task_debug.side = task->point->side;
        g_app_fruit_task_debug.point_id = task->point->point_id;
        g_app_fruit_task_debug.place_profile_id =
            task->place_profile->profile_id;
    }
    if (chassis != NULL) {
        if (task != NULL && task->move_before_pick_mm == 0.0f) {
            g_app_fruit_task_debug.chassis_command_id = 0u;
            g_app_fruit_task_debug.chassis_target_distance_mm = 0.0f;
            g_app_fruit_task_debug.chassis_actual_distance_mm = 0.0f;
        } else {
            g_app_fruit_task_debug.chassis_actual_distance_mm =
                chassis->actual_distance_mm;
        }
    }
    g_app_fruit_task_debug.arm_active_flow =
        g_app_arm_pick_place_test_debug.active_flow;
    g_app_fruit_task_debug.arm_pick_step =
        g_app_arm_pick_place_test_debug.pick_step;
    g_app_fruit_task_debug.arm_place_step =
        g_app_arm_pick_place_test_debug.place_step;
}

static uint32_t AppFruitNextChassisCommandId(void)
{
    app_fruit_next_chassis_command_id++;
    if (app_fruit_next_chassis_command_id == 0u) {
        app_fruit_next_chassis_command_id =
            APP_FRUIT_CHASSIS_COMMAND_ID_BASE + 1u;
    }
    return app_fruit_next_chassis_command_id;
}

void AppFruitTaskInit(void)
{
    memset(&g_app_fruit_task_debug, 0, sizeof(g_app_fruit_task_debug));
    g_app_fruit_task_debug.state = APP_FRUIT_TASK_WAIT_READY;
    g_app_fruit_task_debug.task_count =
        (uint8_t)(sizeof(app_fruit_test_tasks) /
                  sizeof(app_fruit_test_tasks[0]));
    app_fruit_next_chassis_command_id =
        APP_FRUIT_CHASSIS_COMMAND_ID_BASE;
}

void AppFruitTask(uint32_t now_ms)
{
    const App_Fruit_Task_Item_s *task = AppFruitCurrentTask();
    Chassis_Status_s chassis;
    App_Arm_Flow_Status_e arm_status = AppArmFlowPoll(now_ms);
    Arm_Host_Status_s arm_host;

    if (ChassisGetStatus(&chassis) == 0u) {
        AppFruitFail(APP_FRUIT_FAILURE_CHASSIS,
                     (uint32_t)CHASSIS_FAULT_INIT);
        return;
    }
    AppFruitRefreshWatch(&chassis);
    if (g_app_fruit_task_debug.state == APP_FRUIT_TASK_FAILED ||
        g_app_fruit_task_debug.state == APP_FRUIT_TASK_DONE) {
        return;
    }
    if (chassis.state == CHASSIS_STATE_FAULT) {
        AppFruitFail(APP_FRUIT_FAILURE_CHASSIS, (uint32_t)chassis.fault);
        return;
    }
    if (chassis.state == CHASSIS_STATE_CANCELLED) {
        AppFruitFail(APP_FRUIT_FAILURE_CHASSIS,
                     (uint32_t)CHASSIS_STATE_CANCELLED);
        return;
    }
    if (arm_status == APP_ARM_FLOW_FAILED) {
        AppFruitFail(APP_FRUIT_FAILURE_ARM,
            g_app_arm_pick_place_test_debug.fault);
        return;
    }
    if (task == NULL || task->point == NULL ||
        task->place_profile == NULL ||
        task->point->area != APP_FRUIT_AREA_A ||
        task->place_profile->configured == 0u) {
        AppFruitFail(APP_FRUIT_FAILURE_NOT_CONFIGURED, 0u);
        return;
    }

    switch (g_app_fruit_task_debug.state) {
    case APP_FRUIT_TASK_WAIT_READY:
        if ((chassis.state == CHASSIS_STATE_IDLE ||
             chassis.state == CHASSIS_STATE_COMPLETED) &&
            ArmGetHostStatus(&arm_host) != 0u && arm_host.ready != 0u) {
            Chassis_Command_s command;
            Chassis_Command_Result_e result;

            if (task->move_before_pick_mm == 0.0f) {
                if (AppArmFlowStartPick(&task->point->pick_target,
                                        now_ms) != 0u) {
                    g_app_fruit_task_debug.state = APP_FRUIT_TASK_PICK;
                } else {
                    AppFruitFail(APP_FRUIT_FAILURE_ARM_START,
                                 (uint32_t)APP_ARM_FLOW_START_BUSY);
                }
                break;
            }

            memset(&command, 0, sizeof(command));
            command.command_id = AppFruitNextChassisCommandId();
            command.type = CHASSIS_COMMAND_RELATIVE_STRAIGHT;
            command.distance_mm = task->move_before_pick_mm;
            command.tolerance_mm = APP_FRUIT_CHASSIS_TOLERANCE_MM;
            command.heading_mode = CHASSIS_HEADING_HOLD_START;
            result = ChassisSubmitCommand(&command);
            if (result == CHASSIS_COMMAND_ACCEPTED) {
                g_app_fruit_task_debug.chassis_command_id =
                    command.command_id;
                g_app_fruit_task_debug.chassis_target_distance_mm =
                    command.distance_mm;
                g_app_fruit_task_debug.state =
                    APP_FRUIT_TASK_WAIT_CHASSIS;
            } else {
                AppFruitFail(APP_FRUIT_FAILURE_CHASSIS_SUBMIT,
                             (uint32_t)result);
            }
        }
        break;

    case APP_FRUIT_TASK_WAIT_CHASSIS:
        if (chassis.command_id !=
            g_app_fruit_task_debug.chassis_command_id) {
            AppFruitFail(APP_FRUIT_FAILURE_CHASSIS,
                         (uint32_t)CHASSIS_COMMAND_DUPLICATE);
            break;
        }
        if (chassis.state == CHASSIS_STATE_CANCELLED) {
            AppFruitFail(APP_FRUIT_FAILURE_CHASSIS,
                         (uint32_t)CHASSIS_STATE_CANCELLED);
            break;
        }
        if (chassis.state != CHASSIS_STATE_COMPLETED ||
            ArmGetHostStatus(&arm_host) == 0u || arm_host.ready == 0u) {
            break;
        }
        if (AppArmFlowStartPick(&task->point->pick_target, now_ms) != 0u) {
            g_app_fruit_task_debug.state = APP_FRUIT_TASK_PICK;
        } else {
            AppFruitFail(APP_FRUIT_FAILURE_ARM_START,
                         (uint32_t)APP_ARM_FLOW_START_BUSY);
        }
        break;

    case APP_FRUIT_TASK_PICK:
        if (arm_status == APP_ARM_FLOW_DONE) {
            App_Arm_Flow_Start_Result_e start_result =
                AppArmFlowStartPlace(task->place_profile, now_ms);
            if (start_result == APP_ARM_FLOW_START_ACCEPTED) {
                g_app_fruit_task_debug.state = APP_FRUIT_TASK_PLACE;
            } else {
                AppFruitFail(APP_FRUIT_FAILURE_ARM_START,
                             (uint32_t)start_result);
            }
        }
        break;

    case APP_FRUIT_TASK_PLACE:
        if (arm_status == APP_ARM_FLOW_DONE) {
            g_app_fruit_task_debug.completed_count++;
            g_app_arm_pick_place_test_debug.cycle_count++;
            g_app_fruit_task_debug.task_index++;
            if (g_app_fruit_task_debug.task_index >=
                g_app_fruit_task_debug.task_count) {
#if APP_FRUIT_TEST_IN_PLACE_ALTERNATING_ENABLE
                g_app_fruit_task_debug.task_index = 0u;
                g_app_fruit_task_debug.state = APP_FRUIT_TASK_WAIT_READY;
#else
                g_app_fruit_task_debug.state = APP_FRUIT_TASK_DONE;
                g_app_fruit_task_debug.area = APP_FRUIT_AREA_UNCONFIGURED;
                g_app_fruit_task_debug.side = APP_FRUIT_SIDE_NONE;
                g_app_fruit_task_debug.point_id = APP_FRUIT_POINT_NONE;
#endif
            } else {
                g_app_fruit_task_debug.state = APP_FRUIT_TASK_WAIT_READY;
            }
        }
        break;

    case APP_FRUIT_TASK_DONE:
    case APP_FRUIT_TASK_FAILED:
    default:
        break;
    }
}

#endif /* APP_ARM_ENABLED && APP_ARM_TOOL_CENTER_TEST_ENABLE */
