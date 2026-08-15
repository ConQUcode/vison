/**
 * @file app_arm_command_id.c
 * @brief 在所有应用机械臂流程之间维护单一、单调递增的命令ID序列。
 */

#include "app_arm_command_id.h"

#include "app_config.h"
#include "main.h"

#if APP_ARM_COMMAND_ID_SEED == 0u
#error "APP_ARM_COMMAND_ID_SEED must be non-zero"
#endif

App_Arm_Command_Id_Debug_s g_app_arm_command_id_debug;

static uint32_t app_arm_last_command_id;

void AppArmCommandIdInit(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_app_arm_command_id_debug.init_call_count++;
    if (g_app_arm_command_id_debug.initialized == 0u) {
        app_arm_last_command_id = APP_ARM_COMMAND_ID_SEED;
        g_app_arm_command_id_debug.seed_id = APP_ARM_COMMAND_ID_SEED;
        g_app_arm_command_id_debug.last_issued_id =
            APP_ARM_COMMAND_ID_SEED;
        g_app_arm_command_id_debug.issue_count = 0u;
        g_app_arm_command_id_debug.wrap_count = 0u;
        g_app_arm_command_id_debug.initialized = 1u;
    }
    if (primask == 0u) {
        __enable_irq();
    }
}

uint32_t AppArmCommandIdNext(void)
{
    uint32_t primask;
    uint32_t command_id;

    if (g_app_arm_command_id_debug.initialized == 0u) {
        AppArmCommandIdInit();
    }

    primask = __get_PRIMASK();
    __disable_irq();
    command_id = app_arm_last_command_id + 1u;
    if (command_id == 0u) {
        command_id = 1u;
        g_app_arm_command_id_debug.wrap_count++;
    }
    app_arm_last_command_id = command_id;
    g_app_arm_command_id_debug.last_issued_id = command_id;
    g_app_arm_command_id_debug.issue_count++;
    if (primask == 0u) {
        __enable_irq();
    }
    return command_id;
}
