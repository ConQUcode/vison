/**
 * @file app_arm_command_id.h
 * @brief 应用层机械臂命令ID的唯一分配入口。
 */

#ifndef APP_ARM_COMMAND_ID_H
#define APP_ARM_COMMAND_ID_H

#include <stdint.h>

/** 统一命令ID分配器Watch；底盘使用独立命令域，不包含在这里。 */
typedef struct {
    uint8_t initialized;
    uint32_t seed_id;
    uint32_t last_issued_id;
    uint32_t issue_count;
    uint32_t init_call_count;
    uint32_t wrap_count;
} App_Arm_Command_Id_Debug_s;

extern App_Arm_Command_Id_Debug_s g_app_arm_command_id_debug;

/** 幂等初始化；重复调用不会把已发放序列退回种子。 */
void AppArmCommandIdInit(void);

/** 原子发放下一个非零命令ID；所有固件内部机械臂调用者必须使用本接口。 */
uint32_t AppArmCommandIdNext(void);

#endif
