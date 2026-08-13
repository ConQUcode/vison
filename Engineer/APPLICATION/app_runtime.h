/**
 * @file app_runtime.h
 * @brief 整机初始化和四个 FreeRTOS 周期任务的应用层入口。
 */

#ifndef APP_RUNTIME_H
#define APP_RUNTIME_H

#include <stdint.h>

#include "app_config.h"
/*
 * 抓放子流程的状态枚举、Watch结构和 g_app_arm_pick_place_test_debug
 * 已迁移到 app_arm_flow.h/.c；本文件只保留调度入口和打点模式Watch。
 */
#include "app_arm_flow.h"

/** 无力打点模式的紧凑Watch快照；角度deg，坐标mm。 */
typedef struct {
    uint8_t feedback_ready;
    uint8_t dm_all_disabled;
    uint8_t dm_online[3];       /* 底座、大臂、小臂。 */
    uint8_t dm_enabled[3];      /* 打点模式下应始终为0。 */
    float dm_joint_deg[3];
    uint32_t dm_feedback_age_ms[3];
    uint8_t servo_online[2];    /* ID1俯仰、ID2夹爪。 */
    uint16_t servo_position[2]; /* 控制值0..1000。 */
    float servo_angle_deg[2];
    uint8_t servo_unload_requested; /* 已提交ID1/ID2无力矩命令。 */
    uint8_t servo_unload_done;      /* 无力矩帧已发完且位置轮询已启动。 */
    uint32_t servo_unload_count;    /* 成功提交次数，正常上电应为1。 */
    uint32_t servo_unload_fail_count; /* 发送或启动反馈轮询失败次数。 */
    float tool_pitch_deg;       /* ID1与小臂合成的世界绝对俯仰。 */
    uint8_t wrist_center_valid;
    float wrist_center_mm[3];   /* ID1输出轴中心。 */
    uint8_t tool_center_valid;
    float tool_center_mm[3];    /* 按117mm偏移得到的夹爪中心。 */
    uint32_t update_count;
} App_Arm_Teach_Debug_s;

extern App_Arm_Teach_Debug_s g_app_arm_teach_debug;

void AppInit(void);
void AppImuTask(uint32_t now_ms);
void AppChassisTask(uint32_t now_ms);
void AppUsbTask(uint32_t now_ms);
void AppArmTask(uint32_t now_ms);
/** 高优先级1 kHz电机发送入口；达妙和DJI周期控制的唯一任务调用点。 */
void AppMotorControlTask(uint32_t now_ms);

#endif
