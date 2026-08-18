/**
 * @file app_runtime.h
 * @brief 整机初始化和四个 FreeRTOS 周期任务的应用层入口。
 */

#ifndef APP_RUNTIME_H
#define APP_RUNTIME_H

#include <stdint.h>

#include "app_config.h"
/*
 * 抓放子流程与单侧完整抓放接口分别由独立模块维护；本文件只保留
 * 整机调度入口和无力打点Watch。
 */
#include "app_arm_flow.h"
#include "app_arm_side_pick_place.h"

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

typedef enum {
    APP_ARM_BD_OBSERVATION_WAIT_READY = 0,
    APP_ARM_BD_OBSERVATION_BASE_SUBMITTED,
    APP_ARM_BD_OBSERVATION_SUBMIT_TARGET,
    APP_ARM_BD_OBSERVATION_TARGET_SUBMITTED,
    APP_ARM_BD_OBSERVATION_HOLDING,
    APP_ARM_BD_OBSERVATION_FAILED
} App_Arm_Bd_Observation_State_e;

/** BD区左侧树上水果观察位单次测试Watch；角度deg，坐标mm。 */
typedef struct {
    uint8_t state;
    uint8_t host_ready;
    uint8_t host_busy;
    uint8_t path_preflight_passed;
    uint32_t command_id;
    uint32_t base_command_id;
    uint32_t target_command_id;
    uint32_t submit_result;
    uint32_t command_state;
    uint32_t fault_code;
    uint32_t state_tick_ms;
    float base_target_q_deg[3];
    float base_tool_relative_pitch_deg;
    float target_center_mm[3];
    float target_tool_pitch_deg;
    float target_speed_mm_s;
    float actual_center_mm[3];
    float actual_tool_pitch_deg;
    float actual_q_deg[3];
    float center_error_mm;
    float pitch_error_deg;
    float trajectory_progress;
} App_Arm_Bd_Observation_Debug_s;

extern App_Arm_Bd_Observation_Debug_s
    g_app_arm_bd_observation_debug;

typedef enum {
    APP_ARM_CLEARANCE_WAIT_READY = 0,
    APP_ARM_CLEARANCE_SUBMITTED,
    APP_ARM_CLEARANCE_HOLDING,
    APP_ARM_CLEARANCE_FAILED
} App_Arm_Clearance_Test_State_e;

/** 底盘转弯避让姿态单次测试Watch；角度deg，坐标mm。 */
typedef struct {
    uint8_t state;
    uint8_t host_ready;
    uint8_t host_busy;
    uint8_t path_preflight_passed;
    uint32_t command_id;
    uint32_t submit_result;
    uint32_t command_state;
    uint32_t command_result;
    uint32_t fault_code;
    uint32_t state_tick_ms;
    float target_q_deg[3];
    float target_tool_relative_pitch_deg;
    float target_tool_pitch_deg;
    float actual_q_deg[3];
    float actual_center_mm[3];
    float actual_tool_pitch_deg;
    float q_error_deg[3];
    float trajectory_progress;
} App_Arm_Clearance_Test_Debug_s;

extern App_Arm_Clearance_Test_Debug_s
    g_app_arm_clearance_test_debug;

typedef enum {
    APP_ARM_QR_POSE_WAIT_READY = 0,
    APP_ARM_QR_POSE_SUBMITTED,
    APP_ARM_QR_POSE_HOLDING,
    APP_ARM_QR_POSE_FAILED
} App_Arm_Qr_Pose_Test_State_e;

/** 二维码识别姿态单次测试Watch；角度deg，坐标mm。 */
typedef struct {
    uint8_t state;
    uint8_t host_ready;
    uint8_t host_busy;
    uint8_t path_preflight_passed;
    uint32_t command_id;
    uint32_t submit_result;
    uint32_t command_state;
    uint32_t command_result;
    uint32_t fault_code;
    uint32_t state_tick_ms;
    float target_q_deg[3];
    float target_tool_relative_pitch_deg;
    float target_tool_pitch_deg;
    float target_center_mm[3];
    float actual_q_deg[3];
    float actual_center_mm[3];
    float actual_tool_pitch_deg;
    float q_error_deg[3];
    float center_error_mm;
    float pitch_error_deg;
    float trajectory_progress;
} App_Arm_Qr_Pose_Test_Debug_s;

extern App_Arm_Qr_Pose_Test_Debug_s g_app_arm_qr_pose_test_debug;

void AppInit(void);
void AppImuTask(uint32_t now_ms);
void AppChassisTask(uint32_t now_ms);
void AppUsbTask(uint32_t now_ms);
void AppArmTask(uint32_t now_ms);
/** 高优先级1 kHz电机发送入口；达妙和DJI周期控制的唯一任务调用点。 */
void AppMotorControlTask(uint32_t now_ms);

#endif
