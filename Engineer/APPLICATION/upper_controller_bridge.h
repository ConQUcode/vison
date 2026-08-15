/**
 * @file upper_controller_bridge.h
 * @brief 新版上位机协议到现有底盘、机械臂工具和摄像头舵机接口的适配层。
 */

#ifndef UPPER_CONTROLLER_BRIDGE_H
#define UPPER_CONTROLLER_BRIDGE_H

#include <stdint.h>

#include "arm_host.h"
#include "chassis.h"
#include "protocol.h"

#define UPPER_CAMERA_LOOK_DOWN_DEG (-45.0f)
#define UPPER_CAMERA_LOOK_UP_DEG     45.0f
#define UPPER_CAMERA_SETTLE_MS      500u

typedef enum {
    UPPER_DISCRETE_IDLE = 0,
    UPPER_DISCRETE_PENDING,
    UPPER_DISCRETE_RUNNING
} Upper_Discrete_State_e;

typedef struct {
    uint8_t initialized;
    Upper_Discrete_State_e discrete_state;
    uint8_t pending_task_id;
    uint8_t pending_task_status;
    uint32_t gripper_command_id;
    uint32_t camera_motion_start_tick;
    Arm_Command_Result_e gripper_submit_result;
    Chassis_Command_Result_e velocity_submit_result;
    uint32_t velocity_command_id;
    float velocity_linear_x_m_s;
    float velocity_angular_z_rad_s;
    float velocity_vx_mm_s;
    uint32_t velocity_rx_count;
    uint32_t velocity_accept_count;
    uint32_t velocity_reject_count;
    uint32_t discrete_rx_count;
    uint32_t discrete_complete_count;
    uint32_t discrete_duplicate_count;
    uint32_t discrete_busy_count;
    uint32_t discrete_invalid_count;
    uint32_t execution_callback_tx_count;
    uint32_t execution_callback_tx_fail_count;
    float arm_target_camera_m[3];
    float arm_target_camera_mm[3];
    uint8_t arm_target_z_type;
    uint8_t arm_target_valid;
    uint32_t arm_target_rx_count;
    uint32_t arm_target_deferred_count;
    uint32_t arm_target_invalid_count;
    uint32_t unexpected_callback_rx_count;
} Upper_Controller_Debug_s;

extern Upper_Controller_Debug_s g_upper_controller_debug;

/** 初始化桥状态；不会主动移动任何执行机构。 */
void UpperControllerBridgeInit(void);
/** 由USB应用任务周期调用，重试受理并观察异步夹爪命令终态。 */
void UpperControllerBridgeTask(uint32_t now_ms);

#endif
