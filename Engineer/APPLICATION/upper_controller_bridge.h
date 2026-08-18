/**
 * @file upper_controller_bridge.h
 * @brief 新版上位机协议到现有底盘、机械臂工具和摄像头舵机接口的适配层。
 */

#ifndef UPPER_CONTROLLER_BRIDGE_H
#define UPPER_CONTROLLER_BRIDGE_H

#include <stdint.h>

#include "app_arm_flow.h"
#include "arm_host.h"
#include "camera_target_transform.h"
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

typedef enum {
    UPPER_CONTROLLER_AREA_A = 0,
    UPPER_CONTROLLER_AREA_B,
    UPPER_CONTROLLER_AREA_C,
    UPPER_CONTROLLER_AREA_D,
    UPPER_CONTROLLER_AREA_UNKNOWN = 0xFF
} Upper_Controller_Area_e;

typedef enum {
    UPPER_AC_OBSERVE_IDLE = 0,
    UPPER_AC_OBSERVE_WAIT_READY,
    UPPER_AC_OBSERVE_BASE_SUBMITTED,
    UPPER_AC_OBSERVE_SUBMIT_TARGET,
    UPPER_AC_OBSERVE_TARGET_SUBMITTED,
    UPPER_AC_OBSERVE_HOLDING,
    UPPER_AC_OBSERVE_FAILED
} Upper_Ac_Observe_State_e;

typedef enum {
    UPPER_ARM_TARGET_DEBUG_IDLE = 0,
    UPPER_ARM_TARGET_DEBUG_RX,
    UPPER_ARM_TARGET_DEBUG_INVALID,
    UPPER_ARM_TARGET_DEBUG_TRANSFORM_FAILED,
    UPPER_ARM_TARGET_DEBUG_DEFERRED,
    UPPER_ARM_TARGET_DEBUG_PICK_REJECTED,
    UPPER_ARM_TARGET_DEBUG_PICK_STARTED,
    UPPER_ARM_TARGET_DEBUG_PICK_DONE,
    UPPER_ARM_TARGET_DEBUG_PICK_FAILED,
    UPPER_ARM_TARGET_DEBUG_PLACE_REJECTED,
    UPPER_ARM_TARGET_DEBUG_PLACE_STARTED,
    UPPER_ARM_TARGET_DEBUG_PLACE_DONE
} Upper_Arm_Target_Debug_Stage_e;

typedef enum {
    UPPER_RESET_HOME_IDLE = 0,
    UPPER_RESET_HOME_SUBMIT_CANCEL,
    UPPER_RESET_HOME_WAIT_CANCEL,
    UPPER_RESET_HOME_SUBMIT_HOME,
    UPPER_RESET_HOME_WAIT_HOME,
    UPPER_RESET_HOME_FAILED
} Upper_Reset_Home_State_e;

typedef struct {
    uint32_t rx_count;
    Upper_Arm_Target_Debug_Stage_e stage;
    Camera_Target_Transform_Status_e transform_status;
    uint32_t pose_age_ms;
    uint8_t gate_flags;
    uint8_t pick_start_result;
} Upper_Arm_Target_Debug_s;

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
    uint8_t ac_active_side;
    uint8_t ac_right_pending;
    uint8_t ac_operation_status;
    uint8_t ac_start_result;
    uint32_t ac_start_count;
    uint32_t ac_side_complete_count;
    uint32_t ac_complete_count;
    uint32_t ac_fail_count;
    Upper_Ac_Observe_State_e ac_observe_state;
    uint32_t ac_observe_command_id;
    uint32_t ac_observe_base_command_id;
    uint32_t ac_observe_target_command_id;
    uint32_t ac_observe_capture_id;
    uint32_t ac_observe_start_count;
    uint32_t ac_observe_complete_count;
    uint32_t ac_observe_fail_count;
    float ac_observe_target_center_mm[3];
    float ac_observe_target_tool_pitch_deg;
    uint8_t arm_target_pick_running;
    uint8_t arm_target_pick_start_result;
    App_Arm_Flow_Status_e arm_target_pick_flow_status;
    float arm_target_pick_center_mm[3];
    float arm_target_pick_tool_pitch_deg;
    uint32_t arm_target_pick_start_count;
    uint32_t arm_target_pick_complete_count;
    uint32_t arm_target_pick_fail_count;
    Upper_Controller_Area_e current_area;
    uint8_t current_area_valid;
    uint8_t current_area_callback_pending;
    uint32_t current_area_update_count;
    uint32_t qr_pose_request_count;
    uint32_t qr_pose_unsupported_count;
    Upper_Reset_Home_State_e reset_home_state;
    uint32_t reset_home_command_id;
    Arm_Command_Result_e reset_home_submit_result;
    uint32_t reset_home_request_count;
    uint32_t reset_home_complete_count;
    uint32_t reset_home_fail_count;
    uint32_t execution_callback_tx_count;
    uint32_t execution_callback_tx_fail_count;
    float arm_target_camera_m[3];
    float arm_target_camera_mm[3];
    uint8_t arm_target_z_type;
    uint8_t arm_target_valid;
    Camera_Target_Transform_Status_e arm_target_transform_status;
    float arm_target_reference_mm[3];
    float arm_target_base_mm[3];
    uint32_t arm_target_pose_capture_id;
    uint32_t arm_target_pose_capture_tick_ms;
    uint32_t arm_target_rx_count;
    uint32_t arm_target_deferred_count;
    uint32_t arm_target_invalid_count;
    uint32_t arm_target_transform_success_count;
    uint32_t arm_target_transform_fail_count;
    uint32_t arm_pose_capture_success_count;
    uint32_t arm_pose_capture_fail_count;
    uint32_t unexpected_callback_rx_count;
} Upper_Controller_Debug_s;

extern Upper_Controller_Debug_s g_upper_controller_debug;
extern Upper_Arm_Target_Debug_s g_arm_target_debug;

/** 初始化桥状态；不会主动移动任何执行机构。 */
void UpperControllerBridgeInit(void);
/** 由USB应用任务周期调用，重试受理并观察异步夹爪命令终态。 */
void UpperControllerBridgeTask(uint32_t now_ms);
/**
 * Store the current arm feedback as the pose belonging to one camera frame.
 * This must be called by the future image-capture trigger, not when ArmTarget
 * arrives after image processing.
 */
Camera_Target_Transform_Status_e UpperControllerCaptureCameraPose(
    uint32_t capture_id, uint32_t now_ms);
/** 读取上位机最近一次声明的当前区域；尚未收到task 5时返回0。 */
uint8_t UpperControllerGetCurrentArea(Upper_Controller_Area_e *area);

#endif
