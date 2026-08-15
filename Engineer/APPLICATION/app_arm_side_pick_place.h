/**
 * @file app_arm_side_pick_place.h
 * @brief 可由上位机或应用调度器提交的单侧完整抓取与放置任务。
 */

#ifndef APP_ARM_SIDE_PICK_PLACE_H
#define APP_ARM_SIDE_PICK_PLACE_H

#include <stdint.h>

#include "app_fruit_task.h"

typedef enum {
    APP_ARM_SIDE_PICK_PLACE_IDLE = 0,
    APP_ARM_SIDE_PICK_PLACE_RUNNING,
    APP_ARM_SIDE_PICK_PLACE_DONE,
    APP_ARM_SIDE_PICK_PLACE_FAILED
} App_Arm_Side_Pick_Place_Status_e;

typedef enum {
    APP_ARM_SIDE_PICK_PLACE_START_ACCEPTED = 0,
    APP_ARM_SIDE_PICK_PLACE_START_BUSY,
    APP_ARM_SIDE_PICK_PLACE_START_INVALID_SIDE,
    APP_ARM_SIDE_PICK_PLACE_START_FAILED
} App_Arm_Side_Pick_Place_Start_Result_e;

typedef enum {
    APP_ARM_POSTURE_TEST_WAIT_READY = 0,
    APP_ARM_POSTURE_TEST_WAIT_BASE_AIM,
    APP_ARM_POSTURE_TEST_SUBMIT_TARGET,
    APP_ARM_POSTURE_TEST_WAIT_TARGET,
    APP_ARM_POSTURE_TEST_SUBMIT_ADVANCE,
    APP_ARM_POSTURE_TEST_WAIT_ADVANCE,
    APP_ARM_POSTURE_TEST_WAIT_PITCH_STABLE,
    APP_ARM_POSTURE_TEST_PICK_DWELL,
    APP_ARM_POSTURE_TEST_SUBMIT_CLOSE,
    APP_ARM_POSTURE_TEST_WAIT_CLOSE,
    APP_ARM_POSTURE_TEST_POST_GRIP_DWELL,
    APP_ARM_POSTURE_TEST_START_PLACE,
    APP_ARM_POSTURE_TEST_WAIT_PLACE,
    APP_ARM_POSTURE_TEST_FAILED,
    APP_ARM_POSTURE_TEST_DONE
} App_Arm_Posture_Test_State_e;

/** 单侧完整抓放Watch；保留原全局名以兼容现有Keil Watch配置。 */
typedef struct {
    App_Arm_Posture_Test_State_e state;
    uint8_t active_side; /* App_Fruit_Side_e：LEFT=1，RIGHT=2。 */
    uint32_t completed_count;
    uint32_t left_completed_count;
    uint32_t right_completed_count;
    uint32_t base_command_id;
    uint32_t target_command_id;
    uint32_t advance_command_id;
    uint32_t active_command_id;
    uint32_t submit_result;
    uint32_t command_state;
    uint32_t host_state;
    uint32_t fault_code;
    uint8_t gripper_state;
    uint8_t arm_flow_status;
    uint8_t arm_place_step;
    uint32_t place_profile_id;
    uint32_t place_start_result;
    float target_center_mm[3];
    float advance_center_mm[3];
    float target_tool_pitch_deg;
    float target_q_deg[3];
    float feedback_q_deg[3];
    float error_q_deg[3];
    float feedback_center_mm[3];
    float center_error_mm[3];
    float small_link_pitch_deg;
    float tool_pitch_feedback_deg;
    float tool_pitch_error_deg;
    uint32_t state_elapsed_ms;
    uint32_t update_count;
    uint8_t operation_status;
    uint8_t last_start_result;
    uint32_t start_count;
} App_Arm_Posture_Test_Debug_s;

extern App_Arm_Posture_Test_Debug_s g_app_arm_posture_test_debug;

/** 清零任务状态和Watch；不会重复初始化底层机械臂或AppArmFlow。 */
void AppArmSidePickPlaceInit(void);

/**
 * 提交一次完整单侧任务。受理后依次完成准备、接近、推进、抓取、对应侧
 * 放置和回正；运行中重复提交返回BUSY，不改变当前任务。
 */
App_Arm_Side_Pick_Place_Start_Result_e AppArmSidePickPlaceStart(
    App_Fruit_Side_e side, uint32_t now_ms);

/** 由机械臂应用任务周期调用；DONE表示本次指定侧完整流程已经结束。 */
App_Arm_Side_Pick_Place_Status_e AppArmSidePickPlacePoll(uint32_t now_ms);

#endif
