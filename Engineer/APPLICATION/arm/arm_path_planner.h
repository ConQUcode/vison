/**
 * @file arm_path_planner.h
 * @brief 无硬件副作用的工具中心直线路径预检与候选规划。
 */

#ifndef ARM_PATH_PLANNER_H
#define ARM_PATH_PLANNER_H

#include "arm.h"
#include "arm_config.h"

#define ARM_PATH_PREFLIGHT_FAIL_IK          (1u << 0)
#define ARM_PATH_PREFLIGHT_FAIL_FK_ERROR    (1u << 1)
#define ARM_PATH_PREFLIGHT_FAIL_JOINT_LIMIT (1u << 2)
#define ARM_PATH_PREFLIGHT_FAIL_AUTO_REGION (1u << 3)
#define ARM_PATH_PREFLIGHT_FAIL_TOOL_PITCH  (1u << 4)
#define ARM_PATH_PREFLIGHT_FAIL_WORKSPACE   (1u << 5)
#define ARM_PATH_PREFLIGHT_FAIL_JOINT_STEP  (1u << 6)

typedef enum {
    ARM_PATH_PLAN_OK = 0,
    ARM_PATH_PLAN_INVALID,
    ARM_PATH_PLAN_SAMPLE_CAPACITY,
    ARM_PATH_PLAN_POINT_UNSAFE,
    ARM_PATH_PLAN_NO_CONTINUOUS_IK,
    ARM_PATH_PLAN_RECONSTRUCTION_FAILED
} Arm_Path_Plan_Status_e;

typedef void (*Arm_Path_Planner_Service_Hook_t)(void *context);

typedef struct {
    Arm_Position_s start_center_mm;
    Arm_Position_s target_center_mm;
    float start_q_deg[3];
    float tool_pitch_deg;
    float sample_spacing_mm;
    Arm_Cartesian_Safety_Profile_e safety_profile;
    uint8_t segment_index;
    Arm_Path_Planner_Service_Hook_t service_hook;
    void *service_context;
} Arm_Path_Plan_Request_s;

typedef struct {
    float (*sample_q_deg)[3];
    float *sample_progress;
    uint8_t (*candidate_predecessor)[ARM_TOOL_CENTER_IK_MAX_CANDIDATES];
    uint8_t *candidate_count;
    uint8_t *selected_candidate;
    uint16_t capacity;
} Arm_Path_Plan_Workspace_s;

typedef struct {
    Arm_Path_Plan_Status_e status;
    Arm_IK_Status_e ik_status;
    Arm_Workspace_Safety_Result_e workspace_safety_result;
    uint32_t failed_check_mask;
    uint8_t failed_segment;
    uint16_t failed_sample;
    Arm_Position_s failed_center_mm;
    float failed_q_deg[3];
    float final_q_deg[3];
    uint16_t interval_count;
    uint16_t sample_count;
    float requested_distance_mm;
    float reachable_distance_mm;
} Arm_Path_Plan_Result_s;

typedef struct {
    float staging_q_deg[3];
    Arm_Position_s approach_center_mm;
    float tool_pitch_deg;
    float advance_sign;
    float requested_advance_mm;
    float sample_step_mm;
    Arm_Cartesian_Safety_Profile_e safety_profile;
    Arm_Path_Planner_Service_Hook_t service_hook;
    void *service_context;
} Arm_Path_Advance_Request_s;

typedef struct {
    float requested_advance_mm;
    float selected_advance_mm;
    uint8_t approach_failed;
    uint8_t advance_limited;
    Arm_Path_Plan_Result_s plan_result;
    Arm_Path_Plan_Result_s limiting_plan_result;
} Arm_Path_Advance_Result_s;

/**
 * 对单条工具中心直线执行与正式轨迹相同的全候选动态规划。
 * 成功时workspace保存起点及全部关节样本；失败时result仍返回最大连续
 * 可达前缀。函数不访问硬件或全局调试状态。
 */
uint8_t ArmPathPlanToolCenterSegment(
    const Arm_Path_Plan_Request_s *request,
    Arm_Path_Plan_Workspace_s *workspace,
    Arm_Path_Plan_Result_s *result);

/** 用同一规划内核选择从接近点开始的最大连续可达正推进量。 */
uint8_t ArmPathSelectReachableAdvance(
    const Arm_Path_Advance_Request_s *request,
    Arm_Path_Plan_Workspace_s *workspace,
    Arm_Path_Advance_Result_s *result);

#endif
