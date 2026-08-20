/**
 * @file arm_path_planner.c
 * @brief 工具中心直线路径的共享全候选预检实现。
 */

#include "arm_path_planner.h"

#include "arm_kinematics.h"
#include "arm_tool.h"

#include <math.h>
#include <string.h>

#define ARM_PATH_CANDIDATE_NONE 0xFFu
#define ARM_PATH_COST_INFINITY  1.0e30f

static float ArmPathWrapTo180(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float ArmPathDistance(const Arm_Position_s *a,
                             const Arm_Position_s *b)
{
    float dx = b->x_mm - a->x_mm;
    float dy = b->y_mm - a->y_mm;
    float dz = b->z_mm - a->z_mm;

    return sqrtf(dx * dx + dy * dy + dz * dz);
}

static uint8_t ArmPathAcSidePick(const Arm_Path_Plan_Request_s *request)
{
    return request->safety_profile == ARM_CARTESIAN_SAFETY_AC_SIDE_PICK;
}

static uint8_t ArmPathPointIsRear(const Arm_Path_Plan_Request_s *request,
                                  const Arm_Position_s *point)
{
#if ARM_WORKSPACE_SAFETY_ENABLE != 0u
    if (ArmPathAcSidePick(request) != 0u) {
        return 0u;
    }
    return point != NULL &&
           point->x_mm < ARM_REAR_ZONE_X_BOUNDARY_MM -
                         ARM_REAR_ZONE_X_MARGIN_MM;
#else
    (void)request;
    (void)point;
    return 0u;
#endif
}

static uint8_t ArmPathPointSafe(
    const Arm_Path_Plan_Request_s *request,
    const Arm_Position_s *point, uint8_t target_point,
    Arm_Workspace_Safety_Result_e *workspace_result)
{
#if ARM_WORKSPACE_SAFETY_ENABLE != 0u
    if (point == NULL || !isfinite(point->x_mm) ||
        !isfinite(point->y_mm) || !isfinite(point->z_mm)) {
        *workspace_result = ARM_WORKSPACE_SAFETY_PREFLIGHT_IK;
        return 0u;
    }
    if (ArmPathPointIsRear(request, point) != 0u &&
        point->z_mm < ARM_REAR_ZONE_MIN_TOOL_Z_MM) {
        *workspace_result = target_point != 0u ?
            ARM_WORKSPACE_SAFETY_TARGET_REAR_TOO_LOW :
            ARM_WORKSPACE_SAFETY_PATH_REAR_TOO_LOW;
        return 0u;
    }
#else
    (void)request;
    (void)point;
    (void)target_point;
    (void)workspace_result;
#endif
    return 1u;
}

static uint8_t ArmPathAutoPoseSafe(
    const Arm_Path_Plan_Request_s *request, const float q_deg[3])
{
    if (ArmPathAcSidePick(request) != 0u) {
        return ArmAutoPoseIsSafeWithQ1Limits(
            q_deg, ARM_AC_SIDE_PICK_Q1_MIN_DEG,
            ARM_AC_SIDE_PICK_Q1_MAX_DEG);
    }
    return ArmAutoPoseIsSafe(q_deg);
}

static Arm_IK_Status_e ArmPathInverseAll(
    const Arm_Path_Plan_Request_s *request,
    const Arm_Position_s *center, const float seed_q_deg[3],
    Arm_Tool_Center_IK_Candidate_s candidates[
        ARM_TOOL_CENTER_IK_MAX_CANDIDATES],
    uint8_t *candidate_count)
{
    if (ArmPathAcSidePick(request) != 0u) {
        return ArmInverseKinematicsToolCenterAllWithQ1Limits(
            center, request->tool_pitch_deg, seed_q_deg,
            ARM_AC_SIDE_PICK_Q1_MIN_DEG, ARM_AC_SIDE_PICK_Q1_MAX_DEG,
            candidates, candidate_count);
    }
    return ArmInverseKinematicsToolCenterAll(
        center, request->tool_pitch_deg, seed_q_deg,
        candidates, candidate_count);
}

static uint8_t ArmPathBaseFacesFront(float q1_deg)
{
    return fabsf(ArmPathWrapTo180(q1_deg)) <=
        ARM_FRONT_BARRIER_BASE_Q1_ABS_MAX_DEG +
            ARM_LIMIT_TOLERANCE_DEG;
}

static uint8_t ArmPathPoseSafe(
    const Arm_Path_Plan_Request_s *request, const float q_deg[3],
    uint8_t target_point,
    Arm_Workspace_Safety_Result_e *workspace_result)
{
    Arm_Position_s center;

    if (!ArmForwardKinematicsToolCenter(
            q_deg, request->tool_pitch_deg, &center) ||
        !ArmPathPointSafe(request, &center, target_point,
                          workspace_result)) {
        return 0u;
    }
#if ARM_WORKSPACE_SAFETY_ENABLE != 0u && \
    ARM_FRONT_BARRIER_SHOULDER_LIMIT_ENABLE != 0u
    if (ArmPathBaseFacesFront(q_deg[ARM_JOINT_BASE_YAW]) != 0u &&
        center.x_mm > ARM_FRONT_BARRIER_TOOL_X_MARGIN_MM &&
        q_deg[ARM_JOINT_SHOULDER] >
            ARM_FRONT_BARRIER_SHOULDER_Q2_MAX_DEG) {
        *workspace_result = ARM_WORKSPACE_SAFETY_FRONT_SHOULDER_LIMIT;
        return 0u;
    }
#else
    (void)ArmPathBaseFacesFront;
#endif
    return 1u;
}

static uint8_t ArmPathJointStepContinuous(const float previous_q_deg[3],
                                          const float next_q_deg[3])
{
    return fabsf(ArmPathWrapTo180(next_q_deg[0] - previous_q_deg[0])) <=
               ARM_LINEAR_Q1_STEP_MAX_DEG &&
           fabsf(next_q_deg[1] - previous_q_deg[1]) <=
               ARM_LINEAR_Q2_STEP_MAX_DEG &&
           fabsf(next_q_deg[2] - previous_q_deg[2]) <=
               ARM_LINEAR_Q3_STEP_MAX_DEG;
}

static float ArmPathTransitionCost(const float previous_q_deg[3],
                                   const float next_q_deg[3])
{
    float dq1 = ArmPathWrapTo180(next_q_deg[0] - previous_q_deg[0]) /
                ARM_LINEAR_Q1_STEP_MAX_DEG;
    float dq2 = (next_q_deg[1] - previous_q_deg[1]) /
                ARM_LINEAR_Q2_STEP_MAX_DEG;
    float dq3 = (next_q_deg[2] - previous_q_deg[2]) /
                ARM_LINEAR_Q3_STEP_MAX_DEG;

    return dq1 * dq1 + dq2 * dq2 + dq3 * dq3;
}

static void ArmPathSampleCenter(const Arm_Path_Plan_Request_s *request,
                                float ratio, Arm_Position_s *center)
{
    center->x_mm = request->start_center_mm.x_mm + ratio *
        (request->target_center_mm.x_mm - request->start_center_mm.x_mm);
    center->y_mm = request->start_center_mm.y_mm + ratio *
        (request->target_center_mm.y_mm - request->start_center_mm.y_mm);
    center->z_mm = request->start_center_mm.z_mm + ratio *
        (request->target_center_mm.z_mm - request->start_center_mm.z_mm);
}

static void ArmPathRecordFailure(
    Arm_Path_Plan_Result_s *result, Arm_Path_Plan_Status_e status,
    uint16_t sample, uint32_t check_mask,
    const Arm_Position_s *center, const float q_deg[3])
{
    result->status = status;
    result->failed_sample = sample;
    result->failed_check_mask = check_mask;
    if (center != NULL) {
        result->failed_center_mm = *center;
    }
    if (q_deg != NULL) {
        memcpy(result->failed_q_deg, q_deg,
               sizeof(result->failed_q_deg));
    }
}

uint8_t ArmPathPlanToolCenterSegment(
    const Arm_Path_Plan_Request_s *request,
    Arm_Path_Plan_Workspace_s *workspace,
    Arm_Path_Plan_Result_s *result)
{
    float previous_cost[ARM_TOOL_CENTER_IK_MAX_CANDIDATES];
    float current_cost[ARM_TOOL_CENTER_IK_MAX_CANDIDATES];
    Arm_Tool_Center_IK_Candidate_s previous_candidates[
        ARM_TOOL_CENTER_IK_MAX_CANDIDATES];
    float reconstruction_seed_q_deg[3];
    float length_mm;
    uint16_t intervals;
    uint8_t previous_candidate_count = 0u;
    uint8_t best_last_candidate = ARM_PATH_CANDIDATE_NONE;
    uint8_t unsafe_vertical_escape;

    if (result == NULL) {
        return 0u;
    }
    memset(result, 0, sizeof(*result));
    result->status = ARM_PATH_PLAN_INVALID;
    result->ik_status = ARM_IK_INVALID_ARGUMENT;
    result->workspace_safety_result = ARM_WORKSPACE_SAFETY_OK;
    result->failed_segment = request != NULL ? request->segment_index :
                                               0xFFu;
    if (request == NULL || workspace == NULL ||
        workspace->sample_q_deg == NULL ||
        workspace->sample_progress == NULL ||
        workspace->candidate_predecessor == NULL ||
        workspace->candidate_count == NULL ||
        workspace->selected_candidate == NULL ||
        workspace->capacity < 2u ||
        !isfinite(request->sample_spacing_mm) ||
        request->sample_spacing_mm <= 0.0f ||
        !isfinite(request->tool_pitch_deg) ||
        request->safety_profile > ARM_CARTESIAN_SAFETY_AC_SIDE_PICK ||
        !ArmJointPoseWithinSoftLimits(request->start_q_deg)) {
        return 0u;
    }
    length_mm = ArmPathDistance(&request->start_center_mm,
                                &request->target_center_mm);
    if (!isfinite(length_mm)) {
        return 0u;
    }
    result->requested_distance_mm = length_mm;
    intervals = (uint16_t)ceilf(length_mm / request->sample_spacing_mm);
    if (intervals < 1u) {
        intervals = 1u;
    }
    result->interval_count = intervals;
    if ((uint32_t)intervals + 1u > workspace->capacity) {
        result->status = ARM_PATH_PLAN_SAMPLE_CAPACITY;
        result->workspace_safety_result =
            ARM_WORKSPACE_SAFETY_SAMPLE_CAPACITY;
        return 0u;
    }
    unsafe_vertical_escape =
        ArmPathPointIsRear(request, &request->start_center_mm) != 0u &&
        request->start_center_mm.z_mm < ARM_REAR_ZONE_MIN_TOOL_Z_MM &&
        fabsf(request->target_center_mm.x_mm -
              request->start_center_mm.x_mm) <= 0.5f &&
        fabsf(request->target_center_mm.y_mm -
              request->start_center_mm.y_mm) <= 0.5f &&
        request->target_center_mm.z_mm >= ARM_REAR_CROSSING_TOOL_Z_MM;

    {
        uint32_t start_fail_mask = 0u;

        if (!ArmPathAutoPoseSafe(request, request->start_q_deg)) {
            start_fail_mask |= ARM_PATH_PREFLIGHT_FAIL_AUTO_REGION;
        }
        if (!ArmToolPitchValidForPose(request->tool_pitch_deg,
                                      request->start_q_deg)) {
            start_fail_mask |= ARM_PATH_PREFLIGHT_FAIL_TOOL_PITCH;
        }
        if (unsafe_vertical_escape == 0u &&
            !ArmPathPoseSafe(request, request->start_q_deg, 0u,
                             &result->workspace_safety_result)) {
            start_fail_mask |= ARM_PATH_PREFLIGHT_FAIL_WORKSPACE;
        }
        if (start_fail_mask != 0u) {
            ArmPathRecordFailure(
                result, ARM_PATH_PLAN_POINT_UNSAFE, 0u,
                start_fail_mask, &request->start_center_mm,
                request->start_q_deg);
            return 0u;
        }
    }

    memcpy(workspace->sample_q_deg[0], request->start_q_deg,
           sizeof(request->start_q_deg));
    workspace->sample_progress[0] = 0.0f;
    workspace->candidate_count[0] = 1u;
    workspace->selected_candidate[0] = 0u;
    memset(previous_candidates, 0, sizeof(previous_candidates));
    for (uint8_t candidate = 0u;
         candidate < ARM_TOOL_CENTER_IK_MAX_CANDIDATES; ++candidate) {
        previous_cost[candidate] = ARM_PATH_COST_INFINITY;
    }

    for (uint16_t i = 1u; i <= intervals; ++i) {
        float ratio = (float)i / (float)intervals;
        Arm_Position_s sample_center;
        Arm_Tool_Center_IK_Candidate_s candidates[
            ARM_TOOL_CENTER_IK_MAX_CANDIDATES];
        Arm_IK_Status_e ik_status;
        uint8_t candidate_count = 0u;
        uint8_t reachable_count = 0u;
        uint32_t failed_check_mask = 0u;

        if (request->service_hook != NULL) {
            request->service_hook(request->service_context);
        }
        ArmPathSampleCenter(request, ratio, &sample_center);
        if (unsafe_vertical_escape == 0u &&
            !ArmPathPointSafe(request, &sample_center,
                              i == intervals ? 1u : 0u,
                              &result->workspace_safety_result)) {
            ArmPathRecordFailure(
                result, ARM_PATH_PLAN_POINT_UNSAFE, i,
                ARM_PATH_PREFLIGHT_FAIL_WORKSPACE, &sample_center, NULL);
            result->reachable_distance_mm =
                length_mm * (float)(i - 1u) / (float)intervals;
            return 0u;
        }
        memset(candidates, 0, sizeof(candidates));
        ik_status = ArmPathInverseAll(
            request, &sample_center, request->start_q_deg,
            candidates, &candidate_count);
        result->ik_status = ik_status;
        workspace->candidate_count[i] = candidate_count;
        for (uint8_t candidate = 0u;
             candidate < ARM_TOOL_CENTER_IK_MAX_CANDIDATES; ++candidate) {
            current_cost[candidate] = ARM_PATH_COST_INFINITY;
            workspace->candidate_predecessor[i][candidate] =
                ARM_PATH_CANDIDATE_NONE;
        }
        if (ik_status != ARM_IK_OK || candidate_count == 0u) {
            failed_check_mask |= ARM_PATH_PREFLIGHT_FAIL_IK;
        } else {
            for (uint8_t candidate = 0u; candidate < candidate_count;
                 ++candidate) {
                uint8_t candidate_safe = 1u;
                uint32_t candidate_fail_mask = 0u;
                Arm_Workspace_Safety_Result_e candidate_workspace =
                    ARM_WORKSPACE_SAFETY_OK;

                if (candidates[candidate].position_error_mm >
                    ARM_LINEAR_FK_ERROR_MAX_MM) {
                    candidate_fail_mask |= ARM_PATH_PREFLIGHT_FAIL_FK_ERROR;
                    candidate_safe = 0u;
                }
                if (!ArmJointPoseWithinSoftLimits(
                        candidates[candidate].q_deg)) {
                    candidate_fail_mask |=
                        ARM_PATH_PREFLIGHT_FAIL_JOINT_LIMIT;
                    candidate_safe = 0u;
                }
                if (!ArmPathAutoPoseSafe(
                        request, candidates[candidate].q_deg)) {
                    candidate_fail_mask |=
                        ARM_PATH_PREFLIGHT_FAIL_AUTO_REGION;
                    candidate_safe = 0u;
                }
                if (!ArmToolPitchValidForPose(
                        request->tool_pitch_deg,
                        candidates[candidate].q_deg)) {
                    candidate_fail_mask |=
                        ARM_PATH_PREFLIGHT_FAIL_TOOL_PITCH;
                    candidate_safe = 0u;
                }
                if (!ArmPathPoseSafe(
                        request, candidates[candidate].q_deg,
                        i == intervals ? 1u : 0u,
                        &candidate_workspace)) {
                    candidate_fail_mask |=
                        ARM_PATH_PREFLIGHT_FAIL_WORKSPACE;
                    candidate_safe = 0u;
                    if (result->workspace_safety_result ==
                        ARM_WORKSPACE_SAFETY_OK) {
                        result->workspace_safety_result = candidate_workspace;
                    }
                }
                failed_check_mask |= candidate_fail_mask;
                if (candidate_safe == 0u) {
                    continue;
                }
                if (i == 1u) {
                    if (ArmPathJointStepContinuous(
                            request->start_q_deg,
                            candidates[candidate].q_deg)) {
                        current_cost[candidate] = ArmPathTransitionCost(
                            request->start_q_deg,
                            candidates[candidate].q_deg);
                        reachable_count++;
                    }
                } else {
                    for (uint8_t predecessor = 0u;
                         predecessor < previous_candidate_count;
                         ++predecessor) {
                        float cost;

                        if (previous_cost[predecessor] >=
                                ARM_PATH_COST_INFINITY ||
                            !ArmPathJointStepContinuous(
                                previous_candidates[predecessor].q_deg,
                                candidates[candidate].q_deg)) {
                            continue;
                        }
                        cost = previous_cost[predecessor] +
                            ArmPathTransitionCost(
                                previous_candidates[predecessor].q_deg,
                                candidates[candidate].q_deg);
                        if (cost < current_cost[candidate]) {
                            current_cost[candidate] = cost;
                            workspace->candidate_predecessor[i][candidate] =
                                predecessor;
                        }
                    }
                    if (current_cost[candidate] < ARM_PATH_COST_INFINITY) {
                        reachable_count++;
                    }
                }
            }
            if (reachable_count == 0u) {
                failed_check_mask |= ARM_PATH_PREFLIGHT_FAIL_JOINT_STEP;
            }
        }
        if ((failed_check_mask & ARM_PATH_PREFLIGHT_FAIL_IK) != 0u ||
            reachable_count == 0u) {
            const float *failed_q = candidate_count != 0u ?
                candidates[0].q_deg : NULL;

            ArmPathRecordFailure(
                result, ARM_PATH_PLAN_NO_CONTINUOUS_IK, i,
                failed_check_mask, &sample_center, failed_q);
            result->reachable_distance_mm =
                length_mm * (float)(i - 1u) / (float)intervals;
            if (result->workspace_safety_result ==
                ARM_WORKSPACE_SAFETY_OK) {
                result->workspace_safety_result =
                    ARM_WORKSPACE_SAFETY_PREFLIGHT_IK;
            }
            return 0u;
        }
        workspace->sample_progress[i] = ratio;
        result->reachable_distance_mm = length_mm * ratio;
        for (uint8_t candidate = 0u; candidate < candidate_count;
             ++candidate) {
            previous_cost[candidate] = current_cost[candidate];
        }
        for (uint8_t candidate = candidate_count;
             candidate < ARM_TOOL_CENTER_IK_MAX_CANDIDATES; ++candidate) {
            previous_cost[candidate] = ARM_PATH_COST_INFINITY;
        }
        memcpy(previous_candidates, candidates,
               sizeof(previous_candidates));
        previous_candidate_count = candidate_count;
    }

    for (uint8_t candidate = 0u;
         candidate < workspace->candidate_count[intervals]; ++candidate) {
        if (previous_cost[candidate] < ARM_PATH_COST_INFINITY &&
            (best_last_candidate == ARM_PATH_CANDIDATE_NONE ||
             previous_cost[candidate] < previous_cost[best_last_candidate])) {
            best_last_candidate = candidate;
        }
    }
    if (best_last_candidate == ARM_PATH_CANDIDATE_NONE) {
        ArmPathRecordFailure(
            result, ARM_PATH_PLAN_RECONSTRUCTION_FAILED, intervals,
            ARM_PATH_PREFLIGHT_FAIL_JOINT_STEP,
            &request->target_center_mm, NULL);
        return 0u;
    }
    workspace->selected_candidate[intervals] = best_last_candidate;
    for (uint16_t i = intervals; i > 1u; --i) {
        uint8_t selected = workspace->selected_candidate[i];
        uint8_t predecessor;

        if (selected >= ARM_TOOL_CENTER_IK_MAX_CANDIDATES) {
            ArmPathRecordFailure(
                result, ARM_PATH_PLAN_RECONSTRUCTION_FAILED, i,
                ARM_PATH_PREFLIGHT_FAIL_JOINT_STEP, NULL, NULL);
            return 0u;
        }
        predecessor = workspace->candidate_predecessor[i][selected];
        if (predecessor == ARM_PATH_CANDIDATE_NONE ||
            predecessor >= ARM_TOOL_CENTER_IK_MAX_CANDIDATES) {
            ArmPathRecordFailure(
                result, ARM_PATH_PLAN_RECONSTRUCTION_FAILED, i,
                ARM_PATH_PREFLIGHT_FAIL_JOINT_STEP, NULL, NULL);
            return 0u;
        }
        workspace->selected_candidate[i - 1u] = predecessor;
    }

    memcpy(reconstruction_seed_q_deg, request->start_q_deg,
           sizeof(reconstruction_seed_q_deg));
    for (uint16_t i = 1u; i <= intervals; ++i) {
        float ratio = (float)i / (float)intervals;
        Arm_Position_s sample_center;
        Arm_Tool_Center_IK_Candidate_s candidates[
            ARM_TOOL_CENTER_IK_MAX_CANDIDATES];
        uint8_t candidate_count = 0u;
        uint8_t selected = workspace->selected_candidate[i];

        if (request->service_hook != NULL) {
            request->service_hook(request->service_context);
        }
        ArmPathSampleCenter(request, ratio, &sample_center);
        memset(candidates, 0, sizeof(candidates));
        if (ArmPathInverseAll(
                request, &sample_center, reconstruction_seed_q_deg,
                candidates, &candidate_count) != ARM_IK_OK ||
            selected >= candidate_count) {
            ArmPathRecordFailure(
                result, ARM_PATH_PLAN_RECONSTRUCTION_FAILED, i,
                ARM_PATH_PREFLIGHT_FAIL_IK, &sample_center, NULL);
            return 0u;
        }
        memcpy(workspace->sample_q_deg[i], candidates[selected].q_deg,
               sizeof(candidates[selected].q_deg));
        memcpy(reconstruction_seed_q_deg, candidates[selected].q_deg,
               sizeof(reconstruction_seed_q_deg));
    }
    result->status = ARM_PATH_PLAN_OK;
    result->ik_status = ARM_IK_OK;
    result->workspace_safety_result = ARM_WORKSPACE_SAFETY_OK;
    result->sample_count = (uint16_t)(intervals + 1u);
    result->reachable_distance_mm = length_mm;
    memcpy(result->final_q_deg, reconstruction_seed_q_deg,
           sizeof(result->final_q_deg));
    return 1u;
}

uint8_t ArmPathSelectReachableAdvance(
    const Arm_Path_Advance_Request_s *request,
    Arm_Path_Plan_Workspace_s *workspace,
    Arm_Path_Advance_Result_s *result)
{
    Arm_Path_Plan_Request_s segment_request;
    Arm_Path_Plan_Result_s approach_plan;
    Arm_Position_s staging_center;
    float selected_mm;
    float step_count;

    if (result == NULL) {
        return 0u;
    }
    memset(result, 0, sizeof(*result));
    result->plan_result.status = ARM_PATH_PLAN_INVALID;
    result->plan_result.ik_status = ARM_IK_INVALID_ARGUMENT;
    if (request == NULL || workspace == NULL ||
        !isfinite(request->advance_sign) ||
        fabsf(fabsf(request->advance_sign) - 1.0f) > 0.001f ||
        !isfinite(request->requested_advance_mm) ||
        request->requested_advance_mm <= 0.0f ||
        !isfinite(request->sample_step_mm) ||
        request->sample_step_mm <= 0.0f ||
        !isfinite(request->tool_pitch_deg)) {
        return 0u;
    }
    result->requested_advance_mm = request->requested_advance_mm;
    step_count = request->requested_advance_mm / request->sample_step_mm;
    if (!isfinite(step_count) ||
        fabsf(step_count - floorf(step_count + 0.5f)) > 0.0001f ||
        !ArmForwardKinematicsToolCenter(
            request->staging_q_deg, request->tool_pitch_deg,
            &staging_center)) {
        return 0u;
    }

    memset(&segment_request, 0, sizeof(segment_request));
    segment_request.start_center_mm = staging_center;
    segment_request.target_center_mm = request->approach_center_mm;
    memcpy(segment_request.start_q_deg, request->staging_q_deg,
           sizeof(segment_request.start_q_deg));
    segment_request.tool_pitch_deg = request->tool_pitch_deg;
    segment_request.sample_spacing_mm = request->sample_step_mm;
    segment_request.safety_profile = request->safety_profile;
    segment_request.service_hook = request->service_hook;
    segment_request.service_context = request->service_context;
    if (!ArmPathPlanToolCenterSegment(
            &segment_request, workspace, &approach_plan)) {
        result->approach_failed = 1u;
        result->plan_result = approach_plan;
        return 0u;
    }

    segment_request.start_center_mm = request->approach_center_mm;
    segment_request.target_center_mm = request->approach_center_mm;
    segment_request.target_center_mm.y_mm +=
        request->advance_sign * request->requested_advance_mm;
    memcpy(segment_request.start_q_deg, approach_plan.final_q_deg,
           sizeof(segment_request.start_q_deg));
    if (ArmPathPlanToolCenterSegment(
            &segment_request, workspace, &result->plan_result)) {
        selected_mm = request->requested_advance_mm;
    } else {
        result->advance_limited = 1u;
        result->limiting_plan_result = result->plan_result;
        selected_mm = floorf(
            result->plan_result.reachable_distance_mm /
                request->sample_step_mm + 0.0001f) *
            request->sample_step_mm;
        if (selected_mm <= 0.0f) {
            return 0u;
        }
        segment_request.target_center_mm = request->approach_center_mm;
        segment_request.target_center_mm.y_mm +=
            request->advance_sign * selected_mm;
        if (!ArmPathPlanToolCenterSegment(
                &segment_request, workspace, &result->plan_result)) {
            return 0u;
        }
    }
    result->selected_advance_mm = selected_mm;
    return 1u;
}
