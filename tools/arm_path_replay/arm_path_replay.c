#include "arm_kinematics.h"
#include "arm_config.h"
#include "arm_path_planner.h"
#include "arm_tool.h"
#include "app_config.h"
#include "app_fruit_task_config.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define HOST_PI 3.14159265358979323846f
#define HOST_MAX_POINTS ARM_LINEAR_MAX_SAMPLES
#define HOST_INF 1.0e30f
static float host_sample_q_deg[HOST_MAX_POINTS][3];
static float host_sample_progress[HOST_MAX_POINTS];
static uint8_t host_candidate_predecessor[HOST_MAX_POINTS]
                                         [ARM_TOOL_CENTER_IK_MAX_CANDIDATES];
static uint8_t host_candidate_count[HOST_MAX_POINTS];
static uint8_t host_selected_candidate[HOST_MAX_POINTS];
static Arm_Position_s host_route[ARM_TRAJECTORY_MAX_ROUTE_SEGMENTS + 1u];
static uint8_t host_route_segment_count;

static float wrap180(float value)
{
    while (value > 180.0f) value -= 360.0f;
    while (value < -180.0f) value += 360.0f;
    return value;
}

static uint8_t pitch_safe(const float q[3], float pitch_deg)
{
    return ArmToolPitchValidForPose(pitch_deg, q);
}

static int solve_segment(
    const Arm_Position_s *start, const Arm_Position_s *target,
    float pitch_deg, Arm_Cartesian_Safety_Profile_e safety_profile,
    const float start_q[3], const char *csv_name, float final_q[3],
    Arm_Path_Plan_Result_s *plan_result)
{
    Arm_Path_Plan_Request_s request;
    Arm_Path_Plan_Workspace_s workspace;
    Arm_Path_Plan_Result_s local_result;
    FILE *csv = NULL;

    memset(&request, 0, sizeof(request));
    request.start_center_mm = *start;
    request.target_center_mm = *target;
    memcpy(request.start_q_deg, start_q, sizeof(request.start_q_deg));
    request.tool_pitch_deg = pitch_deg;
    request.sample_spacing_mm = ARM_LINEAR_SAMPLE_SPACING_MM;
    request.safety_profile = safety_profile;

    workspace.sample_q_deg = host_sample_q_deg;
    workspace.sample_progress = host_sample_progress;
    workspace.candidate_predecessor = host_candidate_predecessor;
    workspace.candidate_count = host_candidate_count;
    workspace.selected_candidate = host_selected_candidate;
    workspace.capacity = HOST_MAX_POINTS;
    if (!ArmPathPlanToolCenterSegment(
            &request, &workspace, &local_result)) {
        fprintf(stderr,
                "FAIL shared planner status=%d ik=%d mask=0x%08lx "
                "sample=%u reachable=%.3f/%.3f center=(%.3f,%.3f,%.3f)\n",
                (int)local_result.status, (int)local_result.ik_status,
                (unsigned long)local_result.failed_check_mask,
                local_result.failed_sample,
                local_result.reachable_distance_mm,
                local_result.requested_distance_mm,
                local_result.failed_center_mm.x_mm,
                local_result.failed_center_mm.y_mm,
                local_result.failed_center_mm.z_mm);
        if (plan_result != NULL) *plan_result = local_result;
        return 4;
    }
    if (csv_name != NULL) {
        csv = fopen(csv_name, "w");
        if (csv == NULL) return 2;
        fprintf(csv, "sample,progress,q1,q2,q3\n");
        for (uint16_t i = 0u; i < local_result.sample_count; ++i) {
            fprintf(csv, "%u,%.9g,%.6f,%.6f,%.6f\n", i,
                    host_sample_progress[i], host_sample_q_deg[i][0],
                    host_sample_q_deg[i][1], host_sample_q_deg[i][2]);
        }
        fclose(csv);
    }
    if (final_q != NULL) {
        memcpy(final_q, host_sample_q_deg[local_result.interval_count],
               sizeof(request.start_q_deg));
    }
    if (plan_result != NULL) *plan_result = local_result;
    return 0;
}

static uint8_t build_route(Arm_Position_s start, Arm_Position_s target)
{
    float clearance = fmaxf(ARM_REAR_CROSSING_TOOL_Z_MM,
                            fmaxf(start.z_mm, target.z_mm));
    float start_radius = hypotf(start.x_mm, start.y_mm);
    float target_radius = hypotf(target.x_mm, target.y_mm);
    float start_sign = start.x_mm >= 0.0f ? 1.0f : -1.0f;
    float target_sign = target.x_mm >= 0.0f ? 1.0f : -1.0f;
    uint8_t arc_steps = (uint8_t)(90.0f /
        ARM_REAR_BYPASS_ARC_STEP_DEG + 0.5f);
    host_route_segment_count = 0u;
    host_route[0] = start;
    host_route[++host_route_segment_count] = start;
    host_route[host_route_segment_count].z_mm = clearance;
    for (uint8_t step = 1u; step <= arc_steps; ++step) {
        float angle = step * ARM_REAR_BYPASS_ARC_STEP_DEG * HOST_PI / 180.0f;
        host_route[++host_route_segment_count].x_mm =
            start_sign * start_radius * cosf(angle);
        host_route[host_route_segment_count].y_mm =
            -start_sign * start_radius * sinf(angle);
        host_route[host_route_segment_count].z_mm = clearance;
    }
    host_route[++host_route_segment_count].x_mm = 0.0f;
    host_route[host_route_segment_count].y_mm =
        -target_sign * target_radius;
    host_route[host_route_segment_count].z_mm = clearance;
    for (uint8_t step = 1u; step <= arc_steps; ++step) {
        float angle = (90.0f - step * ARM_REAR_BYPASS_ARC_STEP_DEG) *
                      HOST_PI / 180.0f;
        host_route[++host_route_segment_count].x_mm =
            target_sign * target_radius * cosf(angle);
        host_route[host_route_segment_count].y_mm =
            -target_sign * target_radius * sinf(angle);
        host_route[host_route_segment_count].z_mm = clearance;
    }
    host_route[++host_route_segment_count] = target;
    return host_route_segment_count;
}

static int solve_route(uint8_t segment_count, float pitch_deg,
                       const float start_q[3], const char *csv_name,
                       float final_q[3])
{
    float seed[3] = {start_q[0], start_q[1], start_q[2]};
    uint32_t total_samples = 1u;

    for (uint8_t segment = 0u; segment < segment_count; ++segment) {
        Arm_Path_Plan_Result_s plan_result;
        const char *segment_csv = segment + 1u == segment_count ?
            csv_name : NULL;
        int result = solve_segment(
            &host_route[segment], &host_route[segment + 1u], pitch_deg,
            ARM_CARTESIAN_SAFETY_NORMAL, seed, segment_csv, seed,
            &plan_result);

        if (result != 0) return result;
        total_samples += plan_result.interval_count;
    }
    printf("segments=%u samples=%lu PASS final q=(%.3f, %.3f, %.3f)\n",
           segment_count, (unsigned long)total_samples,
           seed[0], seed[1], seed[2]);
    if (final_q != NULL) memcpy(final_q, seed, sizeof(seed));
    return 0;
}

static uint8_t staging_pose_safe(const float q_deg[3],
                                 float relative_pitch_deg,
                                 Arm_Position_s *center)
{
    float absolute_pitch_deg =
        q_deg[ARM_JOINT_SHOULDER] +
        (-180.0f - q_deg[ARM_JOINT_ELBOW]) +
        relative_pitch_deg;

    if (!ArmJointPoseWithinSoftLimits(q_deg) ||
        !ArmAutoPoseIsSafe(q_deg) ||
        !pitch_safe(q_deg, absolute_pitch_deg) ||
        !ArmForwardKinematicsToolCenter(q_deg, absolute_pitch_deg,
                                        center)) {
        return 0u;
    }
    if (fabsf(wrap180(q_deg[ARM_JOINT_BASE_YAW])) <=
            ARM_FRONT_BARRIER_BASE_Q1_ABS_MAX_DEG +
                ARM_LIMIT_TOLERANCE_DEG &&
        center->x_mm > ARM_FRONT_BARRIER_TOOL_X_MARGIN_MM &&
        q_deg[ARM_JOINT_SHOULDER] >
            ARM_FRONT_BARRIER_SHOULDER_Q2_MAX_DEG) {
        return 0u;
    }
    if (center->x_mm < ARM_REAR_ZONE_X_BOUNDARY_MM -
                           ARM_REAR_ZONE_X_MARGIN_MM &&
        center->z_mm < ARM_REAR_ZONE_MIN_TOOL_Z_MM) {
        return 0u;
    }
    return 1u;
}

/* Joint placement may intentionally rotate q1 beyond the automatic +/-90deg
 * Cartesian range. Keep the remaining production joint-path safety checks. */
static uint8_t transfer_joint_pose_safe(const float q_deg[3],
                                        float relative_pitch_deg,
                                        Arm_Position_s *center)
{
    float absolute_pitch_deg =
        q_deg[ARM_JOINT_SHOULDER] +
        (-180.0f - q_deg[ARM_JOINT_ELBOW]) +
        relative_pitch_deg;

    if (!ArmJointPoseWithinSoftLimits(q_deg) ||
        !pitch_safe(q_deg, absolute_pitch_deg) ||
        !ArmForwardKinematicsToolCenter(q_deg, absolute_pitch_deg,
                                        center)) {
        return 0u;
    }
    if (fabsf(wrap180(q_deg[ARM_JOINT_BASE_YAW])) <=
            ARM_FRONT_BARRIER_BASE_Q1_ABS_MAX_DEG +
                ARM_LIMIT_TOLERANCE_DEG &&
        center->x_mm > ARM_FRONT_BARRIER_TOOL_X_MARGIN_MM &&
        q_deg[ARM_JOINT_SHOULDER] >
            ARM_FRONT_BARRIER_SHOULDER_Q2_MAX_DEG) {
        return 0u;
    }
    if (center->x_mm < ARM_REAR_ZONE_X_BOUNDARY_MM -
                           ARM_REAR_ZONE_X_MARGIN_MM &&
        center->z_mm < ARM_REAR_ZONE_MIN_TOOL_Z_MM) {
        return 0u;
    }
    return 1u;
}

static int verify_ac_transfer_segment(
    const float left_start_q_deg[3], const float right_start_q_deg[3],
    const float left_target_q_deg[3], const float right_target_q_deg[3],
    float left_relative_pitch_deg, float right_relative_pitch_deg,
    uint8_t require_z_raise, float *max_abs_y_mm,
    float *waypoint_z_mm, float *z_raise_mm)
{
    float max_delta_deg = 0.0f;
    float left_start_z_mm = 0.0f;
    float right_start_z_mm = 0.0f;
    float left_previous_z_mm = 0.0f;
    float right_previous_z_mm = 0.0f;
    uint16_t intervals;

    for (uint8_t joint = 0u; joint < 3u; ++joint) {
        float delta_deg = joint == ARM_JOINT_BASE_YAW ?
            fabsf(wrap180(left_target_q_deg[joint] -
                          left_start_q_deg[joint])) :
            fabsf(left_target_q_deg[joint] - left_start_q_deg[joint]);

        max_delta_deg = fmaxf(max_delta_deg, delta_deg);
    }
    intervals = (uint16_t)ceilf(max_delta_deg);
    if (intervals < 1u) intervals = 1u;

    for (uint16_t i = 0u; i <= intervals; ++i) {
        float ratio = (float)i / (float)intervals;
        float left_q_deg[3];
        float right_q_deg[3];
        Arm_Position_s left_center;
        Arm_Position_s right_center;

        left_q_deg[0] = left_start_q_deg[0] + ratio *
            wrap180(left_target_q_deg[0] - left_start_q_deg[0]);
        right_q_deg[0] = right_start_q_deg[0] + ratio *
            wrap180(right_target_q_deg[0] - right_start_q_deg[0]);
        for (uint8_t joint = 1u; joint < 3u; ++joint) {
            left_q_deg[joint] = left_start_q_deg[joint] + ratio *
                (left_target_q_deg[joint] - left_start_q_deg[joint]);
            right_q_deg[joint] = right_start_q_deg[joint] + ratio *
                (right_target_q_deg[joint] - right_start_q_deg[joint]);
        }
        if (!transfer_joint_pose_safe(left_q_deg, left_relative_pitch_deg,
                                      &left_center) ||
            !transfer_joint_pose_safe(right_q_deg, right_relative_pitch_deg,
                                      &right_center)) {
            fprintf(stderr,
                    "FAIL AC transfer safety sample=%u "
                    "leftQ=(%.3f,%.3f,%.3f) "
                    "rightQ=(%.3f,%.3f,%.3f)\n",
                    i, left_q_deg[0], left_q_deg[1], left_q_deg[2],
                    right_q_deg[0], right_q_deg[1], right_q_deg[2]);
            return 40;
        }
        if (fabsf(left_center.y_mm) >
                APP_ARM_POSTURE_TEST_TRANSFER_PATH_Y_MAX_MM + 0.001f ||
            fabsf(right_center.y_mm) >
                APP_ARM_POSTURE_TEST_TRANSFER_PATH_Y_MAX_MM + 0.001f) {
            fprintf(stderr,
                    "FAIL AC transfer Y sample=%u leftY=%.3f rightY=%.3f "
                    "limit=%.3f\n",
                    i, left_center.y_mm, right_center.y_mm,
                    APP_ARM_POSTURE_TEST_TRANSFER_PATH_Y_MAX_MM);
            return 41;
        }
        if (i == 0u) {
            left_start_z_mm = left_center.z_mm;
            right_start_z_mm = right_center.z_mm;
        } else if (require_z_raise != 0u &&
                   (left_center.z_mm < left_previous_z_mm - 0.01f ||
                    right_center.z_mm < right_previous_z_mm - 0.01f)) {
            fprintf(stderr,
                    "FAIL AC transfer Z dip sample=%u "
                    "leftZ=%.3f/%.3f rightZ=%.3f/%.3f\n",
                    i, left_previous_z_mm, left_center.z_mm,
                    right_previous_z_mm, right_center.z_mm);
            return 45;
        }
        left_previous_z_mm = left_center.z_mm;
        right_previous_z_mm = right_center.z_mm;
        if (fabsf(left_center.x_mm - right_center.x_mm) > 0.001f ||
            fabsf(left_center.y_mm + right_center.y_mm) > 0.001f ||
            fabsf(left_center.z_mm - right_center.z_mm) > 0.001f) {
            fprintf(stderr,
                    "FAIL AC transfer mirror sample=%u "
                    "left=(%.3f,%.3f,%.3f) right=(%.3f,%.3f,%.3f)\n",
                    i, left_center.x_mm, left_center.y_mm,
                    left_center.z_mm, right_center.x_mm,
                    right_center.y_mm, right_center.z_mm);
            return 42;
        }
        *max_abs_y_mm = fmaxf(*max_abs_y_mm,
                              fabsf(left_center.y_mm));
        *max_abs_y_mm = fmaxf(*max_abs_y_mm,
                              fabsf(right_center.y_mm));
        if (i == intervals && require_z_raise != 0u) {
            float left_raise_mm = left_center.z_mm - left_start_z_mm;
            float right_raise_mm = right_center.z_mm - right_start_z_mm;

            if (left_raise_mm +
                    APP_ARM_POSTURE_TEST_TRANSFER_Z_TOLERANCE_MM <
                    APP_ARM_POSTURE_TEST_TRANSFER_Z_RAISE_MM ||
                right_raise_mm +
                    APP_ARM_POSTURE_TEST_TRANSFER_Z_TOLERANCE_MM <
                    APP_ARM_POSTURE_TEST_TRANSFER_Z_RAISE_MM) {
                fprintf(stderr,
                        "FAIL AC transfer Z raise left=%.3f right=%.3f "
                        "minimum=%.3f tolerance=%.3f\n",
                        left_raise_mm, right_raise_mm,
                        APP_ARM_POSTURE_TEST_TRANSFER_Z_RAISE_MM,
                        APP_ARM_POSTURE_TEST_TRANSFER_Z_TOLERANCE_MM);
                return 46;
            }
            if (waypoint_z_mm != NULL) {
                *waypoint_z_mm = left_center.z_mm;
            }
            if (z_raise_mm != NULL) {
                *z_raise_mm = left_raise_mm;
            }
        }
    }
    return 0;
}

static int verify_ac_post_grip_transfer(void)
{
    const Arm_Position_s left_center = {
        APP_ARM_POSTURE_TEST_ADVANCE_X_MM,
        APP_ARM_POSTURE_TEST_LEFT_ADVANCE_Y_MM,
        APP_ARM_POSTURE_TEST_ADVANCE_Z_MM
    };
    const Arm_Position_s right_center = {
        APP_ARM_POSTURE_TEST_ADVANCE_X_MM,
        APP_ARM_POSTURE_TEST_RIGHT_ADVANCE_Y_MM,
        APP_ARM_POSTURE_TEST_ADVANCE_Z_MM
    };
    const float left_seed_q_deg[3] = {90.0f, 15.0f, -85.0f};
    const float right_seed_q_deg[3] = {-90.0f, 15.0f, -85.0f};
    const float left_waypoint_q_deg[3] = {
        APP_FRUIT_A_LEFT_SAFE_Q1_DEG,
        APP_ARM_POSTURE_TEST_TRANSFER_WAYPOINT_Q2_DEG,
        APP_ARM_POSTURE_TEST_TRANSFER_WAYPOINT_Q3_DEG
    };
    const float right_waypoint_q_deg[3] = {
        APP_FRUIT_A_RIGHT_SAFE_Q1_DEG,
        APP_ARM_POSTURE_TEST_TRANSFER_WAYPOINT_Q2_DEG,
        APP_ARM_POSTURE_TEST_TRANSFER_WAYPOINT_Q3_DEG
    };
    const float left_release_q_deg[3] = {
        APP_FRUIT_A_LEFT_PLACE_Q1_DEG,
        APP_FRUIT_A_RELEASE_Q2_DEG,
        APP_FRUIT_A_RELEASE_Q3_DEG
    };
    const float right_release_q_deg[3] = {
        APP_FRUIT_A_RIGHT_PLACE_Q1_DEG,
        APP_FRUIT_A_RELEASE_Q2_DEG,
        APP_FRUIT_A_RELEASE_Q3_DEG
    };
    Arm_Tool_Center_IK_Result_s left_ik;
    Arm_Tool_Center_IK_Result_s right_ik;
    float left_relative_pitch_deg;
    float right_relative_pitch_deg;
    float max_abs_y_mm = 0.0f;
    float waypoint_z_mm = 0.0f;
    float z_raise_mm = 0.0f;
    int result;

    memset(&left_ik, 0, sizeof(left_ik));
    memset(&right_ik, 0, sizeof(right_ik));
    if (ArmInverseKinematicsToolCenter(
            &left_center, APP_ARM_POSTURE_TEST_TOOL_PITCH_DEG,
            left_seed_q_deg, &left_ik) != ARM_IK_OK ||
        ArmInverseKinematicsToolCenter(
            &right_center, APP_ARM_POSTURE_TEST_TOOL_PITCH_DEG,
            right_seed_q_deg, &right_ik) != ARM_IK_OK) {
        fprintf(stderr, "FAIL AC transfer start IK\n");
        return 43;
    }
    left_relative_pitch_deg = APP_ARM_POSTURE_TEST_TOOL_PITCH_DEG -
        (left_ik.q_deg[1] + (-180.0f - left_ik.q_deg[2]));
    right_relative_pitch_deg = APP_ARM_POSTURE_TEST_TOOL_PITCH_DEG -
        (right_ik.q_deg[1] + (-180.0f - right_ik.q_deg[2]));
    if (fabsf(left_ik.q_deg[0] + right_ik.q_deg[0]) > 0.001f ||
        fabsf(left_ik.q_deg[1] - right_ik.q_deg[1]) > 0.001f ||
        fabsf(left_ik.q_deg[2] - right_ik.q_deg[2]) > 0.001f ||
        fabsf(left_relative_pitch_deg - right_relative_pitch_deg) >
            0.001f) {
        fprintf(stderr, "FAIL AC transfer start is not mirrored\n");
        return 44;
    }
    result = verify_ac_transfer_segment(
        left_ik.q_deg, right_ik.q_deg,
        left_waypoint_q_deg, right_waypoint_q_deg,
        left_relative_pitch_deg, right_relative_pitch_deg,
        1u, &max_abs_y_mm, &waypoint_z_mm, &z_raise_mm);
    if (result != 0) return result;
    result = verify_ac_transfer_segment(
        left_waypoint_q_deg, right_waypoint_q_deg,
        left_release_q_deg, right_release_q_deg,
        left_relative_pitch_deg, right_relative_pitch_deg,
        0u, &max_abs_y_mm, NULL, NULL);
    if (result != 0) return result;

    printf("PASS AC post-grip transfer: ID1 locked until "
           "waypoint=(+/-90.0,%.1f,%.1f), waypointZ=%.3f raiseZ=%.3f "
           "rear=(+/-178.0,%.1f,%.1f) maxAbsY=%.3f limit=%.1f\n",
           APP_ARM_POSTURE_TEST_TRANSFER_WAYPOINT_Q2_DEG,
           APP_ARM_POSTURE_TEST_TRANSFER_WAYPOINT_Q3_DEG,
           waypoint_z_mm, z_raise_mm,
           APP_FRUIT_A_RELEASE_Q2_DEG,
           APP_FRUIT_A_RELEASE_Q3_DEG,
           max_abs_y_mm, APP_ARM_POSTURE_TEST_TRANSFER_PATH_Y_MAX_MM);
    return 0;
}

static int verify_bd_observation_side(
    float base_q1_deg, const Arm_Position_s *target_center,
    const char *side_name, const char *csv_name)
{
    const float start_q_deg[3] = {0.0f, 90.0f, -60.0f};
    const float staging_q_deg[3] = {
        base_q1_deg,
        APP_ARM_BD_OBSERVATION_STAGING_Q2_DEG,
        APP_ARM_BD_OBSERVATION_STAGING_Q3_DEG
    };
    float relative_pitch_deg = APP_ARM_BD_OBSERVATION_TOOL_PITCH_DEG -
        (staging_q_deg[ARM_JOINT_SHOULDER] +
         (-180.0f - staging_q_deg[ARM_JOINT_ELBOW]));
    float max_delta_deg = 0.0f;
    float max_abs_y_mm = 0.0f;
    float min_z_mm = HOST_INF;
    Arm_Position_s staging_center = {0};
    float final_q_deg[3];
    uint16_t joint_intervals;
    uint16_t linear_intervals;
    int result;

    if (target_center == NULL || side_name == NULL || csv_name == NULL) {
        return 30;
    }

    for (uint8_t joint = 0u; joint < 3u; ++joint) {
        float delta_deg = joint == ARM_JOINT_BASE_YAW ?
            fabsf(wrap180(staging_q_deg[joint] - start_q_deg[joint])) :
            fabsf(staging_q_deg[joint] - start_q_deg[joint]);

        max_delta_deg = fmaxf(max_delta_deg, delta_deg);
    }
    joint_intervals = (uint16_t)ceilf(max_delta_deg);
    if (joint_intervals < 1u) joint_intervals = 1u;
    for (uint16_t i = 0u; i <= joint_intervals; ++i) {
        float ratio = (float)i / (float)joint_intervals;
        float q_deg[3] = {
            start_q_deg[0] + ratio *
                wrap180(staging_q_deg[0] - start_q_deg[0]),
            start_q_deg[1] + ratio *
                (staging_q_deg[1] - start_q_deg[1]),
            start_q_deg[2] + ratio *
                (staging_q_deg[2] - start_q_deg[2])
        };
        Arm_Position_s center;

        if (!staging_pose_safe(q_deg, relative_pitch_deg, &center)) {
            fprintf(stderr,
                    "FAIL BD staging safety sample=%u q=(%.3f,%.3f,%.3f)\n",
                    i, q_deg[0], q_deg[1], q_deg[2]);
            return 31;
        }
        if (fabsf(center.y_mm) >
                APP_ARM_BD_OBSERVATION_PATH_Y_MAX_MM + 0.001f) {
            fprintf(stderr,
                    "FAIL BD staging Y sample=%u y=%.3f limit=%.3f\n",
                    i, center.y_mm,
                    APP_ARM_BD_OBSERVATION_PATH_Y_MAX_MM);
            return 32;
        }
        max_abs_y_mm = fmaxf(max_abs_y_mm, fabsf(center.y_mm));
        min_z_mm = fminf(min_z_mm, center.z_mm);
        staging_center = center;
    }

    linear_intervals = (uint16_t)ceilf(
        hypotf(target_center->x_mm - staging_center.x_mm,
               target_center->y_mm - staging_center.y_mm) /
        ARM_LINEAR_SAMPLE_SPACING_MM);
    linear_intervals = (uint16_t)fmaxf(
        (float)linear_intervals,
        ceilf(fabsf(target_center->z_mm - staging_center.z_mm) /
              ARM_LINEAR_SAMPLE_SPACING_MM));
    if (linear_intervals < 1u) linear_intervals = 1u;
    for (uint16_t i = 0u; i <= linear_intervals; ++i) {
        float ratio = (float)i / (float)linear_intervals;
        Arm_Position_s center = {
            staging_center.x_mm + ratio *
                (target_center->x_mm - staging_center.x_mm),
            staging_center.y_mm + ratio *
                (target_center->y_mm - staging_center.y_mm),
            staging_center.z_mm + ratio *
                (target_center->z_mm - staging_center.z_mm)
        };

        if (fabsf(center.y_mm) >
                APP_ARM_BD_OBSERVATION_PATH_Y_MAX_MM + 0.001f) {
            fprintf(stderr,
                    "FAIL BD linear Y sample=%u y=%.3f limit=%.3f\n",
                    i, center.y_mm,
                    APP_ARM_BD_OBSERVATION_PATH_Y_MAX_MM);
            return 34;
        }
        max_abs_y_mm = fmaxf(max_abs_y_mm, fabsf(center.y_mm));
        min_z_mm = fminf(min_z_mm, center.z_mm);
    }
    host_route[0] = staging_center;
    host_route[1] = *target_center;
    result = solve_route(1u, APP_ARM_BD_OBSERVATION_TOOL_PITCH_DEG,
                         staging_q_deg, csv_name, final_q_deg);
    if (result != 0 ||
        fabsf(wrap180(final_q_deg[ARM_JOINT_BASE_YAW] -
                      base_q1_deg)) > 0.01f) {
        fprintf(stderr,
                "FAIL BD %s linear path result=%d final_q1=%.3f\n",
                side_name, result, final_q_deg[ARM_JOINT_BASE_YAW]);
        return 35;
    }
    printf("PASS BD %s coordinated staging: q=(%.1f,%.1f,%.1f) "
           "relative_pitch=%.1f minZ=%.3f maxAbsY=%.3f limit=%.1f\n",
           side_name, staging_q_deg[0], staging_q_deg[1],
           staging_q_deg[2], relative_pitch_deg, min_z_mm, max_abs_y_mm,
           APP_ARM_BD_OBSERVATION_PATH_Y_MAX_MM);
    return 0;
}

static int verify_bd_observation_transition(void)
{
    const Arm_Position_s left_target = {
        APP_ARM_BD_OBSERVATION_LEFT_X_MM,
        APP_ARM_BD_OBSERVATION_LEFT_Y_MM,
        APP_ARM_BD_OBSERVATION_LEFT_Z_MM
    };
    const Arm_Position_s right_target = {
        APP_ARM_BD_OBSERVATION_RIGHT_X_MM,
        APP_ARM_BD_OBSERVATION_RIGHT_Y_MM,
        APP_ARM_BD_OBSERVATION_RIGHT_Z_MM
    };
    int result;

    if (fabsf(APP_ARM_BD_OBSERVATION_LEFT_BASE_Q1_DEG +
              APP_ARM_BD_OBSERVATION_RIGHT_BASE_Q1_DEG) > 0.001f ||
        fabsf(left_target.x_mm - right_target.x_mm) > 0.001f ||
        fabsf(left_target.y_mm + right_target.y_mm) > 0.001f ||
        fabsf(left_target.z_mm - right_target.z_mm) > 0.001f) {
        fprintf(stderr, "FAIL BD left/right configuration is not mirrored\n");
        return 36;
    }
    result = verify_bd_observation_side(
        APP_ARM_BD_OBSERVATION_LEFT_BASE_Q1_DEG, &left_target,
        "LEFT", "bd_observation_left_candidates.csv");
    if (result != 0) return result;
    return verify_bd_observation_side(
        APP_ARM_BD_OBSERVATION_RIGHT_BASE_Q1_DEG, &right_target,
        "RIGHT", "bd_observation_right_candidates.csv");
}

static int verify_post_place_cycle_transition(void)
{
    const float start_q_deg[3] = {0.0f, 120.0f, -80.0f};
    const float target_q1_deg[2] = {
        APP_ARM_PICK_BASE_AIM_MAX_ABS_Q1_DEG,
        -APP_ARM_PICK_BASE_AIM_MAX_ABS_Q1_DEG
    };
    const float approach_y_mm[2] = {
        APP_ARM_POSTURE_TEST_LEFT_Y_MM,
        APP_ARM_POSTURE_TEST_RIGHT_Y_MM
    };
    const float advance_y_mm[2] = {
        APP_ARM_POSTURE_TEST_LEFT_ADVANCE_Y_MM,
        APP_ARM_POSTURE_TEST_RIGHT_ADVANCE_Y_MM
    };
    const char *approach_csv[2] = {
        "staging_left_approach_candidates.csv",
        "staging_right_approach_candidates.csv"
    };
    const char *advance_csv[2] = {
        "staging_left_advance_candidates.csv",
        "staging_right_advance_candidates.csv"
    };
    uint16_t joint_intervals = (uint16_t)ceilf(
        APP_ARM_PICK_BASE_AIM_MAX_ABS_Q1_DEG);

    for (uint16_t joint_sample = 0u;
         joint_sample <= joint_intervals; ++joint_sample) {
        float ratio = (float)joint_sample / (float)joint_intervals;
        Arm_Position_s mirrored_center[2];

        for (uint8_t side = 0u; side < 2u; ++side) {
            float q_deg[3];

            q_deg[ARM_JOINT_BASE_YAW] = start_q_deg[ARM_JOINT_BASE_YAW] +
                ratio * (target_q1_deg[side] -
                         start_q_deg[ARM_JOINT_BASE_YAW]);
            q_deg[ARM_JOINT_SHOULDER] =
                start_q_deg[ARM_JOINT_SHOULDER] + ratio *
                (APP_ARM_PICK_STAGING_Q2_DEG -
                 start_q_deg[ARM_JOINT_SHOULDER]);
            q_deg[ARM_JOINT_ELBOW] = start_q_deg[ARM_JOINT_ELBOW] +
                ratio * (APP_ARM_PICK_STAGING_Q3_DEG -
                         start_q_deg[ARM_JOINT_ELBOW]);
            if (!staging_pose_safe(
                    q_deg, APP_ARM_PICK_STAGING_TOOL_RELATIVE_PITCH_DEG,
                    &mirrored_center[side])) {
                fprintf(stderr,
                        "FAIL staging joint path side=%u sample=%u "
                        "q=(%.3f,%.3f,%.3f)\n",
                        side, joint_sample, q_deg[0], q_deg[1], q_deg[2]);
                return 26;
            }
        }
        if (fabsf(mirrored_center[0].x_mm -
                  mirrored_center[1].x_mm) > 0.001f ||
            fabsf(mirrored_center[0].y_mm +
                  mirrored_center[1].y_mm) > 0.001f ||
            fabsf(mirrored_center[0].z_mm -
                  mirrored_center[1].z_mm) > 0.001f) {
            fprintf(stderr,
                    "FAIL staging mirror sample=%u left=(%.3f,%.3f,%.3f) "
                    "right=(%.3f,%.3f,%.3f)\n",
                    joint_sample, mirrored_center[0].x_mm,
                    mirrored_center[0].y_mm, mirrored_center[0].z_mm,
                    mirrored_center[1].x_mm, mirrored_center[1].y_mm,
                    mirrored_center[1].z_mm);
            return 27;
        }
    }

    for (uint8_t side = 0u; side < 2u; ++side) {
        float staging_q_deg[3] = {
            target_q1_deg[side], APP_ARM_PICK_STAGING_Q2_DEG,
            APP_ARM_PICK_STAGING_Q3_DEG
        };
        float approach_q_deg[3];
        Arm_Position_s staging_center;
        Arm_Position_s approach_center = {
            APP_ARM_POSTURE_TEST_X_MM, approach_y_mm[side],
            APP_ARM_POSTURE_TEST_Z_MM
        };
        Arm_Position_s advance_center = {
            APP_ARM_POSTURE_TEST_ADVANCE_X_MM, advance_y_mm[side],
            APP_ARM_POSTURE_TEST_ADVANCE_Z_MM
        };
        int result;

        if (!ArmForwardKinematicsToolCenter(
                staging_q_deg, APP_ARM_POSTURE_TEST_TOOL_PITCH_DEG,
                &staging_center)) {
            return 28;
        }
        host_route[0] = staging_center;
        host_route[1] = approach_center;
        result = solve_route(1u, APP_ARM_POSTURE_TEST_TOOL_PITCH_DEG,
                             staging_q_deg, approach_csv[side],
                             approach_q_deg);
        if (result != 0) {
            fprintf(stderr, "FAIL staging approach side=%u result=%d\n",
                    side, result);
            return 29;
        }

        host_route[0] = approach_center;
        host_route[1] = advance_center;
        result = solve_route(1u, APP_ARM_POSTURE_TEST_TOOL_PITCH_DEG,
                             approach_q_deg, advance_csv[side], NULL);
        if (result != 0) {
            fprintf(stderr, "FAIL staging advance side=%u result=%d\n",
                    side, result);
            return 30;
        }
    }
    printf("PASS post-place staging and mirrored pick transitions\n");
    return 0;
}

static int verify_observation_to_high_pick_staging(void)
{
    const Arm_Position_s observation_center[2] = {
        {APP_ARM_AC_OBSERVATION_LEFT_X_MM,
         APP_ARM_AC_OBSERVATION_LEFT_Y_MM,
         APP_ARM_AC_OBSERVATION_LEFT_Z_MM},
        {APP_ARM_AC_OBSERVATION_RIGHT_X_MM,
         APP_ARM_AC_OBSERVATION_RIGHT_Y_MM,
         APP_ARM_AC_OBSERVATION_RIGHT_Z_MM}
    };
    const float target_q1_deg[2] = {
        APP_ARM_PICK_BASE_AIM_MAX_ABS_Q1_DEG,
        -APP_ARM_PICK_BASE_AIM_MAX_ABS_Q1_DEG
    };
    Arm_Position_s final_center[2];

    for (uint8_t side = 0u; side < 2u; ++side) {
        const float seed_q_deg[3] = {
            side == 0u ? 90.0f : -90.0f, 123.0f, -84.0f
        };
        Arm_Tool_Center_IK_Result_s observation_ik;
        float segment_q_deg[3][3];

        memset(&observation_ik, 0, sizeof(observation_ik));
        if (ArmInverseKinematicsToolCenter(
                &observation_center[side],
                APP_ARM_AC_OBSERVATION_TOOL_PITCH_DEG,
                seed_q_deg, &observation_ik) != ARM_IK_OK) {
            fprintf(stderr, "FAIL high staging observation IK side=%u\n",
                    side);
            return 77;
        }
        memcpy(segment_q_deg[0], observation_ik.q_deg,
               sizeof(segment_q_deg[0]));
        segment_q_deg[1][0] = observation_ik.q_deg[0];
        segment_q_deg[1][1] = APP_ARM_PICK_STAGING_Q2_DEG;
        segment_q_deg[1][2] = APP_ARM_PICK_STAGING_Q3_DEG;
        segment_q_deg[2][0] = target_q1_deg[side];
        segment_q_deg[2][1] = APP_ARM_PICK_STAGING_Q2_DEG;
        segment_q_deg[2][2] = APP_ARM_PICK_STAGING_Q3_DEG;

        for (uint8_t segment = 0u; segment < 2u; ++segment) {
            float max_delta_deg = 0.0f;
            uint16_t intervals;

            for (uint8_t joint = 0u; joint < 3u; ++joint) {
                float delta_deg = joint == ARM_JOINT_BASE_YAW ?
                    fabsf(wrap180(segment_q_deg[segment + 1u][joint] -
                                  segment_q_deg[segment][joint])) :
                    fabsf(segment_q_deg[segment + 1u][joint] -
                          segment_q_deg[segment][joint]);
                max_delta_deg = fmaxf(max_delta_deg, delta_deg);
            }
            intervals = (uint16_t)ceilf(max_delta_deg);
            if (intervals < 1u) intervals = 1u;
            for (uint16_t sample = 0u; sample <= intervals; ++sample) {
                float ratio = (float)sample / (float)intervals;
                float q_deg[3];
                Arm_Position_s center;

                q_deg[0] = segment_q_deg[segment][0] + ratio *
                    wrap180(segment_q_deg[segment + 1u][0] -
                            segment_q_deg[segment][0]);
                for (uint8_t joint = 1u; joint < 3u; ++joint) {
                    q_deg[joint] = segment_q_deg[segment][joint] + ratio *
                        (segment_q_deg[segment + 1u][joint] -
                         segment_q_deg[segment][joint]);
                }
                if (!staging_pose_safe(
                        q_deg, APP_ARM_PICK_STAGING_TOOL_RELATIVE_PITCH_DEG,
                        &center)) {
                    fprintf(stderr,
                            "FAIL observation-high staging side=%u segment=%u "
                            "sample=%u q=(%.3f,%.3f,%.3f)\n",
                            side, segment, sample,
                            q_deg[0], q_deg[1], q_deg[2]);
                    return 78;
                }
                if (segment == 1u && sample == intervals) {
                    final_center[side] = center;
                }
            }
        }
        if (final_center[side].z_mm <
                observation_center[side].z_mm + 50.0f) {
            fprintf(stderr,
                    "FAIL high staging raise side=%u observationZ=%.3f "
                    "stagingZ=%.3f\n",
                    side, observation_center[side].z_mm,
                    final_center[side].z_mm);
            return 79;
        }
    }
    if (fabsf(final_center[0].x_mm - final_center[1].x_mm) > 0.001f ||
        fabsf(final_center[0].y_mm + final_center[1].y_mm) > 0.001f ||
        fabsf(final_center[0].z_mm - final_center[1].z_mm) > 0.001f) {
        fprintf(stderr, "FAIL high staging is not mirrored\n");
        return 80;
    }
    printf("PASS observation to high pick staging: q2=%.1f q3=%.1f "
           "centerZ=%.3f raiseZ=%.3f\n",
           APP_ARM_PICK_STAGING_Q2_DEG,
           APP_ARM_PICK_STAGING_Q3_DEG,
           final_center[0].z_mm,
           final_center[0].z_mm - observation_center[0].z_mm);
    return 0;
}

static int verify_world_y_mirror(void)
{
    const float pitch_deg = -5.0f;
    Arm_Position_s wrist = {100.0f, 200.0f, 300.0f};
    Arm_Position_s center;
    Arm_Position_s recovered;
    Arm_Tool_Center_IK_Result_s result;
    float left_seed[3] = {90.0f, 80.0f, -90.0f};
    float right_seed[3] = {-90.0f, 80.0f, -90.0f};
    float kinematics_error_mm = 0.0f;
    float min_q2_deg = 1000.0f;
    float max_relative_pitch_deg = -1000.0f;
    float min_pitch_position = 1000.0f;

    if (!ArmKinematicsSelfTest(&kinematics_error_mm) ||
        kinematics_error_mm > 0.001f) {
        fprintf(stderr, "FAIL kinematics self-test error=%.6f mm\n",
                kinematics_error_mm);
        return 20;
    }
    if (!ArmToolGetCenterFromWrist(&wrist, 90.0f, 0.0f, &center) ||
        fabsf(center.y_mm -
              (wrist.y_mm + ARM_TOOL_PITCH_AXIS_TO_CENTER_MM)) > 0.001f ||
        !ArmToolGetWristFromCenter(&center, 90.0f, 0.0f, &recovered) ||
        fabsf(recovered.x_mm - wrist.x_mm) > 0.001f ||
        fabsf(recovered.y_mm - wrist.y_mm) > 0.001f ||
        fabsf(recovered.z_mm - wrist.z_mm) > 0.001f) {
        fprintf(stderr, "FAIL tool-center Y mirror round trip\n");
        return 21;
    }

    for (uint16_t offset_mm = 0u; offset_mm <= 140u; ++offset_mm) {
        Arm_Position_s left_target = {
            0.0f, 340.0f + (float)offset_mm, -150.0f
        };
        Arm_Position_s right_target = {
            left_target.x_mm, -left_target.y_mm, left_target.z_mm
        };
        Arm_Tool_Center_IK_Result_s left_result;
        Arm_Tool_Center_IK_Result_s right_result;
        float left_small_link_pitch_deg;
        float right_small_link_pitch_deg;
        float left_relative_pitch_deg;
        float right_relative_pitch_deg;
        float left_pitch_position;
        float right_pitch_position;

        memset(&left_result, 0, sizeof(left_result));
        memset(&right_result, 0, sizeof(right_result));
        if (ArmInverseKinematicsToolCenter(
                &left_target, pitch_deg, left_seed, &left_result) !=
                ARM_IK_OK ||
            ArmInverseKinematicsToolCenter(
                &right_target, pitch_deg, right_seed, &right_result) !=
                ARM_IK_OK ||
            fabsf(left_result.q_deg[0] - 90.0f) > 0.01f ||
            fabsf(right_result.q_deg[0] + 90.0f) > 0.01f ||
            fabsf(left_result.q_deg[0] + right_result.q_deg[0]) > 0.001f ||
            fabsf(left_result.q_deg[1] - right_result.q_deg[1]) > 0.001f ||
            fabsf(left_result.q_deg[2] - right_result.q_deg[2]) > 0.001f ||
            !ArmAutoPoseIsSafe(left_result.q_deg) ||
            !ArmAutoPoseIsSafe(right_result.q_deg)) {
            fprintf(stderr,
                    "FAIL mirrored path |Y|=%.1f left=(%d,%.3f,%.3f,%.3f) "
                    "right=(%d,%.3f,%.3f,%.3f)\n",
                    left_target.y_mm, (int)left_result.status,
                    left_result.q_deg[0], left_result.q_deg[1],
                    left_result.q_deg[2], (int)right_result.status,
                    right_result.q_deg[0], right_result.q_deg[1],
                    right_result.q_deg[2]);
            return 22;
        }
        left_small_link_pitch_deg =
            left_result.q_deg[1] + (-180.0f - left_result.q_deg[2]);
        right_small_link_pitch_deg =
            right_result.q_deg[1] + (-180.0f - right_result.q_deg[2]);
        left_relative_pitch_deg = pitch_deg - left_small_link_pitch_deg;
        right_relative_pitch_deg = pitch_deg - right_small_link_pitch_deg;
        left_pitch_position = (float)ARM_TOOL_PITCH_NEUTRAL_POS +
            ARM_TOOL_PITCH_DIRECTION * left_relative_pitch_deg *
            (float)(ARM_TOOL_SERVO_POS_MAX - ARM_TOOL_SERVO_POS_MIN) /
            ARM_TOOL_SERVO_RANGE_DEG;
        right_pitch_position = (float)ARM_TOOL_PITCH_NEUTRAL_POS +
            ARM_TOOL_PITCH_DIRECTION * right_relative_pitch_deg *
            (float)(ARM_TOOL_SERVO_POS_MAX - ARM_TOOL_SERVO_POS_MIN) /
            ARM_TOOL_SERVO_RANGE_DEG;
        if (left_relative_pitch_deg < ARM_TOOL_PITCH_RELATIVE_MIN_DEG ||
            left_relative_pitch_deg > ARM_TOOL_PITCH_RELATIVE_MAX_DEG ||
            right_relative_pitch_deg < ARM_TOOL_PITCH_RELATIVE_MIN_DEG ||
            right_relative_pitch_deg > ARM_TOOL_PITCH_RELATIVE_MAX_DEG ||
            left_pitch_position < (float)ARM_TOOL_PITCH_SERVO_MIN_POS ||
            left_pitch_position > (float)ARM_TOOL_PITCH_SERVO_MAX_POS ||
            right_pitch_position < (float)ARM_TOOL_PITCH_SERVO_MIN_POS ||
            right_pitch_position > (float)ARM_TOOL_PITCH_SERVO_MAX_POS ||
            fabsf(left_relative_pitch_deg - right_relative_pitch_deg) >
                0.001f ||
            fabsf(left_pitch_position - right_pitch_position) > 0.001f) {
            fprintf(stderr,
                    "FAIL mirrored ID1 |Y|=%.1f relative=(%.3f,%.3f) "
                    "position=(%.3f,%.3f)\n",
                    left_target.y_mm, left_relative_pitch_deg,
                    right_relative_pitch_deg, left_pitch_position,
                    right_pitch_position);
            return 23;
        }
        if (left_result.q_deg[1] < min_q2_deg) {
            min_q2_deg = left_result.q_deg[1];
        }
        if (left_relative_pitch_deg > max_relative_pitch_deg) {
            max_relative_pitch_deg = left_relative_pitch_deg;
        }
        if (left_pitch_position < min_pitch_position) {
            min_pitch_position = left_pitch_position;
        }
        memcpy(left_seed, left_result.q_deg, sizeof(left_seed));
        memcpy(right_seed, right_result.q_deg, sizeof(right_seed));
    }

    {
        Arm_Position_s left = {0.0f, 400.0f, -100.0f};
        Arm_Position_s right = {0.0f, -400.0f, -100.0f};
        float left_point_seed[3] = {90.0f, 80.0f, -90.0f};
        float right_point_seed[3] = {-90.0f, 80.0f, -90.0f};

        if (ArmInverseKinematicsToolCenter(
                &left, -90.0f, left_point_seed, &result) != ARM_IK_OK ||
            fabsf(result.q_deg[0] - 90.0f) > 0.01f) {
            fprintf(stderr, "FAIL A-left point q1=%.3f\n", result.q_deg[0]);
            return 24;
        }
        if (ArmInverseKinematicsToolCenter(
                &right, -90.0f, right_point_seed, &result) != ARM_IK_OK ||
            fabsf(result.q_deg[0] + 90.0f) > 0.01f) {
            fprintf(stderr, "FAIL A-right point q1=%.3f\n", result.q_deg[0]);
            return 25;
        }
    }

    printf("PASS mirrored paths: left +Y/right -Y; min q2=%.3f "
           "max ID1 relative=%.3f min position=%.3f\n",
           min_q2_deg, max_relative_pitch_deg, min_pitch_position);
    return 0;
}

static Arm_Path_Plan_Workspace_s host_planner_workspace(void)
{
    Arm_Path_Plan_Workspace_s workspace;

    workspace.sample_q_deg = host_sample_q_deg;
    workspace.sample_progress = host_sample_progress;
    workspace.candidate_predecessor = host_candidate_predecessor;
    workspace.candidate_count = host_candidate_count;
    workspace.selected_candidate = host_selected_candidate;
    workspace.capacity = HOST_MAX_POINTS;
    return workspace;
}

static uint8_t run_advance_case_at_x(
    float side_sign, float approach_x_mm, float approach_y_abs_mm,
    Arm_Path_Advance_Result_s *result)
{
    Arm_Path_Advance_Request_s request;
    Arm_Path_Plan_Workspace_s workspace = host_planner_workspace();

    memset(&request, 0, sizeof(request));
    request.staging_q_deg[0] =
        side_sign * APP_ARM_PICK_BASE_AIM_MAX_ABS_Q1_DEG;
    request.staging_q_deg[1] = APP_ARM_PICK_STAGING_Q2_DEG;
    request.staging_q_deg[2] = APP_ARM_PICK_STAGING_Q3_DEG;
    request.approach_center_mm.x_mm = approach_x_mm;
    request.approach_center_mm.y_mm = side_sign * approach_y_abs_mm;
    request.approach_center_mm.z_mm =
        APP_ARM_AC_CLOSED_LOOP_PICK_Z_MM;
    request.tool_pitch_deg =
        APP_ARM_AC_CLOSED_LOOP_PICK_TOOL_PITCH_DEG;
    request.advance_sign = side_sign;
    request.requested_advance_mm =
        APP_ARM_AC_CLOSED_LOOP_ADVANCE_MM;
    request.sample_step_mm =
        APP_ARM_AC_CLOSED_LOOP_ADVANCE_SEARCH_STEP_MM;
    request.safety_profile = ARM_CARTESIAN_SAFETY_AC_SIDE_PICK;
    return ArmPathSelectReachableAdvance(&request, &workspace, result);
}

static uint8_t run_advance_case(
    float side_sign, float approach_y_abs_mm,
    Arm_Path_Advance_Result_s *result)
{
    return run_advance_case_at_x(
        side_sign, -160.0f, approach_y_abs_mm, result);
}

static int verify_shared_advance_selection(void)
{
    const float approach_y_abs_mm[] = {400.0f, 537.0f, 565.0f};
    const float expected_advance_mm[] = {30.0f, 29.0f, 1.0f};
    Arm_Path_Advance_Result_s first_fallback;

    memset(&first_fallback, 0, sizeof(first_fallback));
    for (uint8_t side = 0u; side < 2u; ++side) {
        float side_sign = side == 0u ? 1.0f : -1.0f;

        for (uint8_t test = 0u; test < 3u; ++test) {
            Arm_Path_Advance_Result_s result;

            if (!run_advance_case(
                    side_sign, approach_y_abs_mm[test], &result) ||
                fabsf(result.selected_advance_mm -
                       expected_advance_mm[test]) > 0.001f ||
                result.approach_failed != 0u ||
                result.plan_result.status != ARM_PATH_PLAN_OK ||
                (test == 0u && result.advance_limited != 0u) ||
                (test != 0u &&
                 (result.advance_limited == 0u ||
                  result.limiting_plan_result.status == ARM_PATH_PLAN_OK ||
                  result.limiting_plan_result.failed_sample == 0u))) {
                fprintf(stderr,
                        "FAIL shared advance side=%.0f y=%.1f ok=%u "
                        "selected=%.3f approach_failed=%u status=%d\n",
                        side_sign, approach_y_abs_mm[test],
                        result.selected_advance_mm > 0.0f ? 1u : 0u,
                        result.selected_advance_mm,
                        result.approach_failed,
                        (int)result.plan_result.status);
                return 50;
            }
            if (side == 0u && test == 1u) {
                first_fallback = result;
            }
        }

        {
            Arm_Path_Advance_Result_s zero_result;

            if (run_advance_case(side_sign, 566.0f, &zero_result) ||
                zero_result.approach_failed != 0u ||
                zero_result.plan_result.failed_sample != 1u ||
                zero_result.plan_result.reachable_distance_mm >= 1.0f) {
                fprintf(stderr,
                        "FAIL shared zero-advance side=%.0f "
                        "approach_failed=%u sample=%u reachable=%.3f\n",
                        side_sign, zero_result.approach_failed,
                        zero_result.plan_result.failed_sample,
                        zero_result.plan_result.reachable_distance_mm);
                return 51;
            }
        }
    }

    {
        Arm_Path_Advance_Result_s repeated;
        Arm_Path_Advance_Result_s approach_failure;
        Arm_Path_Advance_Result_s field_result;
        Arm_Path_Advance_Result_s near_pass;
        Arm_Path_Advance_Result_s near_reject;

        if (!run_advance_case(1.0f, 537.0f, &repeated) ||
            repeated.selected_advance_mm !=
                first_fallback.selected_advance_mm ||
            repeated.plan_result.status != first_fallback.plan_result.status ||
            repeated.limiting_plan_result.status !=
                first_fallback.limiting_plan_result.status ||
            repeated.limiting_plan_result.ik_status !=
                first_fallback.limiting_plan_result.ik_status ||
            repeated.limiting_plan_result.failed_check_mask !=
                first_fallback.limiting_plan_result.failed_check_mask ||
            repeated.limiting_plan_result.failed_sample !=
                first_fallback.limiting_plan_result.failed_sample) {
            fprintf(stderr, "FAIL shared advance repeated preflight mismatch\n");
            return 52;
        }
        if (run_advance_case(1.0f, 900.0f, &approach_failure) ||
            approach_failure.approach_failed == 0u ||
            approach_failure.plan_result.status == ARM_PATH_PLAN_OK) {
            fprintf(stderr,
                    "FAIL shared approach diagnostic failed=%u status=%d\n",
                    approach_failure.approach_failed,
                    (int)approach_failure.plan_result.status);
            return 53;
        }
        if (!run_advance_case_at_x(
                -1.0f, 97.403122f, 415.169006f, &field_result) ||
            fabsf(field_result.selected_advance_mm - 30.0f) > 0.001f ||
            field_result.approach_failed != 0u ||
            field_result.plan_result.status != ARM_PATH_PLAN_OK) {
            fprintf(stderr,
                    "FAIL field advance selected=%.3f approach=%u status=%d\n",
                    field_result.selected_advance_mm,
                    field_result.approach_failed,
                    (int)field_result.plan_result.status);
            return 74;
        }
        if (!run_advance_case_at_x(
                1.0f, 0.0f, 275.0f, &near_pass) ||
            fabsf(near_pass.selected_advance_mm - 30.0f) > 0.001f ||
            near_pass.approach_failed != 0u ||
            near_pass.plan_result.status != ARM_PATH_PLAN_OK ||
            run_advance_case_at_x(
                1.0f, 0.0f, 274.0f, &near_reject) ||
            near_reject.approach_failed == 0u ||
            near_reject.plan_result.status == ARM_PATH_PLAN_OK) {
            fprintf(stderr,
                    "FAIL near boundary pass=%.3f reject_approach=%u "
                    "reject_status=%d\n",
                    near_pass.selected_advance_mm,
                    near_reject.approach_failed,
                    (int)near_reject.plan_result.status);
            return 75;
        }
    }

    printf("PASS shared advance: mirrored 30/29/1mm, 0mm reject, field "
           "30mm, near 275/274mm, diagnostics and repeat consistency\n");
    return 0;
}

static int verify_planner_boundaries(void)
{
    const float outside_tolerance_deg = ARM_LIMIT_TOLERANCE_DEG + 0.01f;
    float q_deg[3] = {0.0f, 90.0f, -90.0f};
    uint16_t pitch_position;
    Arm_Path_Plan_Request_s request;
    Arm_Path_Plan_Result_s result;
    Arm_Path_Plan_Workspace_s workspace = host_planner_workspace();

    q_deg[0] = ARM_AUTO_Q1_MAX_DEG;
    if (!ArmAutoPoseIsSafe(q_deg)) return 54;
    q_deg[0] = ARM_AUTO_Q1_MAX_DEG + outside_tolerance_deg;
    if (ArmAutoPoseIsSafe(q_deg) ||
        !ArmAutoPoseIsSafeWithQ1Limits(
            q_deg, ARM_AC_SIDE_PICK_Q1_MIN_DEG,
            ARM_AC_SIDE_PICK_Q1_MAX_DEG)) return 55;
    q_deg[0] = ARM_AC_SIDE_PICK_Q1_MAX_DEG;
    if (!ArmAutoPoseIsSafeWithQ1Limits(
            q_deg, ARM_AC_SIDE_PICK_Q1_MIN_DEG,
            ARM_AC_SIDE_PICK_Q1_MAX_DEG)) return 56;
    q_deg[0] = ARM_AC_SIDE_PICK_Q1_MAX_DEG + outside_tolerance_deg;
    if (ArmAutoPoseIsSafeWithQ1Limits(
            q_deg, ARM_AC_SIDE_PICK_Q1_MIN_DEG,
            ARM_AC_SIDE_PICK_Q1_MAX_DEG)) return 57;
    q_deg[0] = ARM_AUTO_Q1_MIN_DEG;
    if (!ArmAutoPoseIsSafe(q_deg)) return 58;
    q_deg[0] = ARM_AUTO_Q1_MIN_DEG - outside_tolerance_deg;
    if (ArmAutoPoseIsSafe(q_deg) ||
        !ArmAutoPoseIsSafeWithQ1Limits(
            q_deg, ARM_AC_SIDE_PICK_Q1_MIN_DEG,
            ARM_AC_SIDE_PICK_Q1_MAX_DEG)) return 59;

    q_deg[0] = 0.0f;
    q_deg[1] = ARM_AUTO_Q2_MIN_DEG;
    if (!ArmAutoPoseIsSafe(q_deg)) return 60;
    q_deg[1] = ARM_AUTO_Q2_MIN_DEG - outside_tolerance_deg;
    if (ArmAutoPoseIsSafe(q_deg)) return 61;
    q_deg[1] = ARM_AUTO_Q2_MAX_DEG;
    if (!ArmAutoPoseIsSafe(q_deg)) return 62;
    q_deg[1] = ARM_AUTO_Q2_MAX_DEG + outside_tolerance_deg;
    if (ArmAutoPoseIsSafe(q_deg)) return 63;
    q_deg[1] = 90.0f;
    q_deg[2] = ARM_AUTO_Q3_MIN_DEG;
    if (!ArmAutoPoseIsSafe(q_deg)) return 64;
    q_deg[2] = ARM_AUTO_Q3_MIN_DEG - outside_tolerance_deg;
    if (ArmAutoPoseIsSafe(q_deg)) return 65;
    q_deg[2] = ARM_AUTO_Q3_MAX_DEG;
    if (!ArmAutoPoseIsSafe(q_deg)) return 66;
    q_deg[2] = ARM_AUTO_Q3_MAX_DEG + outside_tolerance_deg;
    if (ArmAutoPoseIsSafe(q_deg)) return 67;

    q_deg[2] = -90.0f;
    if (!ArmToolPitchPositionForPose(
            ARM_TOOL_PITCH_RELATIVE_MIN_DEG, 0.0f, &pitch_position) ||
        pitch_position != ARM_TOOL_PITCH_SERVO_MAX_POS ||
        !ArmToolPitchPositionForPose(
            ARM_TOOL_PITCH_RELATIVE_MAX_DEG, 0.0f, &pitch_position) ||
        pitch_position != ARM_TOOL_PITCH_SERVO_MIN_POS ||
        ArmToolPitchPositionForPose(
            ARM_TOOL_PITCH_RELATIVE_MIN_DEG - 0.01f,
            0.0f, &pitch_position) ||
        ArmToolPitchPositionForPose(
            ARM_TOOL_PITCH_RELATIVE_MAX_DEG + 0.01f,
            0.0f, &pitch_position)) return 68;

    memset(&request, 0, sizeof(request));
    request.start_q_deg[0] = 0.0f;
    request.start_q_deg[1] = 80.0f;
    request.start_q_deg[2] = -90.0f;
    request.tool_pitch_deg = -15.0f;
    request.sample_spacing_mm = 1.0f;
    request.safety_profile = ARM_CARTESIAN_SAFETY_NORMAL;
    if (!ArmForwardKinematicsToolCenter(
            request.start_q_deg, request.tool_pitch_deg,
            &request.start_center_mm)) return 69;
    request.target_center_mm = request.start_center_mm;
    request.target_center_mm.x_mm +=
        (float)(ARM_LINEAR_MAX_SAMPLES - 1u);
    if (ArmPathPlanToolCenterSegment(&request, &workspace, &result) ||
        result.status == ARM_PATH_PLAN_SAMPLE_CAPACITY) return 70;
    request.target_center_mm.x_mm += 1.0f;
    if (ArmPathPlanToolCenterSegment(&request, &workspace, &result) ||
        result.status != ARM_PATH_PLAN_SAMPLE_CAPACITY ||
        result.workspace_safety_result !=
            ARM_WORKSPACE_SAFETY_SAMPLE_CAPACITY) return 71;
    request.target_center_mm = request.start_center_mm;
    request.target_center_mm.x_mm = NAN;
    if (ArmPathPlanToolCenterSegment(&request, &workspace, &result) ||
        result.status != ARM_PATH_PLAN_INVALID) return 72;
    request.target_center_mm = request.start_center_mm;
    request.tool_pitch_deg = INFINITY;
    if (ArmPathPlanToolCenterSegment(&request, &workspace, &result) ||
        result.status != ARM_PATH_PLAN_INVALID) return 73;

    printf("PASS planner boundaries: q1 normal/AC, q2/q3, ID1, "
           "1536 samples and NaN/Inf\n");
    return 0;
}

int main(void)
{
    const float pitch_deg = -30.0f;
    Arm_Position_s pick = {250.0f, 0.0f, 200.0f};
    Arm_Position_s release = {-22.0f, 0.0f, 230.0f};
    Arm_Tool_Center_IK_Result_s start_ik;
    float seed[3] = {0.0f, 114.0f, -43.0f};
    uint8_t segment_count;
    float release_q[3];
    int result;

    result = verify_bd_observation_transition();
    if (result != 0) return result;

    result = verify_ac_post_grip_transfer();
    if (result != 0) return result;

    result = verify_observation_to_high_pick_staging();
    if (result != 0) return result;

    result = verify_post_place_cycle_transition();
    if (result != 0) return result;

    result = verify_world_y_mirror();
    if (result != 0) return result;

    result = verify_shared_advance_selection();
    if (result != 0) return result;

    result = verify_planner_boundaries();
    if (result != 0) return result;

    memset(&start_ik, 0, sizeof(start_ik));
    if (ArmInverseKinematicsToolCenter(&pick, pitch_deg, seed, &start_ik) !=
        ARM_IK_OK) return 10;
    segment_count = build_route(pick, release);
    result = solve_route(segment_count, pitch_deg, start_ik.q_deg,
                         "forward_candidates.csv", release_q);
    if (result != 0) return result;

    segment_count = build_route(release, pick);
    result = solve_route(segment_count, pitch_deg, release_q,
                         "reverse_candidates.csv", NULL);
    return result;
}
