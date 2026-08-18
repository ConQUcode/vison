#include "arm_kinematics.h"
#include "arm_config.h"
#include "arm_tool.h"
#include "app_config.h"
#include "app_fruit_task_config.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define HOST_PI 3.14159265358979323846f
#define HOST_MAX_POINTS ARM_LINEAR_MAX_SAMPLES
#define HOST_NONE 0xFFu
#define HOST_INF 1.0e30f

typedef struct {
    Arm_Position_s center;
    uint8_t count;
    Arm_Tool_Center_IK_Candidate_s candidate[
        ARM_TOOL_CENTER_IK_MAX_CANDIDATES];
    uint8_t predecessor[ARM_TOOL_CENTER_IK_MAX_CANDIDATES];
    float cost[ARM_TOOL_CENTER_IK_MAX_CANDIDATES];
} Host_Point_s;

static Host_Point_s points[HOST_MAX_POINTS];

static float wrap180(float value)
{
    while (value > 180.0f) value -= 360.0f;
    while (value < -180.0f) value += 360.0f;
    return value;
}

static uint8_t continuous(const float a[3], const float b[3])
{
    return fabsf(wrap180(b[0] - a[0])) <= 5.0f &&
           fabsf(b[1] - a[1]) <= 2.0f &&
           fabsf(b[2] - a[2]) <= 2.0f;
}

static float transition_cost(const float a[3], const float b[3])
{
    float d0 = wrap180(b[0] - a[0]) / 5.0f;
    float d1 = (b[1] - a[1]) / 2.0f;
    float d2 = (b[2] - a[2]) / 2.0f;
    return d0 * d0 + d1 * d1 + d2 * d2;
}

static uint8_t pitch_safe(const float q[3], float pitch_deg)
{
    float small_link_pitch = q[1] + (-180.0f - q[2]);
    float relative_pitch = pitch_deg - small_link_pitch;
    float position = ARM_TOOL_PITCH_NEUTRAL_POS +
        ARM_TOOL_PITCH_DIRECTION * relative_pitch * 1000.0f /
        ARM_TOOL_SERVO_RANGE_DEG;
    return position >= ARM_TOOL_PITCH_SERVO_MIN_POS &&
           position <= ARM_TOOL_PITCH_SERVO_MAX_POS;
}

static uint8_t pose_safe(const Arm_Tool_Center_IK_Candidate_s *candidate,
                         float pitch_deg)
{
    return ArmJointPoseWithinSoftLimits(candidate->q_deg) &&
           ArmAutoPoseIsSafe(candidate->q_deg) &&
           pitch_safe(candidate->q_deg, pitch_deg) &&
           !(candidate->tool_center_mm.x_mm >
                 ARM_FRONT_BARRIER_TOOL_X_MARGIN_MM &&
             candidate->q_deg[ARM_JOINT_SHOULDER] >
                 ARM_FRONT_BARRIER_SHOULDER_Q2_MAX_DEG);
}

static uint16_t append_segment(Arm_Position_s start, Arm_Position_s end,
                               uint16_t count)
{
    float dx = end.x_mm - start.x_mm;
    float dy = end.y_mm - start.y_mm;
    float dz = end.z_mm - start.z_mm;
    uint16_t intervals = (uint16_t)ceilf(
        sqrtf(dx * dx + dy * dy + dz * dz) /
        ARM_LINEAR_SAMPLE_SPACING_MM);
    if (intervals < 1u) intervals = 1u;
    for (uint16_t i = 1u; i <= intervals; ++i) {
        float ratio = (float)i / (float)intervals;
        if (count >= HOST_MAX_POINTS) return 0u;
        points[count].center.x_mm = start.x_mm + ratio * dx;
        points[count].center.y_mm = start.y_mm + ratio * dy;
        points[count].center.z_mm = start.z_mm + ratio * dz;
        count++;
    }
    return count;
}

static uint16_t build_route(Arm_Position_s start, Arm_Position_s target)
{
    Arm_Position_s route[ARM_TRAJECTORY_MAX_ROUTE_SEGMENTS + 1u];
    float clearance = fmaxf(ARM_REAR_CROSSING_TOOL_Z_MM,
                            fmaxf(start.z_mm, target.z_mm));
    float start_radius = hypotf(start.x_mm, start.y_mm);
    float target_radius = hypotf(target.x_mm, target.y_mm);
    float start_sign = start.x_mm >= 0.0f ? 1.0f : -1.0f;
    float target_sign = target.x_mm >= 0.0f ? 1.0f : -1.0f;
    uint8_t arc_steps = (uint8_t)(90.0f /
        ARM_REAR_BYPASS_ARC_STEP_DEG + 0.5f);
    uint8_t segment_count = 0u;
    uint16_t count = 1u;

    route[0] = start;
    route[++segment_count] = start;
    route[segment_count].z_mm = clearance;
    for (uint8_t step = 1u; step <= arc_steps; ++step) {
        float angle = step * ARM_REAR_BYPASS_ARC_STEP_DEG * HOST_PI / 180.0f;
        route[++segment_count].x_mm = start_sign * start_radius * cosf(angle);
        route[segment_count].y_mm = -start_sign * start_radius * sinf(angle);
        route[segment_count].z_mm = clearance;
    }
    route[++segment_count].x_mm = 0.0f;
    route[segment_count].y_mm = -target_sign * target_radius;
    route[segment_count].z_mm = clearance;
    for (uint8_t step = 1u; step <= arc_steps; ++step) {
        float angle = (90.0f - step * ARM_REAR_BYPASS_ARC_STEP_DEG) *
                      HOST_PI / 180.0f;
        route[++segment_count].x_mm = target_sign * target_radius * cosf(angle);
        route[segment_count].y_mm = -target_sign * target_radius * sinf(angle);
        route[segment_count].z_mm = clearance;
    }
    route[++segment_count] = target;
    points[0].center = start;
    for (uint8_t segment = 0u; segment < segment_count; ++segment) {
        count = append_segment(route[segment], route[segment + 1u], count);
        if (count == 0u) return 0u;
    }
    printf("segments=%u samples=%u\n", segment_count, count);
    return count;
}

static int solve_route(uint16_t count, float pitch_deg,
                       const float start_q[3], const char *csv_name,
                       float final_q[3])
{
    FILE *csv = fopen(csv_name, "w");
    float seed[3] = {start_q[0], start_q[1], start_q[2]};
    uint8_t selected[HOST_MAX_POINTS];
    uint8_t best = HOST_NONE;

    if (csv == NULL) return 2;
    fprintf(csv, "sample,x,y,z,candidate,q1,q2,q3,cost,predecessor\n");
    for (uint16_t i = 1u; i < count; ++i) {
        float direction_seed[3] = {seed[0], seed[1], seed[2]};
        if (hypotf(points[i].center.x_mm, points[i].center.y_mm) <=
            0.0001f) {
            direction_seed[0] = atan2f(points[i - 1u].center.y_mm,
                                       points[i - 1u].center.x_mm) *
                                180.0f / HOST_PI;
        }
        Arm_IK_Status_e status = ArmInverseKinematicsToolCenterAll(
            &points[i].center, pitch_deg, direction_seed,
            points[i].candidate,
            &points[i].count);
        uint8_t reachable = 0u;
        for (uint8_t c = 0u; c < ARM_TOOL_CENTER_IK_MAX_CANDIDATES; ++c) {
            points[i].cost[c] = HOST_INF;
            points[i].predecessor[c] = HOST_NONE;
        }
        if (status != ARM_IK_OK) {
            printf("IK failure sample=%u center=(%.3f,%.3f,%.3f) status=%d\n",
                   i, points[i].center.x_mm, points[i].center.y_mm,
                   points[i].center.z_mm, status);
            fclose(csv);
            return 3;
        }
        for (uint8_t c = 0u; c < points[i].count; ++c) {
            if (!pose_safe(&points[i].candidate[c], pitch_deg)) continue;
            if (i == 1u) {
                if (continuous(start_q, points[i].candidate[c].q_deg)) {
                    points[i].cost[c] = transition_cost(
                        start_q, points[i].candidate[c].q_deg);
                    reachable++;
                }
            } else {
                for (uint8_t p = 0u; p < points[i - 1u].count; ++p) {
                    float cost;
                    if (points[i - 1u].cost[p] >= HOST_INF ||
                        !continuous(points[i - 1u].candidate[p].q_deg,
                                    points[i].candidate[c].q_deg)) continue;
                    cost = points[i - 1u].cost[p] + transition_cost(
                        points[i - 1u].candidate[p].q_deg,
                        points[i].candidate[c].q_deg);
                    if (cost < points[i].cost[c]) {
                        points[i].cost[c] = cost;
                        points[i].predecessor[c] = p;
                    }
                }
                if (points[i].cost[c] < HOST_INF) reachable++;
            }
            fprintf(csv, "%u,%.6f,%.6f,%.6f,%u,%.6f,%.6f,%.6f,%.9g,%u\n",
                    i, points[i].center.x_mm, points[i].center.y_mm,
                    points[i].center.z_mm, c,
                    points[i].candidate[c].q_deg[0],
                    points[i].candidate[c].q_deg[1],
                    points[i].candidate[c].q_deg[2], points[i].cost[c],
                    points[i].predecessor[c]);
        }
        if (reachable == 0u) {
            printf("No continuous safe candidate sample=%u center=(%.3f,%.3f,%.3f) candidates=%u\n",
                   i, points[i].center.x_mm, points[i].center.y_mm,
                   points[i].center.z_mm, points[i].count);
            fclose(csv);
            return 4;
        }
    }
    for (uint8_t c = 0u; c < points[count - 1u].count; ++c) {
        if (points[count - 1u].cost[c] < HOST_INF &&
            (best == HOST_NONE || points[count - 1u].cost[c] <
                                  points[count - 1u].cost[best])) best = c;
    }
    selected[count - 1u] = best;
    for (uint16_t i = count - 1u; i > 1u; --i) {
        selected[i - 1u] = points[i].predecessor[selected[i]];
        if (selected[i - 1u] == HOST_NONE) {
            fclose(csv);
            return 5;
        }
    }
    printf("PASS final q=(%.3f, %.3f, %.3f) cost=%.6f\n",
           points[count - 1u].candidate[best].q_deg[0],
           points[count - 1u].candidate[best].q_deg[1],
           points[count - 1u].candidate[best].q_deg[2],
           points[count - 1u].cost[best]);
    if (final_q != NULL) {
        memcpy(final_q, points[count - 1u].candidate[best].q_deg,
               sizeof(points[count - 1u].candidate[best].q_deg));
    }
    fclose(csv);
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
        if (!staging_pose_safe(left_q_deg, left_relative_pitch_deg,
                               &left_center) ||
            !staging_pose_safe(right_q_deg, right_relative_pitch_deg,
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

            if (fabsf(left_raise_mm -
                      APP_ARM_POSTURE_TEST_TRANSFER_Z_RAISE_MM) >
                    APP_ARM_POSTURE_TEST_TRANSFER_Z_TOLERANCE_MM ||
                fabsf(right_raise_mm -
                      APP_ARM_POSTURE_TEST_TRANSFER_Z_RAISE_MM) >
                    APP_ARM_POSTURE_TEST_TRANSFER_Z_TOLERANCE_MM) {
                fprintf(stderr,
                        "FAIL AC transfer Z raise left=%.3f right=%.3f "
                        "target=%.3f tolerance=%.3f\n",
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
    const float left_safe_q_deg[3] = {
        APP_FRUIT_A_LEFT_SAFE_Q1_DEG,
        APP_FRUIT_A_TRANSFER_Q2_DEG,
        APP_FRUIT_A_TRANSFER_Q3_DEG
    };
    const float right_safe_q_deg[3] = {
        APP_FRUIT_A_RIGHT_SAFE_Q1_DEG,
        APP_FRUIT_A_TRANSFER_Q2_DEG,
        APP_FRUIT_A_TRANSFER_Q3_DEG
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
        left_safe_q_deg, right_safe_q_deg,
        left_relative_pitch_deg, right_relative_pitch_deg,
        0u, &max_abs_y_mm, NULL, NULL);
    if (result != 0) return result;

    printf("PASS AC post-grip transfer: waypoint=(+/-90.0,%.1f,%.1f) "
           "waypointZ=%.3f raiseZ=%.3f safe=(+/-90.0,%.1f,%.1f) "
           "maxAbsY=%.3f limit=%.1f\n",
           APP_ARM_POSTURE_TEST_TRANSFER_WAYPOINT_Q2_DEG,
           APP_ARM_POSTURE_TEST_TRANSFER_WAYPOINT_Q3_DEG,
           waypoint_z_mm, z_raise_mm,
           APP_FRUIT_A_TRANSFER_Q2_DEG,
           APP_FRUIT_A_TRANSFER_Q3_DEG,
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
    uint16_t count;
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

    memset(points, 0, sizeof(points));
    points[0].center = staging_center;
    count = append_segment(staging_center, *target_center, 1u);
    if (count == 0u) return 33;
    for (uint16_t i = 0u; i < count; ++i) {
        if (fabsf(points[i].center.y_mm) >
                APP_ARM_BD_OBSERVATION_PATH_Y_MAX_MM + 0.001f) {
            fprintf(stderr,
                    "FAIL BD linear Y sample=%u y=%.3f limit=%.3f\n",
                    i, points[i].center.y_mm,
                    APP_ARM_BD_OBSERVATION_PATH_Y_MAX_MM);
            return 34;
        }
        max_abs_y_mm = fmaxf(max_abs_y_mm,
                             fabsf(points[i].center.y_mm));
        min_z_mm = fminf(min_z_mm, points[i].center.z_mm);
    }
    result = solve_route(count, APP_ARM_BD_OBSERVATION_TOOL_PITCH_DEG,
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
        uint16_t count;
        int result;

        if (!ArmForwardKinematicsToolCenter(
                staging_q_deg, APP_ARM_POSTURE_TEST_TOOL_PITCH_DEG,
                &staging_center)) {
            return 28;
        }
        memset(points, 0, sizeof(points));
        points[0].center = staging_center;
        count = append_segment(staging_center, approach_center, 1u);
        result = solve_route(count, APP_ARM_POSTURE_TEST_TOOL_PITCH_DEG,
                             staging_q_deg, approach_csv[side],
                             approach_q_deg);
        if (result != 0) {
            fprintf(stderr, "FAIL staging approach side=%u result=%d\n",
                    side, result);
            return 29;
        }

        memset(points, 0, sizeof(points));
        points[0].center = approach_center;
        count = append_segment(approach_center, advance_center, 1u);
        result = solve_route(count, APP_ARM_POSTURE_TEST_TOOL_PITCH_DEG,
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

int main(void)
{
    const float pitch_deg = -30.0f;
    Arm_Position_s pick = {250.0f, 0.0f, 200.0f};
    Arm_Position_s release = {-22.0f, 0.0f, 230.0f};
    Arm_Tool_Center_IK_Result_s start_ik;
    float seed[3] = {0.0f, 114.0f, -43.0f};
    uint16_t count;
    float release_q[3];
    int result;

    result = verify_bd_observation_transition();
    if (result != 0) return result;

    result = verify_ac_post_grip_transfer();
    if (result != 0) return result;

    result = verify_post_place_cycle_transition();
    if (result != 0) return result;

    result = verify_world_y_mirror();
    if (result != 0) return result;

    memset(&start_ik, 0, sizeof(start_ik));
    if (ArmInverseKinematicsToolCenter(&pick, pitch_deg, seed, &start_ik) !=
        ARM_IK_OK) return 10;
    count = build_route(pick, release);
    result = solve_route(count, pitch_deg, start_ik.q_deg,
                         "forward_candidates.csv", release_q);
    if (result != 0) return result;

    memset(points, 0, sizeof(points));
    count = build_route(release, pick);
    result = solve_route(count, pitch_deg, release_q,
                         "reverse_candidates.csv", NULL);
    return result;
}
