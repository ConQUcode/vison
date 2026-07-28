#include "arm_kinematics.h"

#include "math.h"
#include "string.h"

#define ARM_KIN_PI                    3.14159265358979323846f
#define ARM_KIN_DEG_TO_RAD            (ARM_KIN_PI / 180.0f)
#define ARM_KIN_RAD_TO_DEG            (180.0f / ARM_KIN_PI)
#define ARM_KIN_EPSILON               0.0001f
#define ARM_KIN_LIMIT_EPSILON_DEG     0.001f

static float ArmKinematicsClamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float ArmKinematicsWrapTo180(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

static float ArmKinematicsAngleDifference(float target_deg, float current_deg)
{
    return ArmKinematicsWrapTo180(target_deg - current_deg);
}

static float ArmKinematicsCandidateScore(const float q_deg[3],
                                          const float seed_q_deg[3])
{
    float shoulder_span = g_arm_calibration.shoulder_soft_max_deg -
                          g_arm_calibration.shoulder_soft_min_deg;
    float elbow_span = g_arm_calibration.elbow_soft_max_deg -
                       g_arm_calibration.elbow_soft_min_deg;
    float dq1;
    float dq2;
    float dq3;

    if (shoulder_span <= ARM_KIN_EPSILON || elbow_span <= ARM_KIN_EPSILON) {
        return INFINITY;
    }
    dq1 = ArmKinematicsAngleDifference(q_deg[0], seed_q_deg[0]) / 360.0f;
    dq2 = (q_deg[1] - seed_q_deg[1]) / shoulder_span;
    dq3 = (q_deg[2] - seed_q_deg[2]) / elbow_span;
    return dq1 * dq1 + dq2 * dq2 + dq3 * dq3;
}

uint8_t ArmJointPoseWithinSoftLimits(const float q_deg[3])
{
    if (q_deg == NULL || !isfinite(q_deg[0]) || !isfinite(q_deg[1]) ||
        !isfinite(q_deg[2])) {
        return 0u;
    }
    return q_deg[0] >= -180.0f - ARM_KIN_LIMIT_EPSILON_DEG &&
           q_deg[0] <= 180.0f + ARM_KIN_LIMIT_EPSILON_DEG &&
           q_deg[1] >= g_arm_calibration.shoulder_soft_min_deg -
                           ARM_KIN_LIMIT_EPSILON_DEG &&
           q_deg[1] <= g_arm_calibration.shoulder_soft_max_deg +
                           ARM_KIN_LIMIT_EPSILON_DEG &&
           q_deg[2] >= g_arm_calibration.elbow_soft_min_deg -
                           ARM_KIN_LIMIT_EPSILON_DEG &&
           q_deg[2] <= g_arm_calibration.elbow_soft_max_deg +
                           ARM_KIN_LIMIT_EPSILON_DEG;
}

uint8_t ArmAutoPoseIsSafe(const float q_deg[3])
{
    if (!ArmJointPoseWithinSoftLimits(q_deg)) {
        return 0u;
    }
    return q_deg[0] >= ARM_AUTO_Q1_MIN_DEG &&
           q_deg[0] <= ARM_AUTO_Q1_MAX_DEG &&
           q_deg[1] >= ARM_AUTO_Q2_MIN_DEG &&
           q_deg[1] <= ARM_AUTO_Q2_MAX_DEG &&
           q_deg[2] >= ARM_AUTO_Q3_MIN_DEG &&
           q_deg[2] <= ARM_AUTO_Q3_MAX_DEG;
}

void ArmForwardKinematics3DOF(float q1_deg,
                              float q2_deg,
                              float q3_deg,
                              Arm_Position_s *position)
{
    float q1;
    float q2;
    float q23;
    float link_radial;
    float local_x;
    float local_y;

    if (position == NULL) {
        return;
    }
    q1 = q1_deg * ARM_KIN_DEG_TO_RAD;
    q2 = q2_deg * ARM_KIN_DEG_TO_RAD;
    q23 = (q2_deg + q3_deg) * ARM_KIN_DEG_TO_RAD;
    link_radial = ARM_LINK_1_MM * cosf(q2) +
                  ARM_LINK_2_MM * cosf(q23);
    local_x = ARM_SHOULDER_OFFSET_FORWARD_MM + link_radial;
    local_y = ARM_SHOULDER_OFFSET_LEFT_MM;
    position->x_mm = local_x * cosf(q1) - local_y * sinf(q1);
    position->y_mm = local_x * sinf(q1) + local_y * cosf(q1);
    position->z_mm = ARM_BASE_HEIGHT_MM +
                     ARM_LINK_1_MM * sinf(q2) +
                     ARM_LINK_2_MM * sinf(q23);
}

uint8_t ArmKinematicsSelfTest(float *error_mm)
{
    Arm_Position_s position;
    float error;

    ArmForwardKinematics3DOF(0.0f, 180.0f, -180.0f, &position);
    error = sqrtf(position.x_mm * position.x_mm +
                  (position.y_mm - ARM_SHOULDER_OFFSET_LEFT_MM) *
                      (position.y_mm - ARM_SHOULDER_OFFSET_LEFT_MM) +
                  (position.z_mm - ARM_BASE_HEIGHT_MM) *
                      (position.z_mm - ARM_BASE_HEIGHT_MM));
    if (error_mm != NULL) {
        *error_mm = error;
    }
    return isfinite(error) && error <= 0.01f;
}

Arm_IK_Status_e ArmInverseKinematics3DOF(const Arm_Position_s *target,
                                         const float seed_q_deg[3],
                                         Arm_IK_Result_s *result)
{
    float rho;
    float z_planar;
    float local_x_magnitude;
    float target_azimuth;
    float best_score = 0.0f;
    uint8_t found = 0u;
    uint8_t geometry_found = 0u;
    uint8_t limited_found = 0u;

    if (result == NULL) {
        return ARM_IK_INVALID_ARGUMENT;
    }
    memset(result, 0, sizeof(*result));
    if (target == NULL || seed_q_deg == NULL ||
        !isfinite(target->x_mm) || !isfinite(target->y_mm) ||
        !isfinite(target->z_mm) || !isfinite(seed_q_deg[0]) ||
        !isfinite(seed_q_deg[1]) || !isfinite(seed_q_deg[2])) {
        result->status = ARM_IK_INVALID_ARGUMENT;
        return result->status;
    }
    if (!g_arm_calibration.joint_calibrated) {
        result->status = ARM_IK_JOINT_NOT_CALIBRATED;
        return result->status;
    }
    if (!g_arm_calibration.base_calibrated) {
        result->status = ARM_IK_BASE_NOT_CALIBRATED;
        return result->status;
    }

    rho = sqrtf(target->x_mm * target->x_mm +
                target->y_mm * target->y_mm);
    z_planar = target->z_mm - ARM_BASE_HEIGHT_MM;
    if (!isfinite(rho) || !isfinite(z_planar)) {
        result->status = ARM_IK_NUMERICAL_ERROR;
        return result->status;
    }
    if (rho * rho + ARM_KIN_EPSILON <
        ARM_SHOULDER_OFFSET_LEFT_MM * ARM_SHOULDER_OFFSET_LEFT_MM) {
        result->status = ARM_IK_OUT_OF_REACH;
        return result->status;
    }
    local_x_magnitude = sqrtf(fmaxf(
        0.0f,
        rho * rho -
            ARM_SHOULDER_OFFSET_LEFT_MM * ARM_SHOULDER_OFFSET_LEFT_MM));
    target_azimuth = rho > ARM_KIN_EPSILON ?
        atan2f(target->y_mm, target->x_mm) :
        seed_q_deg[0] * ARM_KIN_DEG_TO_RAD;

    for (uint8_t radial_index = 0u; radial_index < 2u; ++radial_index) {
        float local_x;
        float signed_radius;
        float q1_deg;
        float cos_q3;

        if (local_x_magnitude <= ARM_KIN_EPSILON && radial_index == 1u) {
            continue;
        }
        local_x = radial_index == 0u ?
            local_x_magnitude : -local_x_magnitude;
        signed_radius = local_x - ARM_SHOULDER_OFFSET_FORWARD_MM;
        q1_deg = ArmKinematicsWrapTo180(
            (target_azimuth -
             atan2f(ARM_SHOULDER_OFFSET_LEFT_MM, local_x)) *
            ARM_KIN_RAD_TO_DEG);
        cos_q3 = (signed_radius * signed_radius +
                  z_planar * z_planar -
                  ARM_LINK_1_MM * ARM_LINK_1_MM -
                  ARM_LINK_2_MM * ARM_LINK_2_MM) /
                 (2.0f * ARM_LINK_1_MM * ARM_LINK_2_MM);
        if (!isfinite(cos_q3)) {
            result->status = ARM_IK_NUMERICAL_ERROR;
            return result->status;
        }
        if (cos_q3 < -1.0f - ARM_KIN_EPSILON ||
            cos_q3 > 1.0f + ARM_KIN_EPSILON) {
            continue;
        }
        geometry_found = 1u;
        cos_q3 = ArmKinematicsClamp(cos_q3, -1.0f, 1.0f);

        for (uint8_t elbow_index = 0u; elbow_index < 2u; ++elbow_index) {
            float q3_rad = acosf(cos_q3);
            float q2_rad;
            float candidate[3];
            float score;

            if (elbow_index != 0u) {
                q3_rad = -q3_rad;
            }
            q2_rad = atan2f(z_planar, signed_radius) -
                     atan2f(ARM_LINK_2_MM * sinf(q3_rad),
                            ARM_LINK_1_MM +
                            ARM_LINK_2_MM * cosf(q3_rad));
            candidate[0] = q1_deg;
            candidate[1] = q2_rad * ARM_KIN_RAD_TO_DEG;
            candidate[2] = q3_rad * ARM_KIN_RAD_TO_DEG;
            if (!ArmJointPoseWithinSoftLimits(candidate)) {
                continue;
            }
            limited_found = 1u;
            if (!ArmAutoPoseIsSafe(candidate)) {
                continue;
            }

            result->candidate_count++;
            score = ArmKinematicsCandidateScore(candidate, seed_q_deg);
            if (!isfinite(score)) {
                result->status = ARM_IK_NUMERICAL_ERROR;
                return result->status;
            }
            if (!found || score < best_score) {
                found = 1u;
                best_score = score;
                memcpy(result->q_deg, candidate, sizeof(result->q_deg));
            }
        }
    }

    if (!found) {
        if (limited_found) {
            result->status = ARM_IK_COLLISION_RISK;
        } else if (geometry_found) {
            result->status = ARM_IK_NO_LIMITED_SOLUTION;
        } else {
            result->status = ARM_IK_OUT_OF_REACH;
        }
        return result->status;
    }

    ArmForwardKinematics3DOF(result->q_deg[0], result->q_deg[1],
                             result->q_deg[2], &result->fk_position);
    result->position_error_mm = sqrtf(
        (result->fk_position.x_mm - target->x_mm) *
            (result->fk_position.x_mm - target->x_mm) +
        (result->fk_position.y_mm - target->y_mm) *
            (result->fk_position.y_mm - target->y_mm) +
        (result->fk_position.z_mm - target->z_mm) *
            (result->fk_position.z_mm - target->z_mm));
    if (!isfinite(result->position_error_mm)) {
        result->status = ARM_IK_NUMERICAL_ERROR;
        return result->status;
    }
    result->status = ARM_IK_OK;
    return result->status;
}
