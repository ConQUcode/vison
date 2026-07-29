#include "arm_kinematics.h"

#include "math.h"
#include "string.h"

#define ARM_KIN_PI                    3.14159265358979323846f
#define ARM_KIN_DEG_TO_RAD            (ARM_KIN_PI / 180.0f)
#define ARM_KIN_RAD_TO_DEG            (180.0f / ARM_KIN_PI)
#define ARM_KIN_EPSILON               0.0001f
#define ARM_KIN_LIMIT_EPSILON_DEG     0.001f

/*
 * 三自由度运动学说明：
 * q1为底座偏航，q2为大臂绝对俯仰。
 * q3采用机械定义：q3 = -两杆物理内夹角。
 * 两杆物理夹角90deg时q3=-90deg，两杆完全伸直180deg时q3=-180deg。
 * 标准二连杆内部有向转角q3_math = -180deg - q3。
 * 当前输出点是腕部舵机安装轴心，不包含q4和末端工具长度。
 * 底座、大臂、小臂转轴无Y向偏移；肩轴高度为250mm。
 */

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

static float ArmElbowMechanicalToMathDeg(float q3_mech_deg)
{
    return -180.0f - q3_mech_deg;
}

static float ArmElbowMathToMechanicalDeg(float q3_math_deg)
{
    return -180.0f - q3_math_deg;
}

static float ArmKinematicsCandidateScore(const float q_deg[3],
                                          const float seed_q_deg[3])
{
    float shoulder_span = ARM_Q2_SOFT_MAX_DEG - ARM_Q2_SOFT_MIN_DEG;
    float elbow_span = ARM_Q3_SOFT_MAX_DEG - ARM_Q3_SOFT_MIN_DEG;
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
    return q_deg[0] >= ARM_Q1_SOFT_MIN_DEG - ARM_LIMIT_TOLERANCE_DEG &&
           q_deg[0] <= ARM_Q1_SOFT_MAX_DEG + ARM_LIMIT_TOLERANCE_DEG &&
           q_deg[1] >= ARM_Q2_SOFT_MIN_DEG - ARM_LIMIT_TOLERANCE_DEG &&
           q_deg[1] <= ARM_Q2_SOFT_MAX_DEG + ARM_LIMIT_TOLERANCE_DEG &&
           q_deg[2] >= ARM_Q3_SOFT_MIN_DEG - ARM_LIMIT_TOLERANCE_DEG &&
           q_deg[2] <= ARM_Q3_SOFT_MAX_DEG + ARM_LIMIT_TOLERANCE_DEG;
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
    q23 = (q2_deg + ArmElbowMechanicalToMathDeg(q3_deg)) *
        ARM_KIN_DEG_TO_RAD;
    /* 先在机械臂竖直平面内计算肩部到腕部的径向长度。 */
    link_radial = ARM_LINK_1_MM * cosf(q2) +
                  ARM_LINK_2_MM * cosf(q23);
    local_x = ARM_SHOULDER_OFFSET_FORWARD_MM + link_radial;
    local_y = ARM_SHOULDER_OFFSET_LEFT_MM;
    /* 将肩部固定偏移和连杆径向位置随q1旋转到小车坐标系。 */
    position->x_mm = local_x * cosf(q1) - local_y * sinf(q1);
    position->y_mm = local_x * sinf(q1) + local_y * cosf(q1);
    position->z_mm = ARM_BASE_HEIGHT_MM +
                     ARM_LINK_1_MM * sinf(q2) +
                     ARM_LINK_2_MM * sinf(q23);
}

uint8_t ArmKinematicsSelfTest(float *error_mm)
{
    Arm_Position_s position;
    Arm_Position_s expected;
    float error;

    ArmForwardKinematics3DOF(ARM_SAFE_Q1_DEG, ARM_SAFE_Q2_DEG,
                             ARM_SAFE_Q3_DEG, &position);
    expected.x_mm = ARM_LINK_2_MM;
    expected.y_mm = 0.0f;
    expected.z_mm = ARM_BASE_HEIGHT_MM + ARM_LINK_1_MM;
    error = sqrtf(
        (position.x_mm - expected.x_mm) *
            (position.x_mm - expected.x_mm) +
        (position.y_mm - expected.y_mm) *
            (position.y_mm - expected.y_mm) +
        (position.z_mm - expected.z_mm) *
            (position.z_mm - expected.z_mm));
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

    /*
     * 同一空间点可能对应局部径向正/负两个分支，每个分支又有肘上/肘下
     * 两个q3解，因此最多得到4组候选。候选必须通过软限位和自动区域，
     * 最后选择与seed姿态距离最近的连续解。
     */
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
            float q3_math_deg;

            if (elbow_index != 0u) {
                q3_rad = -q3_rad;
            }
            q2_rad = atan2f(z_planar, signed_radius) -
                     atan2f(ARM_LINK_2_MM * sinf(q3_rad),
                            ARM_LINK_1_MM +
                            ARM_LINK_2_MM * cosf(q3_rad));
            q3_math_deg = q3_rad * ARM_KIN_RAD_TO_DEG;
            candidate[0] = q1_deg;
            candidate[1] = q2_rad * ARM_KIN_RAD_TO_DEG;
            candidate[2] = ArmElbowMathToMechanicalDeg(q3_math_deg);
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
