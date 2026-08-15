/**
 * @file arm_kinematics.c
 * @brief 以 ID1 俯仰舵机轴心为末端点的三自由度机械臂 FK/IK。
 */

#include "arm_kinematics.h"

#include "arm_tool.h"

#include "math.h"
#include "string.h"

#define ARM_KIN_PI                    3.14159265358979323846f
#define ARM_KIN_DEG_TO_RAD            (ARM_KIN_PI / 180.0f)
#define ARM_KIN_RAD_TO_DEG            (180.0f / ARM_KIN_PI)
#define ARM_KIN_EPSILON               0.0001f
#define ARM_TOOL_IK_BASE_DIRECTION_TOLERANCE_DEG 0.5f
#define ARM_KIN_LIMIT_EPSILON_DEG     0.001f

/*
 * 三自由度运动学说明：
 * q1为底座电机偏航，正角对应机械臂朝物理左侧（世界Y正方向）旋转；
 * q2为大臂绝对俯仰。
 * q3采用机械定义：q3 = -两杆物理内夹角。
 * 两杆物理夹角90deg时q3=-90deg，两杆完全伸直180deg时q3=-180deg。
 * 标准二连杆内部有向转角q3_math = -180deg - q3。
 * 当前输出点是腕部舵机安装轴心，不包含q4和末端工具长度。
 * 底座、大臂、小臂转轴无Y向偏移；肩轴高度由ARM_BASE_HEIGHT_MM配置。
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
    /*
     * 自动区域与软限位必须使用同一数值容差。圆弧端点理论上为q1=90deg，
     * 但float三角函数会产生约1e-5deg误差；无容差复查会把边界合法点误拒。
     */
    return q_deg[0] >= ARM_AUTO_Q1_MIN_DEG - ARM_LIMIT_TOLERANCE_DEG &&
           q_deg[0] <= ARM_AUTO_Q1_MAX_DEG + ARM_LIMIT_TOLERANCE_DEG &&
           q_deg[1] >= ARM_AUTO_Q2_MIN_DEG - ARM_LIMIT_TOLERANCE_DEG &&
           q_deg[1] <= ARM_AUTO_Q2_MAX_DEG + ARM_LIMIT_TOLERANCE_DEG &&
           q_deg[2] >= ARM_AUTO_Q3_MIN_DEG - ARM_LIMIT_TOLERANCE_DEG &&
           q_deg[2] <= ARM_AUTO_Q3_MAX_DEG + ARM_LIMIT_TOLERANCE_DEG;
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
    /* q1正方向朝Y正侧；local_y正值表示肩轴向物理左侧安装。 */
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
    float max_error;

    /*
     * 运动学自检必须使用固定的已知解析姿态，不能复用可调的上电待机
     * 姿态ARM_SAFE_Q*。否则只要修改启动姿态，就会在电机使能前误报
     * ARM_FAULT_CONFIG并触发三轴失能。
     *
     * 固定参考姿态[0, 90, -90]：
     * 大臂竖直向上，小臂水平向前，腕部轴心为
     * (LINK_2, 0, BASE_HEIGHT + LINK_1)。
     */
    ArmForwardKinematics3DOF(0.0f, 90.0f, -90.0f, &position);
    expected.x_mm = ARM_SHOULDER_OFFSET_FORWARD_MM + ARM_LINK_2_MM;
    expected.y_mm = ARM_SHOULDER_OFFSET_LEFT_MM;
    expected.z_mm = ARM_BASE_HEIGHT_MM + ARM_LINK_1_MM;
    error = sqrtf(
        (position.x_mm - expected.x_mm) *
            (position.x_mm - expected.x_mm) +
        (position.y_mm - expected.y_mm) *
            (position.y_mm - expected.y_mm) +
        (position.z_mm - expected.z_mm) *
            (position.z_mm - expected.z_mm));
    max_error = error;

    /* 实机坐标约定：q1=+90deg朝物理左侧，即世界Y正方向。 */
    ArmForwardKinematics3DOF(90.0f, 90.0f, -90.0f, &position);
    expected.x_mm = -ARM_SHOULDER_OFFSET_LEFT_MM;
    expected.y_mm = ARM_SHOULDER_OFFSET_FORWARD_MM + ARM_LINK_2_MM;
    expected.z_mm = ARM_BASE_HEIGHT_MM + ARM_LINK_1_MM;
    error = sqrtf(
        (position.x_mm - expected.x_mm) *
            (position.x_mm - expected.x_mm) +
        (position.y_mm - expected.y_mm) *
            (position.y_mm - expected.y_mm) +
        (position.z_mm - expected.z_mm) *
            (position.z_mm - expected.z_mm));
    if (!isfinite(error)) {
        max_error = INFINITY;
    } else if (error > max_error) {
        max_error = error;
    }

    /* q1=-90deg必须镜像到物理右侧，即世界Y负方向。 */
    ArmForwardKinematics3DOF(-90.0f, 90.0f, -90.0f, &position);
    expected.x_mm = ARM_SHOULDER_OFFSET_LEFT_MM;
    expected.y_mm = -(ARM_SHOULDER_OFFSET_FORWARD_MM + ARM_LINK_2_MM);
    expected.z_mm = ARM_BASE_HEIGHT_MM + ARM_LINK_1_MM;
    error = sqrtf(
        (position.x_mm - expected.x_mm) *
            (position.x_mm - expected.x_mm) +
        (position.y_mm - expected.y_mm) *
            (position.y_mm - expected.y_mm) +
        (position.z_mm - expected.z_mm) *
            (position.z_mm - expected.z_mm));
    if (!isfinite(error)) {
        max_error = INFINITY;
    } else if (error > max_error) {
        max_error = error;
    }
    if (error_mm != NULL) {
        *error_mm = max_error;
    }
    return isfinite(max_error) && max_error <= 0.01f;
}

/*
 * 三自由度逆解的统一实现。
 * 普通腕部轴心命令不限制底座方向；工具中心命令在反算117 mm偏移后，
 * 必须把q1限制在同一工具径向方向，否则靠近底座轴线时可能选中相反
 * 径向分支，导致一个实际可达的工具中心点被误判为逆解失败。
 */
static Arm_IK_Status_e ArmInverseKinematics3DOFInternal(
    const Arm_Position_s *target,
    const float seed_q_deg[3],
    uint8_t base_direction_constraint_valid,
    float required_base_direction_deg,
    float base_direction_tolerance_deg,
    float all_q_deg[ARM_TOOL_CENTER_IK_MAX_CANDIDATES][3],
    uint8_t *all_count,
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
    if (all_count != NULL) {
        *all_count = 0u;
    }
    if (target == NULL || seed_q_deg == NULL ||
        !isfinite(target->x_mm) || !isfinite(target->y_mm) ||
        !isfinite(target->z_mm) || !isfinite(seed_q_deg[0]) ||
        !isfinite(seed_q_deg[1]) || !isfinite(seed_q_deg[2]) ||
        (base_direction_constraint_valid != 0u &&
         (!isfinite(required_base_direction_deg) ||
          !isfinite(base_direction_tolerance_deg) ||
          base_direction_tolerance_deg < 0.0f))) {
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
        atan2f(target->y_mm, target->x_mm) : 0.0f;

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
        q1_deg = rho > ARM_KIN_EPSILON ?
            ArmKinematicsWrapTo180(
                (target_azimuth -
                 atan2f(ARM_SHOULDER_OFFSET_LEFT_MM, local_x)) *
                ARM_KIN_RAD_TO_DEG) :
            seed_q_deg[0];
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
            /*
             * 约束必须在候选枚举阶段执行，不能靠修改seed权重间接选择。
             * 同一个腕部点的正/负径向分支相差约180deg；工具偏移使用哪
             * 个方向反算，就只允许该方向对应的q1候选继续参与限位检查。
             */
            if (base_direction_constraint_valid != 0u &&
                fabsf(ArmKinematicsAngleDifference(
                    candidate[ARM_JOINT_BASE_YAW],
                    required_base_direction_deg)) >
                    base_direction_tolerance_deg) {
                continue;
            }
            if (!ArmJointPoseWithinSoftLimits(candidate)) {
                continue;
            }
            limited_found = 1u;
            if (!ArmAutoPoseIsSafe(candidate)) {
                continue;
            }

            result->candidate_count++;
            if (all_q_deg != NULL && all_count != NULL &&
                *all_count < ARM_TOOL_CENTER_IK_MAX_CANDIDATES) {
                memcpy(all_q_deg[*all_count], candidate,
                       sizeof(all_q_deg[*all_count]));
                (*all_count)++;
            }
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

Arm_IK_Status_e ArmInverseKinematics3DOF(const Arm_Position_s *target,
                                         const float seed_q_deg[3],
                                         Arm_IK_Result_s *result)
{
    return ArmInverseKinematics3DOFInternal(target, seed_q_deg,
                                             0u, 0.0f, 0.0f,
                                             NULL, NULL, result);
}

uint8_t ArmForwardKinematicsToolCenter(
    const float q_deg[3], float tool_pitch_deg,
    Arm_Position_s *tool_center_mm)
{
    Arm_Position_s wrist_center;

    if (q_deg == NULL || tool_center_mm == NULL ||
        !isfinite(q_deg[0]) || !isfinite(q_deg[1]) ||
        !isfinite(q_deg[2]) || !isfinite(tool_pitch_deg)) {
        return 0u;
    }
    ArmForwardKinematics3DOF(q_deg[0], q_deg[1], q_deg[2], &wrist_center);
    return ArmToolGetCenterFromWrist(&wrist_center, q_deg[0],
                                      tool_pitch_deg, tool_center_mm);
}

Arm_IK_Status_e ArmInverseKinematicsToolCenterAll(
    const Arm_Position_s *target_center_mm,
    float tool_pitch_deg,
    const float seed_q_deg[3],
    Arm_Tool_Center_IK_Candidate_s candidates[
        ARM_TOOL_CENTER_IK_MAX_CANDIDATES],
    uint8_t *candidate_count)
{
    float azimuth_deg;
    uint8_t output_count = 0u;
    Arm_IK_Status_e best_failure = ARM_IK_OUT_OF_REACH;

    if (candidate_count == NULL) {
        return ARM_IK_INVALID_ARGUMENT;
    }
    *candidate_count = 0u;
    if (target_center_mm == NULL || seed_q_deg == NULL ||
        candidates == NULL ||
        !isfinite(target_center_mm->x_mm) ||
        !isfinite(target_center_mm->y_mm) ||
        !isfinite(target_center_mm->z_mm) ||
        !isfinite(tool_pitch_deg) || !isfinite(seed_q_deg[0]) ||
        !isfinite(seed_q_deg[1]) || !isfinite(seed_q_deg[2])) {
        return ARM_IK_INVALID_ARGUMENT;
    }

    azimuth_deg = (fabsf(target_center_mm->x_mm) > ARM_KIN_EPSILON ||
                   fabsf(target_center_mm->y_mm) > ARM_KIN_EPSILON) ?
        atan2f(target_center_mm->y_mm, target_center_mm->x_mm) *
            ARM_KIN_RAD_TO_DEG : seed_q_deg[0];

    /*
     * 夹爪中心可能位于主臂q1方向的正径向或负径向一侧。两个候选都先
     * 反算ID1轴心，再交给基础IK；最终必须使用基础IK给出的真实q1重新
     * 做工具FK，因此负X目标不会被atan2产生的180deg方位误导。
     */
    for (uint8_t direction = 0u; direction < 2u; ++direction) {
        Arm_Position_s wrist_candidate;
        Arm_IK_Result_s wrist_ik;
        float wrist_q_deg[ARM_TOOL_CENTER_IK_MAX_CANDIDATES][3];
        uint8_t wrist_count = 0u;
        float direction_deg = ArmKinematicsWrapTo180(
            azimuth_deg + (direction != 0u ? 180.0f : 0.0f));

        memset(&wrist_ik, 0, sizeof(wrist_ik));
        if (!ArmToolGetWristFromCenter(target_center_mm, direction_deg,
                                      tool_pitch_deg, &wrist_candidate)) {
            best_failure = ARM_IK_INVALID_ARGUMENT;
            continue;
        }
        /*
         * 117 mm工具偏移按direction_deg反算后，基础IK必须显式筛选同向
         * q1分支。真实seed仍用于同一分支内的肘部解连续性评分。
         */
        if (ArmInverseKinematics3DOFInternal(
                &wrist_candidate, seed_q_deg, 1u, direction_deg,
                ARM_TOOL_IK_BASE_DIRECTION_TOLERANCE_DEG,
                wrist_q_deg, &wrist_count,
                &wrist_ik) != ARM_IK_OK) {
            best_failure = wrist_ik.status;
            continue;
        }
        /*
         * 此处只求几何可达且满足关节软件限位的解。前方栏框限制依赖
         * “普通前方运动”或“已规划的高位跨区运动”等路径意图，因此
         * 统一由轨迹工作区安全层判断，不能在通用IK中提前过滤。
         */
        for (uint8_t i = 0u; i < wrist_count; ++i) {
            Arm_Position_s center_check;
            Arm_Position_s wrist_check;
            float error_mm;

            if (!ArmForwardKinematicsToolCenter(
                    wrist_q_deg[i], tool_pitch_deg, &center_check)) {
                best_failure = ARM_IK_NUMERICAL_ERROR;
                continue;
            }
            ArmForwardKinematics3DOF(
                wrist_q_deg[i][0], wrist_q_deg[i][1], wrist_q_deg[i][2],
                &wrist_check);
            error_mm = sqrtf(
                (center_check.x_mm - target_center_mm->x_mm) *
                    (center_check.x_mm - target_center_mm->x_mm) +
                (center_check.y_mm - target_center_mm->y_mm) *
                    (center_check.y_mm - target_center_mm->y_mm) +
                (center_check.z_mm - target_center_mm->z_mm) *
                    (center_check.z_mm - target_center_mm->z_mm));
            if (!isfinite(error_mm) ||
                error_mm > ARM_LINEAR_FK_ERROR_MAX_MM) {
                best_failure = ARM_IK_NUMERICAL_ERROR;
                continue;
            }
            if (output_count < ARM_TOOL_CENTER_IK_MAX_CANDIDATES) {
                memcpy(candidates[output_count].q_deg, wrist_q_deg[i],
                       sizeof(candidates[output_count].q_deg));
                candidates[output_count].wrist_center_mm = wrist_check;
                candidates[output_count].tool_center_mm = center_check;
                candidates[output_count].position_error_mm = error_mm;
                output_count++;
            }
        }
    }
    *candidate_count = output_count;
    return output_count != 0u ? ARM_IK_OK : best_failure;
}

Arm_IK_Status_e ArmInverseKinematicsToolCenter(
    const Arm_Position_s *target_center_mm,
    float tool_pitch_deg,
    const float seed_q_deg[3],
    Arm_Tool_Center_IK_Result_s *result)
{
    Arm_Tool_Center_IK_Candidate_s candidates[
        ARM_TOOL_CENTER_IK_MAX_CANDIDATES];
    Arm_IK_Status_e status;
    float best_score = 0.0f;
    uint8_t candidate_count = 0u;
    uint8_t best_index = 0u;

    if (result == NULL) {
        return ARM_IK_INVALID_ARGUMENT;
    }
    memset(result, 0, sizeof(*result));
    memset(candidates, 0, sizeof(candidates));
    status = ArmInverseKinematicsToolCenterAll(
        target_center_mm, tool_pitch_deg, seed_q_deg,
        candidates, &candidate_count);
    result->status = status;
    result->candidate_count = candidate_count;
    if (status != ARM_IK_OK) {
        return status;
    }
    for (uint8_t i = 0u; i < candidate_count; ++i) {
        float score = ArmKinematicsCandidateScore(candidates[i].q_deg,
                                                   seed_q_deg);
        if (!isfinite(score)) {
            result->status = ARM_IK_NUMERICAL_ERROR;
            return result->status;
        }
        if (i == 0u || score < best_score) {
            best_score = score;
            best_index = i;
        }
    }
    memcpy(result->q_deg, candidates[best_index].q_deg,
           sizeof(result->q_deg));
    result->wrist_center_mm = candidates[best_index].wrist_center_mm;
    result->tool_center_mm = candidates[best_index].tool_center_mm;
    result->position_error_mm = candidates[best_index].position_error_mm;
    result->status = ARM_IK_OK;
    return result->status;
}
