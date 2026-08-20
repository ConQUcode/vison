/**
 * @file arm_trajectory.c
 * @brief 对整条路径先做 IK/限位预检，再用五次曲线执行连续轨迹。
 */

#include "arm_trajectory.h"

#include "arm_config.h"
#include "arm_internal.h"
#include "arm_kinematics.h"
#include "arm_path_planner.h"
#include "arm_tool.h"
#include "dmmotor.h"
#include "stm32f4xx_hal.h"
#include "math.h"
#include "string.h"

/*
 * 轨迹层只接收关节角或腕部轴心坐标，不直接决定电机是否初始化成功。
 * 空间直线在启动前按配置间隔完整预检并缓存IK解；执行时按配置周期
 * 根据当前笛卡尔插值点重新求IK，1ms任务在相邻两组合法解之间插值。
 * 预检采样密度不决定轨迹时间，时间参数由空间/关节速度及加速度共同决定。
 * 五次曲线10t^3-15t^4+6t^5保证起止速度和加速度均为0。
 */

#define ARM_LINEAR_MIN_DISTANCE_MM         0.01f
#define ARM_LINEAR_MIN_SPEED_MM_S          1.0f
#define ARM_LINEAR_MIN_ACCEL_MM_S2         1.0f
#define ARM_CARTESIAN_PI                   3.14159265358979323846f
#define ARM_LINEAR_QUINTIC_PEAK_FACTOR     1.875f
#define ARM_LINEAR_QUINTIC_ACCEL_FACTOR    5.7735f

#define ARM_COMPOSITE_JOINT_STEP_DEG        1.0f
#define ARM_SAMPLE_PROGRESS_EPSILON          0.000001f
#define ARM_COMPOSITE_BLEND_JOINT_INTERVALS 12u
#define ARM_COMPOSITE_BLEND_LINEAR_INTERVALS 12u

typedef enum {
    ARM_TRAJECTORY_PATH_NONE = 0,
    ARM_TRAJECTORY_PATH_JOINT_STAGING,
    ARM_TRAJECTORY_PATH_CARTESIAN_LINEAR
} Arm_Trajectory_Path_e;

typedef struct {
    uint32_t state_start_tick;
    uint32_t trajectory_start_tick;
    uint32_t last_task_tick;
    uint32_t trajectory_duration_ms;
    uint16_t sample_count;
    uint8_t kinematics_self_test_passed;
    uint8_t reference_update_rejected;
    uint8_t online_ik_valid;
    uint8_t realtime_active;
    uint8_t realtime_timed_out;
    uint8_t active_tool_pitch_valid;
    uint8_t active_tool_pitch_relative_mode;
    uint8_t safety_route_enabled;
    uint8_t route_segment_count;
    uint8_t route_active_segment;
    uint8_t route_crossing_segment;
    uint8_t route_waiting_for_arrival;
    Arm_Cartesian_Safety_Profile_e safety_profile;
    Arm_Trajectory_Path_e path_type;
    Arm_Position_s start_position;
    Arm_Position_s target_position;
    Arm_Position_s realtime_reference_position;
    Arm_Position_s realtime_last_valid_position;
    Arm_Position_s realtime_reference_tool_tip;
    Arm_Position_s realtime_last_valid_tool_tip;
    Arm_Realtime_Cartesian_Target_s realtime_target;
    Arm_Control_Point_e realtime_control_point;
    Arm_Control_Point_e active_control_point;
    Arm_Position_s active_target_tool_tip;
    Arm_Position_s active_target_wrist;
    uint32_t last_ik_tick;
    uint32_t online_ik_interval_start_tick;
    uint32_t online_ik_interval_ms;
    uint32_t realtime_last_command_tick;
    uint32_t arrival_stable_tick;
    float commanded_speed_mm_s;
    float commanded_accel_mm_s2;
    float path_length_mm;
    float active_tool_pitch_deg;
    float active_tool_relative_pitch_deg;
    float online_ik_previous_q_deg[3];
    float online_ik_next_q_deg[3];
    float realtime_command_q_deg[3];
    float realtime_velocity_mm_s[3];
    float realtime_last_velocity_mm_s[3];
    float initial_target_error_deg[3];
    uint8_t target_error_valid[3];
    uint8_t target_crossed[3];
    uint16_t route_segment_start[ARM_TRAJECTORY_MAX_ROUTE_SEGMENTS];
    uint16_t route_segment_end[ARM_TRAJECTORY_MAX_ROUTE_SEGMENTS];
    uint32_t route_segment_duration_ms[ARM_TRAJECTORY_MAX_ROUTE_SEGMENTS];
    Arm_Position_s route_segment_start_position[ARM_TRAJECTORY_MAX_ROUTE_SEGMENTS];
    Arm_Position_s route_segment_target_position[ARM_TRAJECTORY_MAX_ROUTE_SEGMENTS];
    float sample_q_deg[ARM_LINEAR_MAX_SAMPLES][3];
    float sample_progress[ARM_LINEAR_MAX_SAMPLES];
    /*
     * 工具中心分段预检的动态规划前驱表。每点最多4个候选，只保存前驱
     * 编号；回填时按相同几何重新生成候选，额外CCM占用约6KB。
     */
    uint8_t tool_candidate_predecessor[ARM_LINEAR_MAX_SAMPLES]
                                         [ARM_TOOL_CENTER_IK_MAX_CANDIDATES];
    uint8_t tool_candidate_count[ARM_LINEAR_MAX_SAMPLES];
    uint8_t tool_selected_candidate[ARM_LINEAR_MAX_SAMPLES];
} Arm_Cartesian_Runtime_s;

Arm_Motion_Debug_s g_arm_motion_debug;
/*
 * 1536点轨迹缓存约25KB，只由Cortex-M4内核访问，不交给CAN/USART DMA。
 * ARMCC将其固定放入F407的64KB CCM RAM，避免挤占普通SRAM中的RTOS堆栈；
 * GCC分支用于严格语法检查及未来GNU链接脚本的同名段适配。
 */
#if defined(__CC_ARM)
__attribute__((at(0x10000000)))
#else
__attribute__((section(".ccmram")))
#endif
static Arm_Cartesian_Runtime_s arm_cartesian_runtime;

static uint8_t ArmCartesianAcSidePickActive(void)
{
    return arm_cartesian_runtime.safety_profile ==
           ARM_CARTESIAN_SAFETY_AC_SIDE_PICK;
}

static uint8_t ArmCartesianAutoPoseIsSafe(const float q_deg[3])
{
    if (ArmCartesianAcSidePickActive() != 0u) {
        return ArmAutoPoseIsSafeWithQ1Limits(
            q_deg, ARM_AC_SIDE_PICK_Q1_MIN_DEG,
            ARM_AC_SIDE_PICK_Q1_MAX_DEG);
    }
    return ArmAutoPoseIsSafe(q_deg);
}

static Arm_IK_Status_e ArmCartesianInverseToolCenter(
    const Arm_Position_s *target_center_mm,
    float tool_pitch_deg,
    const float seed_q_deg[3],
    Arm_Tool_Center_IK_Result_s *result)
{
    if (ArmCartesianAcSidePickActive() != 0u) {
        return ArmInverseKinematicsToolCenterWithQ1Limits(
            target_center_mm, tool_pitch_deg, seed_q_deg,
            ARM_AC_SIDE_PICK_Q1_MIN_DEG, ARM_AC_SIDE_PICK_Q1_MAX_DEG,
            result);
    }
    return ArmInverseKinematicsToolCenter(
        target_center_mm, tool_pitch_deg, seed_q_deg, result);
}

static void ArmCartesianRecordRejected(Arm_IK_Status_e ik_status);
static uint8_t ArmCartesianJointStepContinuous(
    const float previous_q_deg[3], const float next_q_deg[3]);

/*
 * 工具中心路径预检是同步计算，复杂跨区路径可能持续超过100 ms。
 * 末端舵机的反馈轮询也由ArmControlTask拥有，因此必须在采样期间推进
 * 非阻塞通信状态机，避免有效通信被误判为反馈过期。这里只服务通信和
 * 既有工具状态，不修改主臂参考，也不会提前发送尚未通过预检的轨迹。
 */
static void ArmCartesianPreflightServiceTool(void *context)
{
    (void)context;
    ArmToolTask(HAL_GetTick());
    g_arm_motion_debug.preflight_tool_service_count++;
}

static float ArmCartesianClamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float ArmCartesianWrapTo180(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

/*
 * 前栏框限制取决于底座朝向，不能只看工具中心的世界坐标X。
 * 底座转到约180deg后，连杆径向量可能再次让世界X为正，但此时机械臂
 * 实际朝向后方，不应误触发只针对正前方栏框的大臂角度限制。
 */
#if ARM_WORKSPACE_SAFETY_ENABLE != 0u && \
    ARM_FRONT_BARRIER_SHOULDER_LIMIT_ENABLE != 0u
static uint8_t ArmWorkspaceBaseFacesFront(float q1_deg)
{
    return fabsf(ArmCartesianWrapTo180(q1_deg)) <=
        ARM_FRONT_BARRIER_BASE_Q1_ABS_MAX_DEG +
            ARM_LIMIT_TOLERANCE_DEG;
}
#endif

static float ArmCartesianPositionDistance(const Arm_Position_s *a,
                                          const Arm_Position_s *b)
{
    float dx = b->x_mm - a->x_mm;
    float dy = b->y_mm - a->y_mm;
    float dz = b->z_mm - a->z_mm;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

static uint8_t ArmCartesianResolveToolPitch(uint8_t requested_valid,
                                            float requested_pitch_deg)
{
    const Arm_Tool_State_s *tool = ArmToolGetState();
    float pitch_deg = requested_pitch_deg;

    if (requested_valid == 0u) {
        /* 笛卡尔命令未指定俯仰时，锁存进入命令时的世界绝对俯仰。 */
        if (tool->servo_feedback_valid[0] == 0u ||
            !isfinite(tool->tool_pitch_feedback_deg)) {
            return 0u;
        }
        pitch_deg = tool->tool_pitch_feedback_deg;
    }
    if (!isfinite(pitch_deg) ||
        pitch_deg < ARM_USB_TOOL_PITCH_MIN_DEG ||
        pitch_deg > ARM_USB_TOOL_PITCH_MAX_DEG) {
        return 0u;
    }
    arm_cartesian_runtime.active_tool_pitch_deg = pitch_deg;
    arm_cartesian_runtime.active_tool_pitch_relative_mode = 0u;
    arm_cartesian_runtime.active_tool_pitch_valid = 1u;
    return 1u;
}

/*
 * 普通关节动作不要求工具保持世界水平。进入动作时锁存ID1相对小臂角，
 * 后续关节变化只保持该机械相对角，避免世界绝对俯仰补偿把ID1推到限位。
 */
static uint8_t ArmCartesianResolveRelativeToolPitch(void)
{
    const Arm_Tool_State_s *tool = ArmToolGetState();
    float small_link_pitch_deg =
        ArmToolSmallLinkPitchFromJoint(g_arm_state.q_feedback_deg);
    float relative_pitch_deg;

    if (tool->servo_feedback_valid[0] == 0u ||
        !isfinite(tool->tool_pitch_feedback_deg) ||
        !isfinite(small_link_pitch_deg)) {
        return 0u;
    }
    relative_pitch_deg = tool->tool_pitch_feedback_deg -
        small_link_pitch_deg;
    if (!isfinite(relative_pitch_deg) ||
        relative_pitch_deg < ARM_TOOL_PITCH_RELATIVE_MIN_DEG ||
        relative_pitch_deg > ARM_TOOL_PITCH_RELATIVE_MAX_DEG) {
        return 0u;
    }
    arm_cartesian_runtime.active_tool_relative_pitch_deg =
        relative_pitch_deg;
    arm_cartesian_runtime.active_tool_pitch_deg =
        tool->tool_pitch_feedback_deg;
    arm_cartesian_runtime.active_tool_pitch_relative_mode = 1u;
    arm_cartesian_runtime.active_tool_pitch_valid = 1u;
    return 1u;
}

/*
 * 抓放固定姿态可显式指定ID1相对小臂角度，使ID1与三台达妙共用同一
 * 关节轨迹时间轴。普通关节命令仍锁存动作开始时的相对角，行为不变。
 */
static uint8_t ArmCartesianResolveCommandRelativeToolPitch(
    uint8_t requested_valid, float requested_relative_pitch_deg)
{
    if (requested_valid == 0u) {
        return ArmCartesianResolveRelativeToolPitch();
    }
    if (!isfinite(requested_relative_pitch_deg) ||
        requested_relative_pitch_deg < ARM_TOOL_PITCH_RELATIVE_MIN_DEG ||
        requested_relative_pitch_deg > ARM_TOOL_PITCH_RELATIVE_MAX_DEG) {
        return 0u;
    }
    arm_cartesian_runtime.active_tool_relative_pitch_deg =
        requested_relative_pitch_deg;
    arm_cartesian_runtime.active_tool_pitch_deg =
        ArmToolSmallLinkPitchFromJoint(g_arm_state.q_feedback_deg) +
        requested_relative_pitch_deg;
    arm_cartesian_runtime.active_tool_pitch_relative_mode = 1u;
    arm_cartesian_runtime.active_tool_pitch_valid = 1u;
    return isfinite(arm_cartesian_runtime.active_tool_pitch_deg);
}

static float ArmCartesianToolPitchForQ(const float q_deg[3])
{
    if (q_deg == NULL ||
        arm_cartesian_runtime.active_tool_pitch_valid == 0u) {
        return NAN;
    }
    if (arm_cartesian_runtime.active_tool_pitch_relative_mode != 0u) {
        return ArmToolSmallLinkPitchFromJoint(q_deg) +
            arm_cartesian_runtime.active_tool_relative_pitch_deg;
    }
    return arm_cartesian_runtime.active_tool_pitch_deg;
}

static uint8_t ArmCartesianToolPitchAllowedForQ(const float q_deg[3])
{
    float pitch_deg = ArmCartesianToolPitchForQ(q_deg);

    return isfinite(pitch_deg) &&
           ArmToolPitchValidForPose(pitch_deg, q_deg);
}

/* X<-2mm才进入后区；X在+/-2mm内视为侧面边界。 */
static uint8_t ArmWorkspacePointIsRear(const Arm_Position_s *point)
{
#if ARM_WORKSPACE_SAFETY_ENABLE != 0u
    /* AC侧抓由q1的+/-115deg边界保护，不把X略小于0误判成车后方。 */
    if (ArmCartesianAcSidePickActive() != 0u) {
        return 0u;
    }
    return point != NULL &&
           point->x_mm < ARM_REAR_ZONE_X_BOUNDARY_MM -
                         ARM_REAR_ZONE_X_MARGIN_MM;
#else
    (void)point;
    return 0u;
#endif
}

static uint8_t ArmWorkspaceCrossesBoundary(const Arm_Position_s *start,
                                           const Arm_Position_s *target)
{
#if ARM_WORKSPACE_SAFETY_ENABLE != 0u
    uint8_t start_rear;
    uint8_t target_rear;

    if (start == NULL || target == NULL) {
        return 0u;
    }
    start_rear = ArmWorkspacePointIsRear(start);
    target_rear = ArmWorkspacePointIsRear(target);
    return start_rear != target_rear;
#else
    (void)start;
    (void)target;
    return 0u;
#endif
}

static uint8_t ArmWorkspacePointSafe(const Arm_Position_s *tool_center,
                                     uint8_t target_point)
{
#if ARM_WORKSPACE_SAFETY_ENABLE != 0u
    if (tool_center == NULL || !isfinite(tool_center->x_mm) ||
        !isfinite(tool_center->y_mm) || !isfinite(tool_center->z_mm)) {
        g_arm_motion_debug.workspace_safety_result =
            ARM_WORKSPACE_SAFETY_PREFLIGHT_IK;
        return 0u;
    }
    if (ArmWorkspacePointIsRear(tool_center) &&
        tool_center->z_mm < ARM_REAR_ZONE_MIN_TOOL_Z_MM) {
        g_arm_motion_debug.workspace_safety_result = target_point != 0u ?
            ARM_WORKSPACE_SAFETY_TARGET_REAR_TOO_LOW :
            ARM_WORKSPACE_SAFETY_PATH_REAR_TOO_LOW;
        return 0u;
    }
#else
    (void)tool_center;
    (void)target_point;
#endif
    return 1u;
}

static uint8_t ArmWorkspacePoseSafe(const float q_deg[3],
                                    uint8_t target_point,
                                    Arm_Position_s *tool_center)
{
    Arm_Position_s local_center;

    if (!ArmForwardKinematicsToolCenter(
            q_deg, ArmCartesianToolPitchForQ(q_deg),
            &local_center) || !ArmWorkspacePointSafe(&local_center,
                                                      target_point)) {
        return 0u;
    }
#if ARM_WORKSPACE_SAFETY_ENABLE != 0u && \
    ARM_FRONT_BARRIER_SHOULDER_LIMIT_ENABLE != 0u
    /*
     * 正X前方仍有栏框时严格限制大臂；规划器必须先把工具中心绕到
     * X=0侧面，再在侧面完成径向分支转换，不能以“高位”为理由豁免。
     */
    if (ArmWorkspaceBaseFacesFront(q_deg[ARM_JOINT_BASE_YAW]) &&
        local_center.x_mm > ARM_FRONT_BARRIER_TOOL_X_MARGIN_MM &&
        q_deg[ARM_JOINT_SHOULDER] >
            ARM_FRONT_BARRIER_SHOULDER_Q2_MAX_DEG) {
        g_arm_motion_debug.workspace_safety_result =
            ARM_WORKSPACE_SAFETY_FRONT_SHOULDER_LIMIT;
        return 0u;
    }
#endif
    if (tool_center != NULL) {
        *tool_center = local_center;
    }
    return 1u;
}

static uint8_t ArmWorkspaceJointPathSafe(const float start_q_deg[3],
                                         const float target_q_deg[3])
{
    float max_delta_deg = 0.0f;
    uint16_t intervals;
    Arm_Position_s previous_center;

    if (!ArmWorkspacePoseSafe(start_q_deg, 0u, &previous_center)) {
        return 0u;
    }
    for (uint8_t joint = 0u; joint < 3u; ++joint) {
        float delta = joint == ARM_JOINT_BASE_YAW ?
            fabsf(ArmCartesianWrapTo180(target_q_deg[joint] -
                                        start_q_deg[joint])) :
            fabsf(target_q_deg[joint] - start_q_deg[joint]);
        max_delta_deg = fmaxf(max_delta_deg, delta);
    }
    intervals = (uint16_t)ceilf(max_delta_deg /
                                ARM_COMPOSITE_JOINT_STEP_DEG);
    if (intervals < 1u) {
        intervals = 1u;
    }
    if ((uint32_t)intervals + 1u > ARM_LINEAR_MAX_SAMPLES) {
        g_arm_motion_debug.workspace_safety_result =
            ARM_WORKSPACE_SAFETY_SAMPLE_CAPACITY;
        return 0u;
    }
    for (uint16_t i = 1u; i <= intervals; ++i) {
        float ratio = (float)i / (float)intervals;
        float q[3];
        Arm_Position_s center;

        q[0] = ArmCartesianWrapTo180(start_q_deg[0] + ratio *
            ArmCartesianWrapTo180(target_q_deg[0] - start_q_deg[0]));
        q[1] = start_q_deg[1] + ratio *
            (target_q_deg[1] - start_q_deg[1]);
        q[2] = start_q_deg[2] + ratio *
            (target_q_deg[2] - start_q_deg[2]);
        if (!ArmJointPoseWithinSoftLimits(q) ||
            !ArmCartesianToolPitchAllowedForQ(q) ||
            !ArmWorkspacePoseSafe(q, i == intervals ? 1u : 0u, &center)) {
            return 0u;
        }
        if (ArmWorkspaceCrossesBoundary(&previous_center, &center) &&
            fminf(previous_center.z_mm, center.z_mm) <
                ARM_REAR_CROSSING_TOOL_Z_MM) {
            g_arm_motion_debug.workspace_safety_result =
                ARM_WORKSPACE_SAFETY_CROSSING_TOO_LOW;
            return 0u;
        }
        previous_center = center;
    }
    return 1u;
}

/*
 * 对已经生成并完成圆角处理的关节样本做最终安全审计。
 * 该检查必须位于所有会改写样本的处理之后，防止复合轨迹的局部
 * Bezier 圆角绕过夹爪中心高度、跨区高度或 ID1 俯仰限位检查。
 */
static uint8_t ArmWorkspaceSampleBufferSafe(uint16_t sample_count)
{
    Arm_Position_s previous_center;

    if (sample_count == 0u || sample_count > ARM_LINEAR_MAX_SAMPLES) {
        g_arm_motion_debug.workspace_safety_result =
            ARM_WORKSPACE_SAFETY_SAMPLE_CAPACITY;
        return 0u;
    }
    if (!ArmWorkspacePoseSafe(arm_cartesian_runtime.sample_q_deg[0],
                              sample_count == 1u ? 1u : 0u,
                              &previous_center)) {
        return 0u;
    }
    for (uint16_t i = 1u; i < sample_count; ++i) {
        const float *sample_q_deg = arm_cartesian_runtime.sample_q_deg[i];
        Arm_Position_s center;

        if (!ArmJointPoseWithinSoftLimits(sample_q_deg) ||
            !ArmCartesianAutoPoseIsSafe(sample_q_deg) ||
            !ArmCartesianToolPitchAllowedForQ(sample_q_deg) ||
            !ArmCartesianJointStepContinuous(
                arm_cartesian_runtime.sample_q_deg[i - 1u],
                sample_q_deg) ||
            !ArmWorkspacePoseSafe(sample_q_deg,
                                  i + 1u == sample_count ? 1u : 0u,
                                  &center)) {
            return 0u;
        }
        if (ArmWorkspaceCrossesBoundary(&previous_center, &center) &&
            fminf(previous_center.z_mm, center.z_mm) <
                ARM_REAR_CROSSING_TOOL_Z_MM) {
            g_arm_motion_debug.workspace_safety_result =
                ARM_WORKSPACE_SAFETY_CROSSING_TOO_LOW;
            return 0u;
        }
        previous_center = center;
    }
    return 1u;
}

static Arm_IK_Status_e ArmCartesianSolveControlPoint(
    Arm_Control_Point_e control_point,
    const Arm_Position_s *target,
    const float seed_q_deg[3],
    Arm_IK_Result_s *result,
    Arm_Position_s *wrist_center)
{
    if (result == NULL) {
        return ARM_IK_INVALID_ARGUMENT;
    }
    memset(result, 0, sizeof(*result));
    if (control_point == ARM_CONTROL_POINT_WRIST_CENTER) {
        result->status = ArmInverseKinematics3DOF(target, seed_q_deg, result);
        if (wrist_center != NULL && result->status == ARM_IK_OK) {
            *wrist_center = result->fk_position;
        }
        return result->status;
    }
    if (control_point == ARM_CONTROL_POINT_TOOL_CENTER) {
        Arm_Tool_Center_IK_Result_s tool_ik;

        memset(&tool_ik, 0, sizeof(tool_ik));
        result->status = ArmCartesianInverseToolCenter(
            target, arm_cartesian_runtime.active_tool_pitch_deg,
            seed_q_deg, &tool_ik);
        result->candidate_count = tool_ik.candidate_count;
        memcpy(result->q_deg, tool_ik.q_deg, sizeof(result->q_deg));
        result->fk_position = tool_ik.wrist_center_mm;
        result->position_error_mm = tool_ik.position_error_mm;
        if (wrist_center != NULL && result->status == ARM_IK_OK) {
            *wrist_center = tool_ik.wrist_center_mm;
        }
        return result->status;
    }
    result->status = ARM_IK_INVALID_ARGUMENT;
    return result->status;
}

static void ArmCartesianResetSafetyRoute(void)
{
    arm_cartesian_runtime.safety_route_enabled = 0u;
    arm_cartesian_runtime.route_segment_count = 0u;
    arm_cartesian_runtime.route_active_segment = 0u;
    arm_cartesian_runtime.route_crossing_segment = 0xFFu;
    arm_cartesian_runtime.route_waiting_for_arrival = 0u;
    g_arm_motion_debug.safety_route_enabled = 0u;
    g_arm_motion_debug.safety_route_segment_count = 0u;
    g_arm_motion_debug.safety_route_active_segment = 0u;
    g_arm_motion_debug.workspace_safety_result = ARM_WORKSPACE_SAFETY_OK;
}

static uint32_t ArmCartesianRouteSegmentDurationMs(
    uint16_t first_index, uint16_t last_index, float path_length_mm,
    float max_speed_mm_s)
{
    float minimum_s = 0.001f;
    const float joint_speed_deg_s[3] = {
        ARM_LINEAR_Q1_MAX_SPEED_DEG_S,
        ARM_LINEAR_Q2_MAX_SPEED_DEG_S,
        ARM_LINEAR_Q3_MAX_SPEED_DEG_S
    };
    const float joint_accel_deg_s2[3] = {
        ARM_LINEAR_Q1_MAX_ACCEL_DEG_S2,
        ARM_LINEAR_Q2_MAX_ACCEL_DEG_S2,
        ARM_LINEAR_Q3_MAX_ACCEL_DEG_S2
    };

    if (path_length_mm > ARM_LINEAR_MIN_DISTANCE_MM) {
        minimum_s = fmaxf(minimum_s,
            ARM_LINEAR_QUINTIC_PEAK_FACTOR * path_length_mm /
                max_speed_mm_s);
        minimum_s = fmaxf(minimum_s, sqrtf(
            ARM_LINEAR_QUINTIC_ACCEL_FACTOR * path_length_mm /
                ARM_LINEAR_MAX_ACCEL_MM_S2));
    }
    for (uint8_t joint = 0u; joint < 3u; ++joint) {
        float travel_deg = 0.0f;

        for (uint16_t i = (uint16_t)(first_index + 1u);
             i <= last_index; ++i) {
            travel_deg += joint == ARM_JOINT_BASE_YAW ?
                fabsf(ArmCartesianWrapTo180(
                    arm_cartesian_runtime.sample_q_deg[i][joint] -
                    arm_cartesian_runtime.sample_q_deg[i - 1u][joint])) :
                fabsf(arm_cartesian_runtime.sample_q_deg[i][joint] -
                      arm_cartesian_runtime.sample_q_deg[i - 1u][joint]);
        }
        minimum_s = fmaxf(minimum_s,
            ARM_LINEAR_QUINTIC_PEAK_FACTOR * travel_deg /
                joint_speed_deg_s[joint]);
        minimum_s = fmaxf(minimum_s, sqrtf(
            ARM_LINEAR_QUINTIC_ACCEL_FACTOR * travel_deg /
                joint_accel_deg_s2[joint]));
    }
    return (uint32_t)(minimum_s * 1000.0f + 0.999f);
}

static uint8_t ArmCartesianAppendToolCenterSegment(
    const Arm_Position_s *start_center,
    const Arm_Position_s *target_center,
    float previous_q_deg[3],
    uint8_t segment_index,
    float max_speed_mm_s,
    Arm_Cartesian_Safety_Profile_e safety_profile)
{
    float length_mm = ArmCartesianPositionDistance(start_center,
                                                   target_center);
    Arm_Path_Plan_Request_s request;
    Arm_Path_Plan_Workspace_s workspace;
    Arm_Path_Plan_Result_s result;
    uint16_t first_index;

    if (segment_index >= ARM_TRAJECTORY_MAX_ROUTE_SEGMENTS ||
        arm_cartesian_runtime.sample_count == 0u ||
        arm_cartesian_runtime.sample_count > ARM_LINEAR_MAX_SAMPLES) {
        g_arm_motion_debug.workspace_safety_result =
            ARM_WORKSPACE_SAFETY_SAMPLE_CAPACITY;
        return 0u;
    }
    first_index = (uint16_t)(arm_cartesian_runtime.sample_count - 1u);
    arm_cartesian_runtime.route_segment_start[segment_index] = first_index;
    arm_cartesian_runtime.route_segment_start_position[segment_index] =
        *start_center;
    arm_cartesian_runtime.route_segment_target_position[segment_index] =
        *target_center;
    memset(&request, 0, sizeof(request));
    request.start_center_mm = *start_center;
    request.target_center_mm = *target_center;
    memcpy(request.start_q_deg, previous_q_deg,
           sizeof(request.start_q_deg));
    request.tool_pitch_deg = arm_cartesian_runtime.active_tool_pitch_deg;
    request.sample_spacing_mm = ARM_LINEAR_SAMPLE_SPACING_MM;
    request.safety_profile = safety_profile;
    request.segment_index = segment_index;
    request.service_hook = ArmCartesianPreflightServiceTool;

    workspace.sample_q_deg =
        &arm_cartesian_runtime.sample_q_deg[first_index];
    workspace.sample_progress =
        &arm_cartesian_runtime.sample_progress[first_index];
    workspace.candidate_predecessor =
        &arm_cartesian_runtime.tool_candidate_predecessor[first_index];
    workspace.candidate_count =
        &arm_cartesian_runtime.tool_candidate_count[first_index];
    workspace.selected_candidate =
        &arm_cartesian_runtime.tool_selected_candidate[first_index];
    workspace.capacity = (uint16_t)(ARM_LINEAR_MAX_SAMPLES - first_index);

    if (!ArmPathPlanToolCenterSegment(&request, &workspace, &result)) {
        g_arm_motion_debug.preflight_failed_segment = result.failed_segment;
        g_arm_motion_debug.preflight_failed_sample = result.failed_sample;
        g_arm_motion_debug.preflight_failed_check_mask =
            result.failed_check_mask;
        g_arm_motion_debug.preflight_failed_center_mm =
            result.failed_center_mm;
        memcpy(g_arm_motion_debug.preflight_failed_q_deg,
               result.failed_q_deg,
               sizeof(g_arm_motion_debug.preflight_failed_q_deg));
        g_arm_motion_debug.workspace_safety_result =
            result.workspace_safety_result;
        ArmCartesianRecordRejected(
            result.ik_status == ARM_IK_OK ?
                ARM_IK_COLLISION_RISK : result.ik_status);
        return 0u;
    }
    memcpy(previous_q_deg, workspace.sample_q_deg[result.interval_count],
           sizeof(request.start_q_deg));
    arm_cartesian_runtime.sample_count =
        (uint16_t)(arm_cartesian_runtime.sample_count +
                   result.interval_count);
    arm_cartesian_runtime.route_segment_end[segment_index] =
        (uint16_t)(arm_cartesian_runtime.sample_count - 1u);
    arm_cartesian_runtime.route_segment_duration_ms[segment_index] =
        ArmCartesianRouteSegmentDurationMs(
            first_index,
            arm_cartesian_runtime.route_segment_end[segment_index],
            length_mm, max_speed_mm_s);
    return 1u;
}

static Arm_Motion_Fault_e ArmCartesianTrackActiveToolPitch(
    const float reference_q_deg[3], uint32_t now_ms)
{
    const Arm_Tool_State_s *tool = ArmToolGetState();
    Arm_Command_Result_e result;
    float pitch_deg = ArmCartesianToolPitchForQ(reference_q_deg);

    if (!isfinite(pitch_deg) ||
        !ArmCartesianToolPitchAllowedForQ(reference_q_deg)) {
        return ARM_MOTION_FAULT_LIMIT;
    }
    /*
     * 单次位置查询延迟不应让整条主臂轨迹立即停住。路径开始前已经要求
     * ID1反馈有效；运行中只在最近反馈超过宽限时间后报告真正的离线。
     */
    if (tool->servo_feedback_valid[0] == 0u &&
        (tool->servo_last_feedback_tick[0] == 0u ||
         (uint32_t)(now_ms - tool->servo_last_feedback_tick[0]) >
             ARM_TOOL_PITCH_FEEDBACK_ABORT_MS)) {
        return ARM_MOTION_FAULT_OFFLINE;
    }
    result = ArmToolTrackPitch(
        pitch_deg,
        ArmToolSmallLinkPitchFromJoint(reference_q_deg), now_ms);
    if (result == ARM_COMMAND_OK) {
        arm_cartesian_runtime.active_tool_pitch_deg = pitch_deg;
        return ARM_MOTION_FAULT_NONE;
    }
    if (result == ARM_COMMAND_INVALID ||
        result == ARM_COMMAND_PREFLIGHT_FAILED) {
        return ARM_MOTION_FAULT_LIMIT;
    }
    return ARM_MOTION_FAULT_OFFLINE;
}

static float ArmCartesianQuintic(float normalized_time)
{
    float t = ArmCartesianClamp(normalized_time, 0.0f, 1.0f);
    float t2 = t * t;
    float t3 = t2 * t;

    return t3 * (10.0f + t * (-15.0f + 6.0f * t));
}

static float ArmCartesianQuinticVelocity(float normalized_time)
{
    float t = ArmCartesianClamp(normalized_time, 0.0f, 1.0f);
    float one_minus_t = 1.0f - t;

    return 30.0f * t * t * one_minus_t * one_minus_t;
}

static float ArmCartesianQuinticAcceleration(float normalized_time)
{
    float t = ArmCartesianClamp(normalized_time, 0.0f, 1.0f);

    return 60.0f * t * (1.0f - t) * (1.0f - 2.0f * t);
}

static uint8_t ArmCartesianMotorsReady(void)
{
    uint8_t control_mode_ready = g_arm_state.mode == ARM_MODE_READY;

#if ARM_BOOT_MODE == ARM_BOOT_MODE_DM_SINGLE_AXIS_TEST
    control_mode_ready = control_mode_ready ||
        g_arm_state.mode == ARM_MODE_DM_SINGLE_AXIS_TEST;
#endif
    return g_arm_state.config_valid && g_arm_state.kinematics_valid &&
           g_arm_state.all_targets_synced &&
           g_arm_state.start_state == ARM_START_READY &&
           control_mode_ready &&
           g_arm_state.fault_latched == ARM_FAULT_NONE &&
           g_arm_state.motor_online[0] && g_arm_state.motor_online[1] &&
           g_arm_state.motor_online[2];
}

static void ArmCartesianSetState(Arm_Motion_State_e state, uint32_t now_ms)
{
    g_arm_motion_debug.motion_state = state;
    arm_cartesian_runtime.state_start_tick = now_ms;
    if (state != ARM_MOTION_SETTLING) {
        arm_cartesian_runtime.arrival_stable_tick = 0u;
    }
}

static uint8_t ArmCartesianFeedbackWithinArrivalLimits(
    float error_limit_deg, float speed_limit_deg_s)
{
    for (uint8_t joint = 0u; joint < 3u; ++joint) {
        float error_deg;
        float speed_deg_s = g_arm_state.motor_speed_dps[joint];

        if (!isfinite(g_arm_motion_debug.target_q_deg[joint]) ||
            !isfinite(g_arm_state.q_feedback_deg[joint]) ||
            !isfinite(speed_deg_s)) {
            return 0u;
        }
        error_deg = joint == ARM_JOINT_BASE_YAW ?
            ArmCartesianWrapTo180(
                g_arm_motion_debug.target_q_deg[joint] -
                g_arm_state.q_feedback_deg[joint]) :
            g_arm_motion_debug.target_q_deg[joint] -
                g_arm_state.q_feedback_deg[joint];
        if (fabsf(error_deg) > error_limit_deg ||
            fabsf(speed_deg_s) > speed_limit_deg_s) {
            return 0u;
        }
    }
    return 1u;
}

static void ArmCartesianResetControlStatistics(void)
{
    memset(g_arm_control_debug.peak_tracking_error_deg, 0,
           sizeof(g_arm_control_debug.peak_tracking_error_deg));
    memset(g_arm_control_debug.overshoot_deg, 0,
           sizeof(g_arm_control_debug.overshoot_deg));
    memset(g_arm_control_debug.tracking_error_warning, 0,
           sizeof(g_arm_control_debug.tracking_error_warning));
    memset(g_arm_control_debug.tracking_error_duration_ms, 0,
           sizeof(g_arm_control_debug.tracking_error_duration_ms));
    memset(arm_cartesian_runtime.target_error_valid, 0,
           sizeof(arm_cartesian_runtime.target_error_valid));
    memset(arm_cartesian_runtime.target_crossed, 0,
           sizeof(arm_cartesian_runtime.target_crossed));
}

static void ArmCartesianUpdateControlDebug(uint32_t now_ms,
                                           uint32_t task_delta_ms)
{
    float duration_s =
        (float)arm_cartesian_runtime.trajectory_duration_ms * 0.001f;
    float normalized_time = duration_s > 0.0f ?
        (float)g_arm_motion_debug.trajectory_elapsed_ms /
            (float)arm_cartesian_runtime.trajectory_duration_ms : 1.0f;

    memcpy(g_arm_control_debug.current_q_deg, g_arm_state.q_feedback_deg,
           sizeof(g_arm_control_debug.current_q_deg));
    memcpy(g_arm_control_debug.reference_q_deg,
           g_arm_motion_debug.trajectory_q_deg,
           sizeof(g_arm_control_debug.reference_q_deg));
    memcpy(g_arm_control_debug.target_q_deg,
           g_arm_motion_debug.target_q_deg,
           sizeof(g_arm_control_debug.target_q_deg));
    memcpy(g_arm_control_debug.motor_speed_deg_s,
           g_arm_state.motor_speed_dps,
           sizeof(g_arm_control_debug.motor_speed_deg_s));
    memcpy(g_arm_control_debug.motor_current, g_arm_state.motor_current,
           sizeof(g_arm_control_debug.motor_current));

    for (uint8_t joint = 0u; joint < 3u; ++joint) {
        float tracking_error = joint == 0u ?
            ArmCartesianWrapTo180(
                g_arm_motion_debug.trajectory_q_deg[joint] -
                g_arm_state.q_feedback_deg[joint]) :
            g_arm_motion_debug.trajectory_q_deg[joint] -
                g_arm_state.q_feedback_deg[joint];
        float target_error = joint == 0u ?
            ArmCartesianWrapTo180(g_arm_motion_debug.target_q_deg[joint] -
                                  g_arm_state.q_feedback_deg[joint]) :
            g_arm_motion_debug.target_q_deg[joint] -
                g_arm_state.q_feedback_deg[joint];

        g_arm_control_debug.tracking_error_deg[joint] = tracking_error;
        if (fabsf(tracking_error) >
            g_arm_control_debug.peak_tracking_error_deg[joint]) {
            g_arm_control_debug.peak_tracking_error_deg[joint] =
                fabsf(tracking_error);
        }
        if (!arm_cartesian_runtime.target_error_valid[joint] &&
            fabsf(target_error) > 0.01f) {
            arm_cartesian_runtime.initial_target_error_deg[joint] =
                target_error;
            arm_cartesian_runtime.target_error_valid[joint] = 1u;
        }
        if (arm_cartesian_runtime.target_error_valid[joint] &&
            arm_cartesian_runtime.initial_target_error_deg[joint] *
                target_error < 0.0f) {
            arm_cartesian_runtime.target_crossed[joint] = 1u;
        }
        if (arm_cartesian_runtime.target_crossed[joint] &&
            fabsf(target_error) >
                g_arm_control_debug.overshoot_deg[joint]) {
            g_arm_control_debug.overshoot_deg[joint] =
                fabsf(target_error);
        }
        if (fabsf(tracking_error) > ARM_TRACKING_ERROR_WARN_DEG) {
            g_arm_control_debug.tracking_error_warning[joint] = 1u;
            g_arm_control_debug.tracking_error_duration_ms[joint] +=
                task_delta_ms;
        } else {
            g_arm_control_debug.tracking_error_warning[joint] = 0u;
            g_arm_control_debug.tracking_error_duration_ms[joint] = 0u;
        }
    }

    if (arm_cartesian_runtime.realtime_active) {
        float dv[3];
        float dt_s = (float)task_delta_ms * 0.001f;

        g_arm_control_debug.trajectory_speed_mm_s = sqrtf(
            arm_cartesian_runtime.realtime_velocity_mm_s[0] *
                arm_cartesian_runtime.realtime_velocity_mm_s[0] +
            arm_cartesian_runtime.realtime_velocity_mm_s[1] *
                arm_cartesian_runtime.realtime_velocity_mm_s[1] +
            arm_cartesian_runtime.realtime_velocity_mm_s[2] *
                arm_cartesian_runtime.realtime_velocity_mm_s[2]);
        dv[0] = arm_cartesian_runtime.realtime_velocity_mm_s[0] -
                arm_cartesian_runtime.realtime_last_velocity_mm_s[0];
        dv[1] = arm_cartesian_runtime.realtime_velocity_mm_s[1] -
                arm_cartesian_runtime.realtime_last_velocity_mm_s[1];
        dv[2] = arm_cartesian_runtime.realtime_velocity_mm_s[2] -
                arm_cartesian_runtime.realtime_last_velocity_mm_s[2];
        g_arm_control_debug.trajectory_acceleration_mm_s2 = dt_s > 0.0f ?
            sqrtf(dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]) /
                dt_s : 0.0f;
    } else {
        g_arm_control_debug.trajectory_speed_mm_s =
            duration_s > 0.0f ?
            arm_cartesian_runtime.path_length_mm *
                ArmCartesianQuinticVelocity(normalized_time) /
                duration_s : 0.0f;
        g_arm_control_debug.trajectory_acceleration_mm_s2 =
            duration_s > 0.0f ?
            arm_cartesian_runtime.path_length_mm *
                fabsf(ArmCartesianQuinticAcceleration(normalized_time)) /
                (duration_s * duration_s) : 0.0f;
    }
    g_arm_control_debug.trajectory_elapsed_ms =
        g_arm_motion_debug.trajectory_elapsed_ms;
    g_arm_control_debug.trajectory_duration_ms =
        arm_cartesian_runtime.trajectory_duration_ms;
    g_arm_control_debug.settling_time_ms =
        g_arm_motion_debug.motion_state == ARM_MOTION_SETTLING ?
        (uint32_t)(now_ms - arm_cartesian_runtime.state_start_tick) : 0u;
    g_arm_control_debug.arrival_stable_ms =
        g_arm_motion_debug.motion_state == ARM_MOTION_SETTLING &&
        arm_cartesian_runtime.arrival_stable_tick != 0u ?
        (uint32_t)(now_ms - arm_cartesian_runtime.arrival_stable_tick) : 0u;
    g_arm_control_debug.settling_timeout_ms =
        ARM_TRAJECTORY_SETTLE_TIMEOUT_MS;
    g_arm_control_debug.arrival_within_tolerance =
        ArmCartesianFeedbackWithinArrivalLimits(
            ARM_ARRIVAL_ERROR_DEG, ARM_ARRIVAL_SPEED_DEG_S);
    g_arm_control_debug.realtime_active =
        arm_cartesian_runtime.realtime_active;
    g_arm_control_debug.realtime_timed_out =
        arm_cartesian_runtime.realtime_timed_out;
    g_arm_control_debug.realtime_command_id =
        arm_cartesian_runtime.realtime_target.command_id;
    g_arm_control_debug.command_age_ms =
        arm_cartesian_runtime.realtime_active ?
        (uint32_t)(now_ms -
                   arm_cartesian_runtime.realtime_last_command_tick) : 0u;
    ArmUpdateControllerDebugSnapshot(&g_arm_control_debug);
}

static void ArmCartesianUpdateDebug(void)
{
    memcpy(g_arm_motion_debug.current_q_deg,
           g_arm_state.q_feedback_deg,
           sizeof(g_arm_motion_debug.current_q_deg));
    for (uint8_t i = 0u; i < 3u; ++i) {
        g_arm_motion_debug.joint_error_deg[i] = i == 0u ?
            ArmCartesianWrapTo180(g_arm_motion_debug.target_q_deg[i] -
                                  g_arm_state.q_feedback_deg[i]) :
            g_arm_motion_debug.target_q_deg[i] -
                g_arm_state.q_feedback_deg[i];
    }
    g_arm_motion_debug.current_position_mm =
        arm_cartesian_runtime.active_control_point ==
            ARM_CONTROL_POINT_TOOL_CENTER ?
            g_arm_state.tool_tip : g_arm_state.wrist_center;
    g_arm_motion_debug.position_error_mm = ArmCartesianPositionDistance(
        &g_arm_motion_debug.current_position_mm,
        &g_arm_motion_debug.target_position_mm);
    memcpy(g_arm_motion_debug.motor_online, g_arm_state.motor_online,
           sizeof(g_arm_motion_debug.motor_online));
    memcpy(g_arm_motion_debug.motor_enabled, g_arm_state.motor_enabled,
           sizeof(g_arm_motion_debug.motor_enabled));
    g_arm_motion_debug.trajectory_duration_ms =
        arm_cartesian_runtime.trajectory_duration_ms;
    g_arm_motion_debug.path_sample_count =
        arm_cartesian_runtime.sample_count;
}

static void ArmCartesianRecordRejected(Arm_IK_Status_e ik_status)
{
    g_arm_motion_debug.ik_status = ik_status;
    g_arm_motion_debug.path_preflight_passed = 0u;
    g_arm_motion_debug.command_accepted = 0u;
    g_arm_motion_debug.command_reject_count++;
}

static uint8_t ArmCartesianJointStepContinuous(const float previous_q_deg[3],
                                               const float next_q_deg[3])
{
    return fabsf(ArmCartesianWrapTo180(next_q_deg[0] - previous_q_deg[0])) <=
               ARM_LINEAR_Q1_STEP_MAX_DEG &&
           fabsf(next_q_deg[1] - previous_q_deg[1]) <=
               ARM_LINEAR_Q2_STEP_MAX_DEG &&
           fabsf(next_q_deg[2] - previous_q_deg[2]) <=
               ARM_LINEAR_Q3_STEP_MAX_DEG;
}

static float ArmCartesianBuildJointSampleProgress(uint16_t sample_count)
{
    const float joint_speed_deg_s[3] = {
        ARM_LINEAR_Q1_MAX_SPEED_DEG_S,
        ARM_LINEAR_Q2_MAX_SPEED_DEG_S,
        ARM_LINEAR_Q3_MAX_SPEED_DEG_S,
    };
    float total_minimum_time_s = 0.0f;

    if (sample_count == 0u || sample_count > ARM_LINEAR_MAX_SAMPLES) {
        return 0.0f;
    }
    arm_cartesian_runtime.sample_progress[0] = 0.0f;
    for (uint16_t i = 1u; i < sample_count; ++i) {
        float segment_minimum_time_s = 0.0f;
        float segment_delta_deg[3];

        segment_delta_deg[0] = fabsf(ArmCartesianWrapTo180(
            arm_cartesian_runtime.sample_q_deg[i][0] -
            arm_cartesian_runtime.sample_q_deg[i - 1u][0]));
        segment_delta_deg[1] = fabsf(
            arm_cartesian_runtime.sample_q_deg[i][1] -
            arm_cartesian_runtime.sample_q_deg[i - 1u][1]);
        segment_delta_deg[2] = fabsf(
            arm_cartesian_runtime.sample_q_deg[i][2] -
            arm_cartesian_runtime.sample_q_deg[i - 1u][2]);
        for (uint8_t joint = 0u; joint < 3u; ++joint) {
            segment_minimum_time_s = fmaxf(segment_minimum_time_s,
                segment_delta_deg[joint] / joint_speed_deg_s[joint]);
        }

        /* 保证进度严格递增，允许缓存中出现重合的安全样本。 */
        if (segment_minimum_time_s < ARM_SAMPLE_PROGRESS_EPSILON) {
            segment_minimum_time_s = ARM_SAMPLE_PROGRESS_EPSILON;
        }
        total_minimum_time_s += segment_minimum_time_s;
        arm_cartesian_runtime.sample_progress[i] = total_minimum_time_s;
    }

    if (sample_count == 1u ||
        total_minimum_time_s < ARM_SAMPLE_PROGRESS_EPSILON) {
        arm_cartesian_runtime.sample_progress[0] = 0.0f;
        return 0.0f;
    }
    for (uint16_t i = 1u; i < sample_count; ++i) {
        arm_cartesian_runtime.sample_progress[i] /= total_minimum_time_s;
    }
    arm_cartesian_runtime.sample_progress[sample_count - 1u] = 1.0f;
    return total_minimum_time_s;
}

static uint8_t ArmCartesianBlendCompositeWaypoint(
    uint16_t waypoint_index,
    uint16_t sample_count)
{
    uint16_t before_count = ARM_COMPOSITE_BLEND_JOINT_INTERVALS;
    uint16_t after_count = ARM_COMPOSITE_BLEND_LINEAR_INTERVALS;
    uint16_t first_index;
    uint16_t last_index;
    uint16_t blend_interval_count;
    float control_q_deg[4][3];

    if (waypoint_index == 0u || waypoint_index + 1u >= sample_count) {
        return 1u;
    }
    if (before_count > waypoint_index) {
        before_count = waypoint_index;
    }
    if (after_count > sample_count - 1u - waypoint_index) {
        after_count = (uint16_t)(sample_count - 1u - waypoint_index);
    }
    if (before_count >= waypoint_index) {
        before_count = (uint16_t)(waypoint_index - 1u);
    }
    if (after_count >= sample_count - 1u - waypoint_index) {
        after_count = (uint16_t)(sample_count - 2u - waypoint_index);
    }
    if (before_count == 0u || after_count == 0u) {
        return 1u;
    }

    first_index = (uint16_t)(waypoint_index - before_count);
    last_index = (uint16_t)(waypoint_index + after_count);
    blend_interval_count = (uint16_t)(last_index - first_index);
    memcpy(control_q_deg[0], arm_cartesian_runtime.sample_q_deg[first_index],
           sizeof(control_q_deg[0]));
    memcpy(control_q_deg[3], arm_cartesian_runtime.sample_q_deg[last_index],
           sizeof(control_q_deg[3]));
    for (uint8_t joint = 0u; joint < 3u; ++joint) {
        float incoming_step_deg = joint == ARM_JOINT_BASE_YAW ?
            ArmCartesianWrapTo180(
                arm_cartesian_runtime.sample_q_deg[first_index][joint] -
                arm_cartesian_runtime.sample_q_deg[first_index - 1u][joint]) :
            arm_cartesian_runtime.sample_q_deg[first_index][joint] -
                arm_cartesian_runtime.sample_q_deg[first_index - 1u][joint];
        float outgoing_step_deg = joint == ARM_JOINT_BASE_YAW ?
            ArmCartesianWrapTo180(
                arm_cartesian_runtime.sample_q_deg[last_index + 1u][joint] -
                arm_cartesian_runtime.sample_q_deg[last_index][joint]) :
            arm_cartesian_runtime.sample_q_deg[last_index + 1u][joint] -
                arm_cartesian_runtime.sample_q_deg[last_index][joint];

        control_q_deg[1][joint] = control_q_deg[0][joint] +
            incoming_step_deg * (float)blend_interval_count / 3.0f;
        control_q_deg[2][joint] = control_q_deg[3][joint] -
            outgoing_step_deg * (float)blend_interval_count / 3.0f;
    }

    for (uint16_t i = first_index; i <= last_index; ++i) {
        float ratio = (float)(i - first_index) /
            (float)blend_interval_count;
        float one_minus_ratio = 1.0f - ratio;
        float one_minus_ratio_2 = one_minus_ratio * one_minus_ratio;
        float ratio_2 = ratio * ratio;
        float blended_q_deg[3];

        for (uint8_t joint = 0u; joint < 3u; ++joint) {
            blended_q_deg[joint] =
                one_minus_ratio_2 * one_minus_ratio *
                    control_q_deg[0][joint] +
                3.0f * one_minus_ratio_2 * ratio *
                    control_q_deg[1][joint] +
                3.0f * one_minus_ratio * ratio_2 *
                    control_q_deg[2][joint] +
                ratio_2 * ratio * control_q_deg[3][joint];
        }
        blended_q_deg[ARM_JOINT_BASE_YAW] = ArmCartesianWrapTo180(
            blended_q_deg[ARM_JOINT_BASE_YAW]);
        if (!ArmJointPoseWithinSoftLimits(blended_q_deg) ||
            !ArmCartesianAutoPoseIsSafe(blended_q_deg) ||
            !ArmCartesianToolPitchAllowedForQ(blended_q_deg)) {
            return 0u;
        }
        memcpy(arm_cartesian_runtime.sample_q_deg[i], blended_q_deg,
               sizeof(blended_q_deg));
    }

    for (uint16_t i = first_index + 1u; i <= last_index + 1u; ++i) {
        if (!ArmCartesianJointStepContinuous(
                arm_cartesian_runtime.sample_q_deg[i - 1u],
                arm_cartesian_runtime.sample_q_deg[i])) {
            return 0u;
        }
    }
    return 1u;
}

static uint32_t ArmCartesianDurationMs(float path_length_mm,
                                       float max_speed_mm_s,
                                       float max_accel_mm_s2,
                                       uint16_t sample_count)
{
    /*
     * 五次曲线的峰值速度系数为1.875，峰值加速度系数为5.7735。
     * 先满足笛卡尔速度/加速度，再按整段及局部最陡IK变化延长时间。
     */
    float duration_s = 0.0f;
    float joint_travel_deg[3] = {0.0f, 0.0f, 0.0f};
    float joint_path_minimum_time_s =
        ArmCartesianBuildJointSampleProgress(sample_count);
    const float joint_speed_deg_s[3] = {
        ARM_LINEAR_Q1_MAX_SPEED_DEG_S,
        ARM_LINEAR_Q2_MAX_SPEED_DEG_S,
        ARM_LINEAR_Q3_MAX_SPEED_DEG_S,
    };
    const float joint_accel_deg_s2[3] = {
        ARM_LINEAR_Q1_MAX_ACCEL_DEG_S2,
        ARM_LINEAR_Q2_MAX_ACCEL_DEG_S2,
        ARM_LINEAR_Q3_MAX_ACCEL_DEG_S2,
    };

    if (path_length_mm > ARM_LINEAR_MIN_DISTANCE_MM) {
        float speed_duration_s = ARM_LINEAR_QUINTIC_PEAK_FACTOR *
            path_length_mm / max_speed_mm_s;
        float accel_duration_s = sqrtf(ARM_LINEAR_QUINTIC_ACCEL_FACTOR *
            path_length_mm / max_accel_mm_s2);

        duration_s = fmaxf(speed_duration_s, accel_duration_s);
    }
    duration_s = fmaxf(duration_s,
        ARM_LINEAR_QUINTIC_PEAK_FACTOR * joint_path_minimum_time_s);

    for (uint16_t i = 1u; i < sample_count; ++i) {
        joint_travel_deg[0] += fabsf(ArmCartesianWrapTo180(
            arm_cartesian_runtime.sample_q_deg[i][0] -
            arm_cartesian_runtime.sample_q_deg[i - 1u][0]));
        joint_travel_deg[1] += fabsf(
            arm_cartesian_runtime.sample_q_deg[i][1] -
            arm_cartesian_runtime.sample_q_deg[i - 1u][1]);
        joint_travel_deg[2] += fabsf(
            arm_cartesian_runtime.sample_q_deg[i][2] -
            arm_cartesian_runtime.sample_q_deg[i - 1u][2]);
    }
    for (uint8_t joint = 0u; joint < 3u; ++joint) {
        float speed_duration_s = ARM_LINEAR_QUINTIC_PEAK_FACTOR *
            joint_travel_deg[joint] / joint_speed_deg_s[joint];
        float accel_duration_s = sqrtf(ARM_LINEAR_QUINTIC_ACCEL_FACTOR *
            joint_travel_deg[joint] / joint_accel_deg_s2[joint]);

        duration_s = fmaxf(duration_s,
                           fmaxf(speed_duration_s, accel_duration_s));
    }
    for (uint16_t i = 1u; i < sample_count; ++i) {
        float segment_delta_deg[3];
        float segment_progress =
            arm_cartesian_runtime.sample_progress[i] -
            arm_cartesian_runtime.sample_progress[i - 1u];

        if (segment_progress < ARM_SAMPLE_PROGRESS_EPSILON) {
            continue;
        }

        segment_delta_deg[0] = fabsf(ArmCartesianWrapTo180(
            arm_cartesian_runtime.sample_q_deg[i][0] -
            arm_cartesian_runtime.sample_q_deg[i - 1u][0]));
        segment_delta_deg[1] = fabsf(
            arm_cartesian_runtime.sample_q_deg[i][1] -
            arm_cartesian_runtime.sample_q_deg[i - 1u][1]);
        segment_delta_deg[2] = fabsf(
            arm_cartesian_runtime.sample_q_deg[i][2] -
            arm_cartesian_runtime.sample_q_deg[i - 1u][2]);
        for (uint8_t joint = 0u; joint < 3u; ++joint) {
            float segment_speed_duration_s = ARM_LINEAR_QUINTIC_PEAK_FACTOR *
                segment_delta_deg[joint] /
                (segment_progress * joint_speed_deg_s[joint]);
            float segment_accel_duration_s = sqrtf(
                ARM_LINEAR_QUINTIC_ACCEL_FACTOR *
                segment_delta_deg[joint] *
                1.0f /
                (segment_progress * joint_accel_deg_s2[joint]));

            duration_s = fmaxf(duration_s,
                fmaxf(segment_speed_duration_s,
                      segment_accel_duration_s));
        }
    }
    if (duration_s < 0.001f) {
        duration_s = 0.001f;
    }
    return (uint32_t)(duration_s * 1000.0f + 0.999f);
}

static void ArmCartesianInterpolateJointSamples(float progress,
                                                float reference_q_deg[3])
{
    float clamped_progress;
    float progress_span;
    float sample_fraction;
    uint16_t lower_index;
    uint16_t upper_index;

    if (arm_cartesian_runtime.sample_count <= 1u) {
        memcpy(reference_q_deg, arm_cartesian_runtime.sample_q_deg[0],
               sizeof(float) * 3u);
        return;
    }
    clamped_progress = ArmCartesianClamp(progress, 0.0f, 1.0f);
    if (clamped_progress >= 1.0f) {
        lower_index = arm_cartesian_runtime.sample_count - 1u;
        upper_index = lower_index;
        sample_fraction = 0.0f;
    } else {
        uint16_t search_low = 0u;
        uint16_t search_high = arm_cartesian_runtime.sample_count - 1u;

        while ((uint16_t)(search_low + 1u) < search_high) {
            uint16_t middle = (uint16_t)(search_low +
                (search_high - search_low) / 2u);

            if (arm_cartesian_runtime.sample_progress[middle] <=
                clamped_progress) {
                search_low = middle;
            } else {
                search_high = middle;
            }
        }
        lower_index = search_low;
        upper_index = search_high;
        progress_span =
            arm_cartesian_runtime.sample_progress[upper_index] -
            arm_cartesian_runtime.sample_progress[lower_index];
        sample_fraction = progress_span > ARM_SAMPLE_PROGRESS_EPSILON ?
            (clamped_progress -
             arm_cartesian_runtime.sample_progress[lower_index]) /
                progress_span : 0.0f;
    }
    reference_q_deg[0] = ArmCartesianWrapTo180(
        arm_cartesian_runtime.sample_q_deg[lower_index][0] +
        sample_fraction * ArmCartesianWrapTo180(
            arm_cartesian_runtime.sample_q_deg[upper_index][0] -
            arm_cartesian_runtime.sample_q_deg[lower_index][0]));
    reference_q_deg[1] =
        arm_cartesian_runtime.sample_q_deg[lower_index][1] +
        sample_fraction *
            (arm_cartesian_runtime.sample_q_deg[upper_index][1] -
             arm_cartesian_runtime.sample_q_deg[lower_index][1]);
    reference_q_deg[2] =
        arm_cartesian_runtime.sample_q_deg[lower_index][2] +
        sample_fraction *
            (arm_cartesian_runtime.sample_q_deg[upper_index][2] -
             arm_cartesian_runtime.sample_q_deg[lower_index][2]);
}

static void ArmCartesianInterpolateRouteSegment(float progress,
                                                float reference_q_deg[3])
{
    uint8_t segment = arm_cartesian_runtime.route_active_segment;
    uint16_t first = arm_cartesian_runtime.route_segment_start[segment];
    uint16_t last = arm_cartesian_runtime.route_segment_end[segment];
    float scaled = ArmCartesianClamp(progress, 0.0f, 1.0f) *
        (float)(last - first);
    uint16_t lower = (uint16_t)scaled;
    uint16_t upper;
    float fraction;

    if (lower >= (uint16_t)(last - first)) {
        lower = (uint16_t)(last - first);
        upper = lower;
        fraction = 0.0f;
    } else {
        upper = (uint16_t)(lower + 1u);
        fraction = scaled - (float)lower;
    }
    lower = (uint16_t)(first + lower);
    upper = (uint16_t)(first + upper);
    reference_q_deg[0] = ArmCartesianWrapTo180(
        arm_cartesian_runtime.sample_q_deg[lower][0] + fraction *
        ArmCartesianWrapTo180(
            arm_cartesian_runtime.sample_q_deg[upper][0] -
            arm_cartesian_runtime.sample_q_deg[lower][0]));
    reference_q_deg[1] = arm_cartesian_runtime.sample_q_deg[lower][1] +
        fraction * (arm_cartesian_runtime.sample_q_deg[upper][1] -
                    arm_cartesian_runtime.sample_q_deg[lower][1]);
    reference_q_deg[2] = arm_cartesian_runtime.sample_q_deg[lower][2] +
        fraction * (arm_cartesian_runtime.sample_q_deg[upper][2] -
                    arm_cartesian_runtime.sample_q_deg[lower][2]);
}

static void ArmCartesianPositionAtProgress(float progress,
                                           Arm_Position_s *position)
{
    position->x_mm = arm_cartesian_runtime.start_position.x_mm +
        progress * (arm_cartesian_runtime.target_position.x_mm -
                    arm_cartesian_runtime.start_position.x_mm);
    position->y_mm = arm_cartesian_runtime.start_position.y_mm +
        progress * (arm_cartesian_runtime.target_position.y_mm -
                    arm_cartesian_runtime.start_position.y_mm);
    position->z_mm = arm_cartesian_runtime.start_position.z_mm +
        progress * (arm_cartesian_runtime.target_position.z_mm -
                    arm_cartesian_runtime.start_position.z_mm);
}

static uint8_t ArmCartesianSolveOnlineIK(uint32_t now_ms,
                                        float progress)
{
    Arm_Position_s sample_position;
    Arm_IK_Result_s result;
    float seed_q_deg[3];

    ArmCartesianPositionAtProgress(progress, &sample_position);
    if (arm_cartesian_runtime.online_ik_valid) {
        memcpy(seed_q_deg, arm_cartesian_runtime.online_ik_next_q_deg,
               sizeof(seed_q_deg));
        memcpy(arm_cartesian_runtime.online_ik_previous_q_deg,
               arm_cartesian_runtime.online_ik_next_q_deg,
               sizeof(arm_cartesian_runtime.online_ik_previous_q_deg));
    } else {
        memcpy(seed_q_deg, g_arm_motion_debug.trajectory_q_deg,
               sizeof(seed_q_deg));
        memcpy(arm_cartesian_runtime.online_ik_previous_q_deg, seed_q_deg,
               sizeof(seed_q_deg));
    }
    memset(&result, 0, sizeof(result));
    if (ArmCartesianSolveControlPoint(
            arm_cartesian_runtime.active_control_point, &sample_position,
            seed_q_deg, &result, NULL) !=
            ARM_IK_OK ||
        result.position_error_mm > ARM_LINEAR_FK_ERROR_MAX_MM ||
        !ArmJointPoseWithinSoftLimits(result.q_deg) ||
        !ArmCartesianAutoPoseIsSafe(result.q_deg) ||
        !ArmCartesianToolPitchAllowedForQ(result.q_deg) ||
        !ArmWorkspacePoseSafe(result.q_deg, 0u, NULL) ||
        !ArmCartesianJointStepContinuous(seed_q_deg, result.q_deg)) {
        g_arm_motion_debug.ik_status = result.status;
        g_arm_motion_debug.command_accepted = 0u;
        if (!arm_cartesian_runtime.reference_update_rejected) {
            g_arm_motion_debug.command_reject_count++;
            arm_cartesian_runtime.reference_update_rejected = 1u;
        }
        return 0u;
    }

    memcpy(arm_cartesian_runtime.online_ik_next_q_deg, result.q_deg,
           sizeof(result.q_deg));
    arm_cartesian_runtime.online_ik_interval_start_tick = now_ms;
    arm_cartesian_runtime.online_ik_interval_ms = ARM_LINEAR_IK_UPDATE_MS;
    arm_cartesian_runtime.last_ik_tick = now_ms;
    arm_cartesian_runtime.online_ik_valid = 1u;
    arm_cartesian_runtime.reference_update_rejected = 0u;
    g_arm_motion_debug.ik_status = ARM_IK_OK;
    g_arm_motion_debug.command_accepted = 1u;
    return 1u;
}

static void ArmCartesianInterpolateOnlineIK(uint32_t now_ms,
                                            float reference_q_deg[3])
{
    float ratio = arm_cartesian_runtime.online_ik_interval_ms > 0u ?
        (float)(now_ms -
                arm_cartesian_runtime.online_ik_interval_start_tick) /
            (float)arm_cartesian_runtime.online_ik_interval_ms : 1.0f;

    ratio = ArmCartesianClamp(ratio, 0.0f, 1.0f);
    reference_q_deg[0] = ArmCartesianWrapTo180(
        arm_cartesian_runtime.online_ik_previous_q_deg[0] +
        ratio * ArmCartesianWrapTo180(
            arm_cartesian_runtime.online_ik_next_q_deg[0] -
            arm_cartesian_runtime.online_ik_previous_q_deg[0]));
    reference_q_deg[1] =
        arm_cartesian_runtime.online_ik_previous_q_deg[1] +
        ratio * (arm_cartesian_runtime.online_ik_next_q_deg[1] -
                 arm_cartesian_runtime.online_ik_previous_q_deg[1]);
    reference_q_deg[2] =
        arm_cartesian_runtime.online_ik_previous_q_deg[2] +
        ratio * (arm_cartesian_runtime.online_ik_next_q_deg[2] -
                 arm_cartesian_runtime.online_ik_previous_q_deg[2]);
}

static void ArmCartesianStartPreparedTrajectory(
    const Arm_Position_s *target,
    uint32_t duration_ms,
    float commanded_speed_mm_s,
    float commanded_accel_mm_s2,
    Arm_Trajectory_Path_e path_type,
    Arm_Motion_State_e motion_state,
    uint32_t now_ms)
{
    uint16_t last_index = arm_cartesian_runtime.sample_count - 1u;

    arm_cartesian_runtime.target_position = *target;
    if (arm_cartesian_runtime.active_control_point ==
        ARM_CONTROL_POINT_TOOL_CENTER) {
        arm_cartesian_runtime.active_target_tool_tip = *target;
    } else {
        arm_cartesian_runtime.active_target_wrist = *target;
        arm_cartesian_runtime.active_target_tool_tip = *target;
    }
    arm_cartesian_runtime.trajectory_duration_ms = duration_ms;
    arm_cartesian_runtime.trajectory_start_tick = now_ms;
    arm_cartesian_runtime.commanded_speed_mm_s = commanded_speed_mm_s;
    arm_cartesian_runtime.commanded_accel_mm_s2 = commanded_accel_mm_s2;
    arm_cartesian_runtime.path_type = path_type;
    arm_cartesian_runtime.reference_update_rejected = 0u;
    arm_cartesian_runtime.online_ik_valid = 0u;
    arm_cartesian_runtime.last_ik_tick = now_ms;
    arm_cartesian_runtime.online_ik_interval_start_tick = now_ms;
    arm_cartesian_runtime.online_ik_interval_ms = ARM_LINEAR_IK_UPDATE_MS;
    arm_cartesian_runtime.realtime_active = 0u;
    arm_cartesian_runtime.realtime_timed_out = 0u;
    arm_cartesian_runtime.path_length_mm = ArmCartesianPositionDistance(
        &arm_cartesian_runtime.start_position, target);
    memcpy(g_arm_motion_debug.trajectory_q_deg,
           arm_cartesian_runtime.sample_q_deg[0],
           sizeof(g_arm_motion_debug.trajectory_q_deg));
    memcpy(g_arm_motion_debug.target_q_deg,
           arm_cartesian_runtime.sample_q_deg[last_index],
           sizeof(g_arm_motion_debug.target_q_deg));
    g_arm_motion_debug.target_position_mm = *target;
    g_arm_motion_debug.ik_status = ARM_IK_OK;
    g_arm_motion_debug.path_preflight_passed = 1u;
    g_arm_motion_debug.command_accepted = 1u;
    g_arm_motion_debug.trajectory_progress = 0.0f;
    g_arm_motion_debug.trajectory_elapsed_ms = 0u;
    g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_NONE;
    ArmCartesianResetControlStatistics();
    ArmCartesianSetState(motion_state, now_ms);
}

static void ArmCartesianStartRouteSegment(uint8_t segment, uint32_t now_ms)
{
    uint16_t first = arm_cartesian_runtime.route_segment_start[segment];
    uint16_t last = arm_cartesian_runtime.route_segment_end[segment];

    arm_cartesian_runtime.route_active_segment = segment;
    arm_cartesian_runtime.trajectory_start_tick = now_ms;
    arm_cartesian_runtime.trajectory_duration_ms =
        arm_cartesian_runtime.route_segment_duration_ms[segment];
    arm_cartesian_runtime.start_position =
        arm_cartesian_runtime.route_segment_start_position[segment];
    arm_cartesian_runtime.target_position =
        arm_cartesian_runtime.route_segment_target_position[segment];
    arm_cartesian_runtime.arrival_stable_tick = 0u;
    memcpy(g_arm_motion_debug.trajectory_q_deg,
           arm_cartesian_runtime.sample_q_deg[first],
           sizeof(g_arm_motion_debug.trajectory_q_deg));
    memcpy(g_arm_motion_debug.target_q_deg,
           arm_cartesian_runtime.sample_q_deg[last],
           sizeof(g_arm_motion_debug.target_q_deg));
    g_arm_motion_debug.target_position_mm =
        arm_cartesian_runtime.route_segment_target_position[segment];
    g_arm_motion_debug.trajectory_progress = 0.0f;
    g_arm_motion_debug.trajectory_elapsed_ms = 0u;
    g_arm_motion_debug.safety_route_active_segment = segment;
    ArmCartesianSetState(ARM_MOTION_RUNNING, now_ms);
}

void ArmAbortMotion(Arm_Motion_Fault_e reason)
{
    g_arm_motion_debug.fault_code = reason;
    g_arm_motion_debug.motion_state = ARM_MOTION_ABORTED;
    g_arm_motion_debug.command_accepted = 0u;
    arm_cartesian_runtime.realtime_active = 0u;
    arm_cartesian_runtime.realtime_timed_out = 0u;
    arm_cartesian_runtime.online_ik_valid = 0u;
    arm_cartesian_runtime.sample_count = 0u;
    memset(arm_cartesian_runtime.realtime_velocity_mm_s, 0,
           sizeof(arm_cartesian_runtime.realtime_velocity_mm_s));
}

uint8_t ArmTrajectoryMotorHoldAllowed(void)
{
    return g_arm_motion_debug.motion_state != ARM_MOTION_ABORTED;
}

uint8_t ArmTrajectoryOwnsControl(void)
{
    return arm_cartesian_runtime.realtime_active ||
           g_arm_motion_debug.motion_state == ARM_MOTION_STAGING ||
           g_arm_motion_debug.motion_state == ARM_MOTION_RUNNING ||
           g_arm_motion_debug.motion_state == ARM_MOTION_SETTLING ||
           g_arm_motion_debug.motion_state == ARM_MOTION_HOLDING;
}

uint8_t ArmTrajectoryIsBusy(void)
{
    return arm_cartesian_runtime.realtime_active ||
           g_arm_motion_debug.motion_state == ARM_MOTION_STAGING ||
           g_arm_motion_debug.motion_state == ARM_MOTION_PREFLIGHT ||
           g_arm_motion_debug.motion_state == ARM_MOTION_RUNNING ||
           g_arm_motion_debug.motion_state == ARM_MOTION_SETTLING;
}

uint8_t ArmTrajectoryRealtimeActive(void)
{
    return arm_cartesian_runtime.realtime_active;
}

void ArmTrajectoryCancel(void)
{
    float hold_q_deg[3];

    if (!ArmCartesianMotorsReady()) {
        return;
    }
    hold_q_deg[0] = g_arm_state.q_feedback_deg[ARM_JOINT_BASE_YAW];
    hold_q_deg[1] = g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER];
    hold_q_deg[2] = g_arm_state.q_feedback_deg[ARM_JOINT_ELBOW];
    /* 取消动作允许在安全脱限走廊内保持当前姿态，因此这里只检查底层硬限位。 */
    ArmUpdateJointReference(hold_q_deg);
    memcpy(g_arm_motion_debug.trajectory_q_deg, hold_q_deg,
           sizeof(hold_q_deg));
    memcpy(g_arm_motion_debug.target_q_deg, hold_q_deg,
           sizeof(hold_q_deg));
    g_arm_motion_debug.target_position_mm = g_arm_state.wrist_center;
    arm_cartesian_runtime.active_control_point =
        ARM_CONTROL_POINT_WRIST_CENTER;
    arm_cartesian_runtime.active_target_wrist = g_arm_state.wrist_center;
    arm_cartesian_runtime.active_target_tool_tip = g_arm_state.wrist_center;
    g_arm_motion_debug.trajectory_progress = 1.0f;
    g_arm_motion_debug.command_accepted = 1u;
    g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_NONE;
    arm_cartesian_runtime.realtime_active = 0u;
    arm_cartesian_runtime.realtime_timed_out = 0u;
    memset(arm_cartesian_runtime.realtime_velocity_mm_s, 0,
           sizeof(arm_cartesian_runtime.realtime_velocity_mm_s));
    arm_cartesian_runtime.sample_count = 1u;
    memcpy(arm_cartesian_runtime.sample_q_deg[0], hold_q_deg,
           sizeof(hold_q_deg));
    ArmCartesianSetState(ARM_MOTION_HOLDING, HAL_GetTick());
}

uint8_t ArmTrajectoryPreflightToolCenterSegment(
    const Arm_Path_Plan_Request_s *request,
    Arm_Path_Plan_Result_s *result)
{
    Arm_Path_Plan_Request_s serviced_request;
    const Arm_Path_Plan_Request_s *planner_request = request;
    Arm_Path_Plan_Workspace_s workspace;

    if (result == NULL) {
        return 0u;
    }
    if (ArmTrajectoryIsBusy()) {
        memset(result, 0, sizeof(*result));
        result->status = ARM_PATH_PLAN_INVALID;
        result->ik_status = ARM_IK_INVALID_ARGUMENT;
        return 0u;
    }
    workspace.sample_q_deg = arm_cartesian_runtime.sample_q_deg;
    workspace.sample_progress = arm_cartesian_runtime.sample_progress;
    workspace.candidate_predecessor =
        arm_cartesian_runtime.tool_candidate_predecessor;
    workspace.candidate_count = arm_cartesian_runtime.tool_candidate_count;
    workspace.selected_candidate =
        arm_cartesian_runtime.tool_selected_candidate;
    workspace.capacity = ARM_LINEAR_MAX_SAMPLES;
    if (request != NULL && request->service_hook == NULL) {
        serviced_request = *request;
        serviced_request.service_hook = ArmCartesianPreflightServiceTool;
        planner_request = &serviced_request;
    }
    return ArmPathPlanToolCenterSegment(
        planner_request, &workspace, result);
}

uint8_t ArmTrajectorySelectReachableToolCenterAdvance(
    const Arm_Path_Advance_Request_s *request,
    Arm_Path_Advance_Result_s *result)
{
    Arm_Path_Advance_Request_s serviced_request;
    const Arm_Path_Advance_Request_s *planner_request = request;
    Arm_Path_Plan_Workspace_s workspace;

    if (result == NULL) {
        return 0u;
    }
    if (ArmTrajectoryIsBusy()) {
        memset(result, 0, sizeof(*result));
        result->plan_result.status = ARM_PATH_PLAN_INVALID;
        result->plan_result.ik_status = ARM_IK_INVALID_ARGUMENT;
        return 0u;
    }
    workspace.sample_q_deg = arm_cartesian_runtime.sample_q_deg;
    workspace.sample_progress = arm_cartesian_runtime.sample_progress;
    workspace.candidate_predecessor =
        arm_cartesian_runtime.tool_candidate_predecessor;
    workspace.candidate_count = arm_cartesian_runtime.tool_candidate_count;
    workspace.selected_candidate =
        arm_cartesian_runtime.tool_selected_candidate;
    workspace.capacity = ARM_LINEAR_MAX_SAMPLES;
    if (request != NULL && request->service_hook == NULL) {
        serviced_request = *request;
        serviced_request.service_hook = ArmCartesianPreflightServiceTool;
        planner_request = &serviced_request;
    }
    return ArmPathSelectReachableAdvance(
        planner_request, &workspace, result);
}

static float ArmCartesianVectorLength(const float vector[3])
{
    return sqrtf(vector[0] * vector[0] + vector[1] * vector[1] +
                 vector[2] * vector[2]);
}

static void ArmCartesianLimitVector(float vector[3], float max_length)
{
    float length = ArmCartesianVectorLength(vector);

    if (length > max_length && length > 0.000001f) {
        float scale = max_length / length;

        vector[0] *= scale;
        vector[1] *= scale;
        vector[2] *= scale;
    }
}

static void ArmCartesianSetVectorLength(float vector[3], float length)
{
    float current_length = ArmCartesianVectorLength(vector);

    if (current_length <= 0.000001f || length <= 0.0f) {
        vector[0] = 0.0f;
        vector[1] = 0.0f;
        vector[2] = 0.0f;
        return;
    }
    vector[0] *= length / current_length;
    vector[1] *= length / current_length;
    vector[2] *= length / current_length;
}

Arm_Command_Result_e ArmTrajectorySubmitRealtimeTarget(
    const Arm_Realtime_Cartesian_Target_s *target)
{
    Arm_IK_Result_s result;
    float seed_q_deg[3];
    Arm_Position_s ik_target;
    Arm_Position_s current_center;
    Arm_Position_s target_center;
    float speed_mm_s;
    float accel_mm_s2;
    uint32_t now_ms = HAL_GetTick();

    if (target == NULL || !isfinite(target->target_mm.x_mm) ||
        !isfinite(target->target_mm.y_mm) ||
        !isfinite(target->target_mm.z_mm) ||
        !isfinite(target->max_speed_mm_s) ||
        !isfinite(target->max_acceleration_mm_s2) ||
        target->max_speed_mm_s < 0.0f ||
        target->max_acceleration_mm_s2 < 0.0f) {
        return ARM_COMMAND_INVALID;
    }
    if (target->tool_yaw_valid != 0u) {
        return ARM_COMMAND_UNSUPPORTED;
    }
    /* 实时控制不继承上一条AC抓取命令的局部放宽策略。 */
    arm_cartesian_runtime.safety_profile = ARM_CARTESIAN_SAFETY_NORMAL;
    if (!ArmCartesianMotorsReady()) {
        return ARM_COMMAND_NOT_READY;
    }
    if ((target->tool_pitch_valid != 0u &&
         !ArmCartesianResolveToolPitch(1u, target->tool_pitch_deg)) ||
        (target->tool_pitch_valid == 0u &&
         arm_cartesian_runtime.realtime_active == 0u &&
         !ArmCartesianResolveToolPitch(0u, 0.0f))) {
        return ARM_COMMAND_PREFLIGHT_FAILED;
    }
    if (target->control_point != ARM_CONTROL_POINT_WRIST_CENTER &&
        target->control_point != ARM_CONTROL_POINT_TOOL_CENTER) {
        return ARM_COMMAND_INVALID;
    }

    speed_mm_s = target->max_speed_mm_s > 0.0f ?
        target->max_speed_mm_s : ARM_LINEAR_DEFAULT_SPEED_MM_S;
    accel_mm_s2 = target->max_acceleration_mm_s2 > 0.0f ?
        target->max_acceleration_mm_s2 :
        ARM_REALTIME_DEFAULT_ACCEL_MM_S2;
    if (speed_mm_s < ARM_LINEAR_MIN_SPEED_MM_S ||
        speed_mm_s > ARM_LINEAR_MAX_SPEED_MM_S ||
        accel_mm_s2 < ARM_LINEAR_MIN_ACCEL_MM_S2 ||
        accel_mm_s2 > ARM_LINEAR_MAX_ACCEL_MM_S2) {
        return ARM_COMMAND_INVALID;
    }
    if (!ArmJointPoseWithinSoftLimits(g_arm_state.q_feedback_deg) ||
        !ArmCartesianAutoPoseIsSafe(g_arm_state.q_feedback_deg) ||
        !ArmCartesianToolPitchAllowedForQ(g_arm_state.q_feedback_deg)) {
        return ARM_COMMAND_NOT_READY;
    }
    if (arm_cartesian_runtime.realtime_active == 0u) {
        ArmCartesianResetSafetyRoute();
    }

    if (arm_cartesian_runtime.realtime_active) {
        memcpy(seed_q_deg, arm_cartesian_runtime.realtime_command_q_deg,
               sizeof(seed_q_deg));
    } else {
        memcpy(seed_q_deg, g_arm_state.q_feedback_deg, sizeof(seed_q_deg));
    }
    memset(&result, 0, sizeof(result));
    g_arm_motion_debug.realtime_reject_reason =
        ARM_REALTIME_REJECT_NONE;
    if (ArmCartesianSolveControlPoint(
            target->control_point, &target->target_mm, seed_q_deg,
            &result, &ik_target) !=
        ARM_IK_OK) {
        g_arm_motion_debug.realtime_reject_reason =
            ARM_REALTIME_REJECT_IK;
    } else if (result.position_error_mm > ARM_LINEAR_FK_ERROR_MAX_MM) {
        g_arm_motion_debug.realtime_reject_reason =
            ARM_REALTIME_REJECT_FK_ERROR;
    } else if (!ArmJointPoseWithinSoftLimits(result.q_deg)) {
        g_arm_motion_debug.realtime_reject_reason =
            ARM_REALTIME_REJECT_SOFT_LIMIT;
    } else if (!ArmCartesianAutoPoseIsSafe(result.q_deg)) {
        g_arm_motion_debug.realtime_reject_reason =
            ARM_REALTIME_REJECT_AUTO_REGION;
    } else if (!ArmCartesianToolPitchAllowedForQ(result.q_deg)) {
        g_arm_motion_debug.realtime_reject_reason =
            ARM_REALTIME_REJECT_AUTO_REGION;
    } else if (!ArmWorkspacePoseSafe(result.q_deg, 1u, NULL)) {
        g_arm_motion_debug.realtime_reject_reason =
            ARM_REALTIME_REJECT_WORKSPACE_SAFETY;
    } else if (!ArmForwardKinematicsToolCenter(
                   seed_q_deg,
                   arm_cartesian_runtime.active_tool_pitch_deg,
                   &current_center) ||
               !ArmForwardKinematicsToolCenter(
                   result.q_deg,
                   arm_cartesian_runtime.active_tool_pitch_deg,
                   &target_center) ||
               (ArmWorkspaceCrossesBoundary(&current_center,
                                             &target_center) &&
                fminf(current_center.z_mm, target_center.z_mm) <
                    ARM_REAR_CROSSING_TOOL_Z_MM)) {
        g_arm_motion_debug.workspace_safety_result =
            ARM_WORKSPACE_SAFETY_CROSSING_TOO_LOW;
        g_arm_motion_debug.realtime_reject_reason =
            ARM_REALTIME_REJECT_WORKSPACE_SAFETY;
    }
    if (g_arm_motion_debug.realtime_reject_reason !=
        ARM_REALTIME_REJECT_NONE) {
        /* 非法目标不覆盖单槽缓存，也不触碰当前参考和使能状态。 */
        g_arm_motion_debug.last_realtime_reject_reason =
            g_arm_motion_debug.realtime_reject_reason;
        ArmCartesianRecordRejected(result.status);
        return ARM_COMMAND_PREFLIGHT_FAILED;
    }

    if (!arm_cartesian_runtime.realtime_active) {
        if (!ArmBeginJointMove(result.q_deg)) {
            return ARM_COMMAND_NOT_READY;
        }
        arm_cartesian_runtime.realtime_reference_position =
            g_arm_state.wrist_center;
        arm_cartesian_runtime.realtime_last_valid_position =
            g_arm_state.wrist_center;
        if (target->control_point == ARM_CONTROL_POINT_TOOL_CENTER) {
            Arm_Position_s start_tip;

            if (!ArmForwardKinematicsToolCenter(
                    seed_q_deg, arm_cartesian_runtime.active_tool_pitch_deg,
                    &start_tip)) {
                return ARM_COMMAND_PREFLIGHT_FAILED;
            }
            arm_cartesian_runtime.realtime_reference_tool_tip = start_tip;
            arm_cartesian_runtime.realtime_last_valid_tool_tip = start_tip;
            arm_cartesian_runtime.realtime_reference_position = start_tip;
            arm_cartesian_runtime.realtime_last_valid_position = start_tip;
            arm_cartesian_runtime.start_position = start_tip;
        } else {
            arm_cartesian_runtime.start_position = g_arm_state.wrist_center;
        }
        memset(arm_cartesian_runtime.realtime_velocity_mm_s, 0,
               sizeof(arm_cartesian_runtime.realtime_velocity_mm_s));
        memset(arm_cartesian_runtime.realtime_last_velocity_mm_s, 0,
               sizeof(arm_cartesian_runtime.realtime_last_velocity_mm_s));
        memcpy(arm_cartesian_runtime.online_ik_previous_q_deg, seed_q_deg,
               sizeof(seed_q_deg));
        memcpy(arm_cartesian_runtime.online_ik_next_q_deg, seed_q_deg,
               sizeof(seed_q_deg));
        memcpy(arm_cartesian_runtime.realtime_command_q_deg, result.q_deg,
               sizeof(result.q_deg));
        arm_cartesian_runtime.online_ik_valid = 1u;
        arm_cartesian_runtime.last_ik_tick = now_ms;
        arm_cartesian_runtime.online_ik_interval_start_tick = now_ms;
        arm_cartesian_runtime.online_ik_interval_ms =
            ARM_LINEAR_IK_UPDATE_MS;
        arm_cartesian_runtime.realtime_active = 1u;
        arm_cartesian_runtime.trajectory_start_tick = now_ms;
        ArmCartesianResetControlStatistics();
    }

    arm_cartesian_runtime.realtime_target = *target;
    arm_cartesian_runtime.realtime_control_point = target->control_point;
    arm_cartesian_runtime.active_control_point = target->control_point;
    memcpy(arm_cartesian_runtime.realtime_command_q_deg, result.q_deg,
           sizeof(result.q_deg));
    arm_cartesian_runtime.realtime_target.max_speed_mm_s = speed_mm_s;
    arm_cartesian_runtime.realtime_target.max_acceleration_mm_s2 =
        accel_mm_s2;
    arm_cartesian_runtime.realtime_last_command_tick = now_ms;
    arm_cartesian_runtime.realtime_timed_out = 0u;
    arm_cartesian_runtime.commanded_speed_mm_s = speed_mm_s;
    arm_cartesian_runtime.commanded_accel_mm_s2 = accel_mm_s2;
    arm_cartesian_runtime.target_position = target->target_mm;
    arm_cartesian_runtime.path_length_mm = ArmCartesianPositionDistance(
        &arm_cartesian_runtime.realtime_reference_position,
        &target->target_mm);
    arm_cartesian_runtime.active_target_tool_tip = target->target_mm;
    arm_cartesian_runtime.active_target_wrist = ik_target;
    memcpy(g_arm_motion_debug.target_q_deg, result.q_deg,
           sizeof(result.q_deg));
    g_arm_motion_debug.target_position_mm = target->target_mm;
    g_arm_motion_debug.ik_status = ARM_IK_OK;
    g_arm_motion_debug.command_accepted = 1u;
    g_arm_motion_debug.path_preflight_passed = 1u;
    g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_NONE;
    ArmCartesianSetState(ARM_MOTION_RUNNING, now_ms);
    return ARM_COMMAND_OK;
}

void ArmTrajectoryStopRealtime(void)
{
    if (!arm_cartesian_runtime.realtime_active) {
        return;
    }
    arm_cartesian_runtime.realtime_active = 0u;
    arm_cartesian_runtime.realtime_timed_out = 0u;
    memset(arm_cartesian_runtime.realtime_velocity_mm_s, 0,
           sizeof(arm_cartesian_runtime.realtime_velocity_mm_s));
    ArmTrajectoryCancel();
}

Arm_Motion_Result_e ArmTrajectoryMoveJoint(const float target_q_deg[3])
{
    return ArmTrajectoryMoveJointWithRelativeToolPitch(
        target_q_deg, 0u, 0.0f);
}

Arm_Motion_Result_e ArmTrajectoryMoveJointWithRelativeToolPitch(
    const float target_q_deg[3],
    uint8_t relative_pitch_valid,
    float relative_pitch_deg)
{
    return ArmTrajectoryMoveJointWithOptions(
        target_q_deg, 0u, NULL,
        relative_pitch_valid, relative_pitch_deg);
}

Arm_Motion_Result_e ArmTrajectoryMoveJointWithOptions(
    const float target_q_deg[3],
    uint8_t waypoint_valid,
    const float waypoint_q_deg[3],
    uint8_t relative_pitch_valid,
    float relative_pitch_deg)
{
    Arm_Position_s target_position;
    uint32_t duration_ms;

    if (target_q_deg == NULL ||
        (waypoint_valid != 0u && waypoint_q_deg == NULL) ||
        ArmTrajectoryIsBusy()) {
        return (target_q_deg == NULL ||
                (waypoint_valid != 0u && waypoint_q_deg == NULL)) ?
            ARM_MOTION_RESULT_INVALID : ARM_MOTION_RESULT_BUSY;
    }
    if (!ArmCartesianMotorsReady()) {
        return ARM_MOTION_RESULT_NOT_READY;
    }
    if (!ArmCartesianResolveCommandRelativeToolPitch(
            relative_pitch_valid, relative_pitch_deg) ||
        !ArmCartesianToolPitchAllowedForQ(g_arm_state.q_feedback_deg) ||
        (waypoint_valid != 0u &&
         !ArmCartesianToolPitchAllowedForQ(waypoint_q_deg)) ||
        !ArmCartesianToolPitchAllowedForQ(target_q_deg)) {
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }
    if (!ArmJointPoseWithinSoftLimits(target_q_deg) ||
        (waypoint_valid != 0u &&
         !ArmJointPoseWithinSoftLimits(waypoint_q_deg)) ||
        (waypoint_valid != 0u &&
         (!ArmWorkspaceJointPathSafe(g_arm_state.q_feedback_deg,
                                     waypoint_q_deg) ||
          !ArmWorkspaceJointPathSafe(waypoint_q_deg, target_q_deg))) ||
        (waypoint_valid == 0u &&
        !ArmWorkspaceJointPathSafe(g_arm_state.q_feedback_deg,
                                   target_q_deg))) {
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }
    ArmCartesianResetSafetyRoute();
    arm_cartesian_runtime.sample_count = waypoint_valid != 0u ? 3u : 2u;
    arm_cartesian_runtime.active_control_point =
        ARM_CONTROL_POINT_WRIST_CENTER;
    memcpy(arm_cartesian_runtime.sample_q_deg[0],
           g_arm_state.q_feedback_deg, sizeof(float) * 3u);
    if (waypoint_valid != 0u) {
        memcpy(arm_cartesian_runtime.sample_q_deg[1], waypoint_q_deg,
               sizeof(float) * 3u);
    }
    memcpy(arm_cartesian_runtime.sample_q_deg[
               arm_cartesian_runtime.sample_count - 1u], target_q_deg,
           sizeof(float) * 3u);
    ArmForwardKinematics3DOF(target_q_deg[0], target_q_deg[1],
                             target_q_deg[2], &target_position);
    arm_cartesian_runtime.start_position = g_arm_state.wrist_center;
    duration_ms = ArmCartesianDurationMs(0.0f,
        ARM_LINEAR_DEFAULT_SPEED_MM_S, ARM_LINEAR_MAX_ACCEL_MM_S2,
        arm_cartesian_runtime.sample_count);
    if (!ArmBeginJointMove(target_q_deg)) {
        return ARM_MOTION_RESULT_NOT_READY;
    }
    ArmCartesianStartPreparedTrajectory(
        &target_position, duration_ms, ARM_LINEAR_DEFAULT_SPEED_MM_S,
        ARM_LINEAR_MAX_ACCEL_MM_S2, ARM_TRAJECTORY_PATH_JOINT_STAGING,
        ARM_MOTION_RUNNING, HAL_GetTick());
    return ARM_MOTION_RESULT_OK;
}

Arm_Motion_Result_e ArmTrajectorySetJointDirect(const float target_q_deg[3])
{
    Arm_Position_s target_position;

    if (target_q_deg == NULL || ArmTrajectoryIsBusy()) {
        return target_q_deg == NULL ? ARM_MOTION_RESULT_INVALID :
                                     ARM_MOTION_RESULT_BUSY;
    }
    if (!ArmCartesianMotorsReady()) {
        return ARM_MOTION_RESULT_NOT_READY;
    }
    if (!ArmCartesianResolveRelativeToolPitch() ||
        !ArmCartesianToolPitchAllowedForQ(g_arm_state.q_feedback_deg) ||
        !ArmCartesianToolPitchAllowedForQ(target_q_deg)) {
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }
    if (!ArmJointPoseWithinSoftLimits(target_q_deg) ||
        !ArmWorkspaceJointPathSafe(g_arm_state.q_feedback_deg,
                                   target_q_deg)) {
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }
    ArmCartesianResetSafetyRoute();
    if (!ArmSetJointTargetDeg(target_q_deg[0], target_q_deg[1],
                              target_q_deg[2])) {
        return ARM_MOTION_RESULT_NOT_READY;
    }
    arm_cartesian_runtime.active_control_point =
        ARM_CONTROL_POINT_WRIST_CENTER;
    arm_cartesian_runtime.sample_count = 1u;
    memcpy(arm_cartesian_runtime.sample_q_deg[0], target_q_deg,
           sizeof(float) * 3u);
    ArmForwardKinematics3DOF(target_q_deg[0], target_q_deg[1],
                             target_q_deg[2], &target_position);
    memcpy(g_arm_motion_debug.trajectory_q_deg, target_q_deg,
           sizeof(float) * 3u);
    memcpy(g_arm_motion_debug.target_q_deg, target_q_deg,
           sizeof(float) * 3u);
    g_arm_motion_debug.target_position_mm = target_position;
    g_arm_motion_debug.command_accepted = 1u;
    g_arm_motion_debug.path_preflight_passed = 1u;
    g_arm_motion_debug.trajectory_progress = 1.0f;
    g_arm_motion_debug.trajectory_duration_ms = 0u;
    g_arm_motion_debug.trajectory_elapsed_ms = 0u;
    g_arm_motion_debug.path_sample_count = 1u;
    g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_NONE;
    arm_cartesian_runtime.arrival_stable_tick = 0u;
    ArmCartesianSetState(ARM_MOTION_SETTLING, HAL_GetTick());
    return ARM_MOTION_RESULT_OK;
}

Arm_Motion_Result_e ArmSetCartesianTarget(const Arm_Position_s *target,
                                          Arm_IK_Result_s *result)
{
    Arm_IK_Result_s local_result;
    float seed_q_deg[3];

    memset(&local_result, 0, sizeof(local_result));
    local_result.status = ARM_IK_INVALID_ARGUMENT;
    if (target == NULL || !isfinite(target->x_mm) ||
        !isfinite(target->y_mm) || !isfinite(target->z_mm)) {
        ArmCartesianRecordRejected(local_result.status);
        if (result != NULL) {
            *result = local_result;
        }
        return ARM_MOTION_RESULT_INVALID;
    }
    if (!ArmCartesianMotorsReady()) {
        ArmCartesianRecordRejected(local_result.status);
        if (result != NULL) {
            *result = local_result;
        }
        return ARM_MOTION_RESULT_NOT_READY;
    }
    if (arm_cartesian_runtime.active_tool_pitch_valid == 0u &&
        !ArmCartesianResolveToolPitch(0u, 0.0f)) {
        ArmCartesianRecordRejected(ARM_IK_COLLISION_RISK);
        return ARM_MOTION_RESULT_NOT_READY;
    }

    seed_q_deg[0] = g_arm_state.q_feedback_deg[ARM_JOINT_BASE_YAW];
    seed_q_deg[1] = g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER];
    seed_q_deg[2] = g_arm_state.q_feedback_deg[ARM_JOINT_ELBOW];
    if (ArmInverseKinematics3DOF(target, seed_q_deg, &local_result) !=
            ARM_IK_OK ||
        local_result.position_error_mm > ARM_LINEAR_FK_ERROR_MAX_MM ||
        !ArmJointPoseWithinSoftLimits(local_result.q_deg) ||
        !ArmCartesianAutoPoseIsSafe(local_result.q_deg) ||
        !ArmCartesianToolPitchAllowedForQ(local_result.q_deg) ||
        !ArmWorkspaceJointPathSafe(seed_q_deg, local_result.q_deg)) {
        ArmCartesianRecordRejected(local_result.status);
        if (result != NULL) {
            *result = local_result;
        }
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }
    ArmCartesianResetSafetyRoute();

    /* All checks finish before the three references are changed. */
    if (!ArmUpdateJointReference(local_result.q_deg)) {
        ArmCartesianRecordRejected(local_result.status);
        if (result != NULL) {
            *result = local_result;
        }
        return ARM_MOTION_RESULT_NOT_READY;
    }
    arm_cartesian_runtime.active_control_point =
        ARM_CONTROL_POINT_WRIST_CENTER;
    arm_cartesian_runtime.active_target_wrist = *target;
    arm_cartesian_runtime.active_target_tool_tip = *target;
    arm_cartesian_runtime.sample_count = 1u;
    memcpy(arm_cartesian_runtime.sample_q_deg[0], local_result.q_deg,
           sizeof(local_result.q_deg));

    memcpy(g_arm_motion_debug.trajectory_q_deg, local_result.q_deg,
           sizeof(g_arm_motion_debug.trajectory_q_deg));
    memcpy(g_arm_motion_debug.target_q_deg, local_result.q_deg,
           sizeof(g_arm_motion_debug.target_q_deg));
    g_arm_motion_debug.target_position_mm = *target;
    g_arm_motion_debug.ik_status = ARM_IK_OK;
    g_arm_motion_debug.path_preflight_passed = 1u;
    g_arm_motion_debug.command_accepted = 1u;
    g_arm_motion_debug.trajectory_progress = 1.0f;
    g_arm_motion_debug.trajectory_duration_ms = 0u;
    g_arm_motion_debug.trajectory_elapsed_ms = 0u;
    g_arm_motion_debug.path_sample_count = 1u;
    g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_NONE;
    arm_cartesian_runtime.arrival_stable_tick = 0u;
    ArmCartesianSetState(ARM_MOTION_SETTLING, HAL_GetTick());
    if (result != NULL) {
        *result = local_result;
    }
    return ARM_MOTION_RESULT_OK;
}

Arm_Motion_Result_e ArmMoveLinear(const Arm_Position_s *target,
                                  float max_speed_mm_s)
{
    Arm_IK_Result_s ik_result;
    Arm_Position_s sample_position;
    float start_q_deg[3];
    float previous_q_deg[3];
    Arm_Position_s previous_tool_center;
    float path_length_mm;
    uint16_t sample_count;
    uint32_t duration_ms;
    uint32_t now_ms = HAL_GetTick();

    memset(&ik_result, 0, sizeof(ik_result));
    ik_result.status = ARM_IK_INVALID_ARGUMENT;
    if (g_arm_motion_debug.motion_state == ARM_MOTION_STAGING ||
        g_arm_motion_debug.motion_state == ARM_MOTION_RUNNING ||
        g_arm_motion_debug.motion_state == ARM_MOTION_PREFLIGHT ||
        g_arm_motion_debug.motion_state == ARM_MOTION_SETTLING) {
        return ARM_MOTION_RESULT_BUSY;
    }
    if (target == NULL || !isfinite(target->x_mm) ||
        !isfinite(target->y_mm) || !isfinite(target->z_mm) ||
        !isfinite(max_speed_mm_s) ||
        max_speed_mm_s < ARM_LINEAR_MIN_SPEED_MM_S ||
        max_speed_mm_s > ARM_LINEAR_MAX_SPEED_MM_S) {
        ArmCartesianRecordRejected(ik_result.status);
        return ARM_MOTION_RESULT_INVALID;
    }
    if (!ArmCartesianMotorsReady()) {
        ArmCartesianRecordRejected(ik_result.status);
        return ARM_MOTION_RESULT_NOT_READY;
    }
    if (arm_cartesian_runtime.active_tool_pitch_valid == 0u &&
        !ArmCartesianResolveToolPitch(0u, 0.0f)) {
        ArmCartesianRecordRejected(ARM_IK_COLLISION_RISK);
        return ARM_MOTION_RESULT_NOT_READY;
    }
    start_q_deg[0] = g_arm_state.q_feedback_deg[ARM_JOINT_BASE_YAW];
    start_q_deg[1] = g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER];
    start_q_deg[2] = g_arm_state.q_feedback_deg[ARM_JOINT_ELBOW];
    if (!ArmJointPoseWithinSoftLimits(start_q_deg) ||
        !ArmCartesianAutoPoseIsSafe(start_q_deg) ||
        !ArmCartesianToolPitchAllowedForQ(start_q_deg)) {
        ArmCartesianRecordRejected(ARM_IK_COLLISION_RISK);
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }
    ArmCartesianResetSafetyRoute();

    ArmForwardKinematics3DOF(start_q_deg[0], start_q_deg[1], start_q_deg[2],
                             &arm_cartesian_runtime.start_position);
    path_length_mm = ArmCartesianPositionDistance(
        &arm_cartesian_runtime.start_position, target);
    if (!isfinite(path_length_mm)) {
        ArmCartesianRecordRejected(ARM_IK_NUMERICAL_ERROR);
        return ARM_MOTION_RESULT_INVALID;
    }
    if (path_length_mm <= ARM_LINEAR_MIN_DISTANCE_MM) {
        return ArmSetCartesianTarget(target, NULL);
    }

    sample_count = (uint16_t)ceilf(path_length_mm /
                                  ARM_LINEAR_SAMPLE_SPACING_MM) + 1u;
    if (sample_count < 2u || sample_count > ARM_LINEAR_MAX_SAMPLES) {
        ArmCartesianRecordRejected(ARM_IK_INVALID_ARGUMENT);
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }

    arm_cartesian_runtime.sample_count = sample_count;
    arm_cartesian_runtime.active_control_point =
        ARM_CONTROL_POINT_WRIST_CENTER;
    memcpy(arm_cartesian_runtime.sample_q_deg[0], start_q_deg,
           sizeof(start_q_deg));
    memcpy(previous_q_deg, start_q_deg, sizeof(previous_q_deg));
    if (!ArmForwardKinematicsToolCenter(
            previous_q_deg, arm_cartesian_runtime.active_tool_pitch_deg,
            &previous_tool_center)) {
        ArmCartesianRecordRejected(ARM_IK_NUMERICAL_ERROR);
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }
    for (uint16_t i = 1u; i < sample_count; ++i) {
        float ratio = (float)i / (float)(sample_count - 1u);
        Arm_Position_s sample_tool_center;

        sample_position.x_mm = arm_cartesian_runtime.start_position.x_mm +
            ratio * (target->x_mm -
                     arm_cartesian_runtime.start_position.x_mm);
        sample_position.y_mm = arm_cartesian_runtime.start_position.y_mm +
            ratio * (target->y_mm -
                     arm_cartesian_runtime.start_position.y_mm);
        sample_position.z_mm = arm_cartesian_runtime.start_position.z_mm +
            ratio * (target->z_mm -
                     arm_cartesian_runtime.start_position.z_mm);
        if (ArmInverseKinematics3DOF(&sample_position, previous_q_deg,
                                    &ik_result) != ARM_IK_OK ||
            ik_result.position_error_mm > ARM_LINEAR_FK_ERROR_MAX_MM ||
            !ArmJointPoseWithinSoftLimits(ik_result.q_deg) ||
            !ArmCartesianAutoPoseIsSafe(ik_result.q_deg) ||
            !ArmCartesianToolPitchAllowedForQ(ik_result.q_deg) ||
            !ArmWorkspacePoseSafe(ik_result.q_deg,
                                  i == sample_count - 1u ? 1u : 0u,
                                  &sample_tool_center) ||
            !ArmCartesianJointStepContinuous(previous_q_deg,
                                             ik_result.q_deg)) {
            ArmCartesianRecordRejected(ik_result.status);
            return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
        }
        if (ArmWorkspaceCrossesBoundary(&previous_tool_center,
                                        &sample_tool_center) &&
            fminf(previous_tool_center.z_mm, sample_tool_center.z_mm) <
                ARM_REAR_CROSSING_TOOL_Z_MM) {
            g_arm_motion_debug.workspace_safety_result =
                ARM_WORKSPACE_SAFETY_CROSSING_TOO_LOW;
            return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
        }
        previous_tool_center = sample_tool_center;
        memcpy(arm_cartesian_runtime.sample_q_deg[i], ik_result.q_deg,
               sizeof(ik_result.q_deg));
        memcpy(previous_q_deg, ik_result.q_deg, sizeof(previous_q_deg));
    }

    duration_ms = ArmCartesianDurationMs(path_length_mm, max_speed_mm_s,
        ARM_LINEAR_MAX_ACCEL_MM_S2, sample_count);
    if (!ArmBeginJointMove(previous_q_deg)) {
        ArmCartesianRecordRejected(ARM_IK_INVALID_ARGUMENT);
        return ARM_MOTION_RESULT_NOT_READY;
    }
    ArmCartesianStartPreparedTrajectory(
        target, duration_ms, max_speed_mm_s, ARM_LINEAR_MAX_ACCEL_MM_S2,
        ARM_TRAJECTORY_PATH_CARTESIAN_LINEAR,
        ARM_MOTION_RUNNING, now_ms);
    return ARM_MOTION_RESULT_OK;
}

Arm_Motion_Result_e ArmSetToolCenterTarget(
    const Arm_Position_s *target_center,
    Arm_Tool_Center_IK_Result_s *result)
{
    Arm_Tool_Center_IK_Result_s local_result;
    float seed_q_deg[3];

    memset(&local_result, 0, sizeof(local_result));
    local_result.status = ARM_IK_INVALID_ARGUMENT;
    if (target_center == NULL || !isfinite(target_center->x_mm) ||
        !isfinite(target_center->y_mm) ||
        !isfinite(target_center->z_mm)) {
        ArmCartesianRecordRejected(local_result.status);
        if (result != NULL) {
            *result = local_result;
        }
        return ARM_MOTION_RESULT_INVALID;
    }
    seed_q_deg[0] = g_arm_state.q_feedback_deg[ARM_JOINT_BASE_YAW];
    seed_q_deg[1] = g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER];
    seed_q_deg[2] = g_arm_state.q_feedback_deg[ARM_JOINT_ELBOW];
    if (!ArmWorkspacePointSafe(target_center, 1u) ||
        ArmCartesianInverseToolCenter(
            target_center, arm_cartesian_runtime.active_tool_pitch_deg,
            seed_q_deg, &local_result) != ARM_IK_OK ||
        local_result.position_error_mm > ARM_LINEAR_FK_ERROR_MAX_MM ||
        !ArmJointPoseWithinSoftLimits(local_result.q_deg) ||
        !ArmCartesianAutoPoseIsSafe(local_result.q_deg) ||
        !ArmCartesianToolPitchAllowedForQ(local_result.q_deg) ||
        !ArmWorkspaceJointPathSafe(seed_q_deg, local_result.q_deg)) {
        ArmCartesianRecordRejected(local_result.status);
        if (result != NULL) {
            *result = local_result;
        }
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }

    if (!ArmUpdateJointReference(local_result.q_deg)) {
        return ARM_MOTION_RESULT_NOT_READY;
    }
    ArmCartesianResetSafetyRoute();
    arm_cartesian_runtime.active_control_point = ARM_CONTROL_POINT_TOOL_CENTER;
    arm_cartesian_runtime.active_target_tool_tip = *target_center;
    arm_cartesian_runtime.active_target_wrist = local_result.wrist_center_mm;
    arm_cartesian_runtime.start_position = *target_center;
    arm_cartesian_runtime.target_position = *target_center;
    arm_cartesian_runtime.sample_count = 1u;
    memcpy(arm_cartesian_runtime.sample_q_deg[0], local_result.q_deg,
           sizeof(local_result.q_deg));
    memcpy(g_arm_motion_debug.trajectory_q_deg, local_result.q_deg,
           sizeof(local_result.q_deg));
    memcpy(g_arm_motion_debug.target_q_deg, local_result.q_deg,
           sizeof(local_result.q_deg));
    g_arm_motion_debug.target_position_mm = *target_center;
    g_arm_motion_debug.path_preflight_passed = 1u;
    g_arm_motion_debug.command_accepted = 1u;
    g_arm_motion_debug.trajectory_progress = 1.0f;
    g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_NONE;
    arm_cartesian_runtime.arrival_stable_tick = 0u;
    ArmCartesianSetState(ARM_MOTION_SETTLING, HAL_GetTick());
    (void)ArmCartesianTrackActiveToolPitch(local_result.q_deg, HAL_GetTick());
    if (result != NULL) {
        *result = local_result;
    }
    return ARM_MOTION_RESULT_OK;
}

static Arm_Motion_Result_e ArmMoveLinearToolCenterWithSafetyProfile(
    const Arm_Position_s *target_center,
    float max_speed_mm_s,
    Arm_Cartesian_Safety_Profile_e safety_profile)
{
    Arm_Tool_Center_IK_Result_s target_ik;
    Arm_Position_s start_center;
    Arm_Position_s route_points[ARM_TRAJECTORY_MAX_ROUTE_SEGMENTS + 1u];
    float start_q_deg[3];
    float previous_q_deg[3];
    uint8_t segment_count;
    uint8_t crossing;
    uint32_t now_ms = HAL_GetTick();
    uint32_t preflight_start_tick = now_ms;

    memset(&target_ik, 0, sizeof(target_ik));
    target_ik.status = ARM_IK_INVALID_ARGUMENT;
    g_arm_motion_debug.workspace_safety_result = ARM_WORKSPACE_SAFETY_OK;
    g_arm_motion_debug.preflight_failed_segment = 0xFFu;
    g_arm_motion_debug.preflight_failed_sample = 0u;
    g_arm_motion_debug.preflight_failed_check_mask = 0u;
    g_arm_motion_debug.preflight_duration_ms = 0u;
    g_arm_motion_debug.preflight_motor_service_count = 0u;
    g_arm_motion_debug.preflight_tool_service_count = 0u;
    memset(&g_arm_motion_debug.preflight_failed_center_mm, 0,
           sizeof(g_arm_motion_debug.preflight_failed_center_mm));
    memset(g_arm_motion_debug.preflight_failed_q_deg, 0,
           sizeof(g_arm_motion_debug.preflight_failed_q_deg));
    if (g_arm_motion_debug.motion_state == ARM_MOTION_STAGING ||
        g_arm_motion_debug.motion_state == ARM_MOTION_RUNNING ||
        g_arm_motion_debug.motion_state == ARM_MOTION_PREFLIGHT ||
        g_arm_motion_debug.motion_state == ARM_MOTION_SETTLING) {
        return ARM_MOTION_RESULT_BUSY;
    }
    if (target_center == NULL || !isfinite(target_center->x_mm) ||
        !isfinite(target_center->y_mm) ||
        !isfinite(target_center->z_mm) ||
        !isfinite(max_speed_mm_s) ||
        max_speed_mm_s < ARM_LINEAR_MIN_SPEED_MM_S ||
        max_speed_mm_s > ARM_LINEAR_MAX_SPEED_MM_S) {
        ArmCartesianRecordRejected(target_ik.status);
        return ARM_MOTION_RESULT_INVALID;
    }
    if (!ArmCartesianMotorsReady()) {
        ArmCartesianRecordRejected(target_ik.status);
        return ARM_MOTION_RESULT_NOT_READY;
    }

    start_q_deg[0] = g_arm_state.q_feedback_deg[ARM_JOINT_BASE_YAW];
    start_q_deg[1] = g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER];
    start_q_deg[2] = g_arm_state.q_feedback_deg[ARM_JOINT_ELBOW];
    if (!ArmJointPoseWithinSoftLimits(start_q_deg) ||
        !ArmCartesianAutoPoseIsSafe(start_q_deg) ||
        !ArmCartesianToolPitchAllowedForQ(start_q_deg) ||
        !ArmForwardKinematicsToolCenter(
            start_q_deg, arm_cartesian_runtime.active_tool_pitch_deg,
            &start_center)) {
        ArmCartesianRecordRejected(ARM_IK_COLLISION_RISK);
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }
    if (ArmWorkspacePointIsRear(&start_center) &&
        start_center.z_mm < ARM_REAR_ZONE_MIN_TOOL_Z_MM) {
        if (fabsf(target_center->x_mm - start_center.x_mm) > 0.5f ||
            fabsf(target_center->y_mm - start_center.y_mm) > 0.5f ||
            target_center->z_mm < ARM_REAR_CROSSING_TOOL_Z_MM) {
            g_arm_motion_debug.workspace_safety_result =
                ARM_WORKSPACE_SAFETY_ESCAPE_ONLY;
            return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
        }
    } else if (!ArmWorkspacePointSafe(&start_center, 0u)) {
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }
    if (!ArmWorkspacePointSafe(target_center, 1u)) {
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }
    if (ArmCartesianInverseToolCenter(
            target_center, arm_cartesian_runtime.active_tool_pitch_deg,
            start_q_deg, &target_ik) != ARM_IK_OK) {
        g_arm_motion_debug.workspace_safety_result =
            ARM_WORKSPACE_SAFETY_PREFLIGHT_IK;
        ArmCartesianRecordRejected(target_ik.status);
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }

    crossing = ArmWorkspaceCrossesBoundary(&start_center, target_center);
    route_points[0] = start_center;
    if (crossing != 0u) {
        float clearance_z = fmaxf(ARM_REAR_CROSSING_TOOL_Z_MM,
            fmaxf(start_center.z_mm, target_center->z_mm));
        float start_radius = sqrtf(start_center.x_mm * start_center.x_mm +
                                   start_center.y_mm * start_center.y_mm);
        float target_radius = sqrtf(target_center->x_mm *
                                    target_center->x_mm +
                                    target_center->y_mm *
                                    target_center->y_mm);
        float start_sign = ArmWorkspacePointIsRear(&start_center) ?
            -1.0f : 1.0f;
        float target_sign = ArmWorkspacePointIsRear(target_center) ?
            -1.0f : 1.0f;
        uint8_t arc_steps = (uint8_t)(90.0f /
            ARM_REAR_BYPASS_ARC_STEP_DEG + 0.5f);

        segment_count = 0u;
        route_points[++segment_count] = start_center;
        route_points[segment_count].z_mm = clearance_z;
        /* 先沿高位圆弧转到-Y侧，保持坐标翻转前的实际绕行方向。 */
        for (uint8_t step = 1u; step <= arc_steps; ++step) {
            float angle_rad = (float)step *
                ARM_REAR_BYPASS_ARC_STEP_DEG * ARM_CARTESIAN_PI / 180.0f;
            route_points[++segment_count].x_mm =
                start_sign * start_radius * cosf(angle_rad);
            route_points[segment_count].y_mm =
                -start_sign * start_radius * sinf(angle_rad);
            route_points[segment_count].z_mm = clearance_z;
        }
        /* 在车体侧面完成正/负径向切换，大臂增角不再朝向前方栏框。 */
        route_points[++segment_count].x_mm = 0.0f;
        route_points[segment_count].y_mm = -target_sign * target_radius;
        route_points[segment_count].z_mm = clearance_z;
        /* 再沿后方圆弧转到最终XY方向。 */
        for (uint8_t step = 1u; step <= arc_steps; ++step) {
            float angle_rad = (90.0f - (float)step *
                ARM_REAR_BYPASS_ARC_STEP_DEG) * ARM_CARTESIAN_PI / 180.0f;
            route_points[++segment_count].x_mm =
                target_sign * target_radius * cosf(angle_rad);
            route_points[segment_count].y_mm =
                -target_sign * target_radius * sinf(angle_rad);
            route_points[segment_count].z_mm = clearance_z;
        }
        route_points[++segment_count] = *target_center;
    } else {
        route_points[1] = *target_center;
        segment_count = 1u;
    }

    ArmCartesianResetSafetyRoute();
    arm_cartesian_runtime.active_control_point = ARM_CONTROL_POINT_TOOL_CENTER;
    arm_cartesian_runtime.active_target_tool_tip = *target_center;
    arm_cartesian_runtime.active_target_wrist = target_ik.wrist_center_mm;
    arm_cartesian_runtime.start_position = start_center;
    arm_cartesian_runtime.target_position = *target_center;
    arm_cartesian_runtime.sample_count = 1u;
    memcpy(arm_cartesian_runtime.sample_q_deg[0], start_q_deg,
           sizeof(start_q_deg));
    memcpy(previous_q_deg, start_q_deg, sizeof(previous_q_deg));
    for (uint8_t segment = 0u; segment < segment_count; ++segment) {
        if (!ArmCartesianAppendToolCenterSegment(
                &route_points[segment], &route_points[segment + 1u],
                previous_q_deg, segment, max_speed_mm_s,
                safety_profile)) {
            g_arm_motion_debug.preflight_duration_ms =
                HAL_GetTick() - preflight_start_tick;
            return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
        }
    }
    g_arm_motion_debug.preflight_duration_ms =
        HAL_GetTick() - preflight_start_tick;
    /* 轨迹时间零点必须是预检完成时，不能沿用百毫秒前的命令提交时间。 */
    now_ms = HAL_GetTick();
    if (!ArmBeginJointMove(previous_q_deg)) {
        ArmCartesianRecordRejected(ARM_IK_INVALID_ARGUMENT);
        return ARM_MOTION_RESULT_NOT_READY;
    }
    arm_cartesian_runtime.safety_route_enabled = crossing;
    arm_cartesian_runtime.route_segment_count = segment_count;
    arm_cartesian_runtime.route_active_segment = 0u;
    arm_cartesian_runtime.route_crossing_segment = crossing != 0u ? 1u :
                                                                    0xFFu;
    arm_cartesian_runtime.trajectory_duration_ms =
        arm_cartesian_runtime.route_segment_duration_ms[0];
    g_arm_motion_debug.safety_route_enabled = crossing;
    g_arm_motion_debug.safety_route_segment_count = segment_count;
    g_arm_motion_debug.safety_route_active_segment = 0u;
    g_arm_motion_debug.workspace_safety_result = ARM_WORKSPACE_SAFETY_OK;
    ArmCartesianStartPreparedTrajectory(
        target_center, arm_cartesian_runtime.route_segment_duration_ms[0],
        max_speed_mm_s, ARM_LINEAR_MAX_ACCEL_MM_S2,
        ARM_TRAJECTORY_PATH_JOINT_STAGING,
        ARM_MOTION_RUNNING, now_ms);
    ArmCartesianStartRouteSegment(0u, now_ms);
    return ARM_MOTION_RESULT_OK;
}

Arm_Motion_Result_e ArmMoveLinearToolCenter(
    const Arm_Position_s *target_center,
    float max_speed_mm_s)
{
    return ArmMoveLinearToolCenterWithSafetyProfile(
        target_center, max_speed_mm_s, ARM_CARTESIAN_SAFETY_NORMAL);
}

Arm_Motion_Result_e ArmTrajectoryMoveJointThenLinear(
    const float waypoint_q_deg[3],
    const Arm_Position_s *target,
    float max_speed_mm_s)
{
    Arm_IK_Result_s ik_result;
    Arm_Position_s waypoint_position;
    Arm_Position_s sample_position;
    float start_q_deg[3];
    float previous_q_deg[3];
    float max_joint_delta_deg = 0.0f;
    float cartesian_length_mm;
    float effective_length_mm;
    uint16_t joint_interval_count;
    uint16_t cartesian_interval_count;
    uint16_t total_sample_count;
    uint32_t duration_ms;
    uint32_t now_ms = HAL_GetTick();

    memset(&ik_result, 0, sizeof(ik_result));
    ik_result.status = ARM_IK_INVALID_ARGUMENT;
    if (waypoint_q_deg == NULL || target == NULL ||
        !isfinite(target->x_mm) || !isfinite(target->y_mm) ||
        !isfinite(target->z_mm) || !isfinite(max_speed_mm_s) ||
        max_speed_mm_s < ARM_LINEAR_MIN_SPEED_MM_S ||
        max_speed_mm_s > ARM_LINEAR_MAX_SPEED_MM_S) {
        ArmCartesianRecordRejected(ik_result.status);
        return ARM_MOTION_RESULT_INVALID;
    }
    if (ArmTrajectoryIsBusy()) {
        return ARM_MOTION_RESULT_BUSY;
    }
    if (!ArmCartesianMotorsReady()) {
        ArmCartesianRecordRejected(ik_result.status);
        return ARM_MOTION_RESULT_NOT_READY;
    }
    if (arm_cartesian_runtime.active_tool_pitch_valid == 0u &&
        !ArmCartesianResolveToolPitch(0u, 0.0f)) {
        ArmCartesianRecordRejected(ARM_IK_COLLISION_RISK);
        return ARM_MOTION_RESULT_NOT_READY;
    }
    start_q_deg[0] = g_arm_state.q_feedback_deg[ARM_JOINT_BASE_YAW];
    start_q_deg[1] = g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER];
    start_q_deg[2] = g_arm_state.q_feedback_deg[ARM_JOINT_ELBOW];
    if (!ArmJointPoseWithinSoftLimits(start_q_deg) ||
        !ArmCartesianAutoPoseIsSafe(start_q_deg) ||
        !ArmJointPoseWithinSoftLimits(waypoint_q_deg) ||
        !ArmCartesianAutoPoseIsSafe(waypoint_q_deg) ||
        !ArmCartesianToolPitchAllowedForQ(start_q_deg) ||
        !ArmCartesianToolPitchAllowedForQ(waypoint_q_deg)) {
        ArmCartesianRecordRejected(ARM_IK_COLLISION_RISK);
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }
    ArmCartesianResetSafetyRoute();

    for (uint8_t joint = 0u; joint < 3u; ++joint) {
        float delta_deg = joint == ARM_JOINT_BASE_YAW ?
            fabsf(ArmCartesianWrapTo180(
                waypoint_q_deg[joint] - start_q_deg[joint])) :
            fabsf(waypoint_q_deg[joint] - start_q_deg[joint]);

        max_joint_delta_deg = fmaxf(max_joint_delta_deg, delta_deg);
    }
    joint_interval_count = (uint16_t)ceilf(
        max_joint_delta_deg / ARM_COMPOSITE_JOINT_STEP_DEG);
    if (joint_interval_count < 1u) {
        joint_interval_count = 1u;
    }

    ArmForwardKinematics3DOF(waypoint_q_deg[0], waypoint_q_deg[1],
                             waypoint_q_deg[2], &waypoint_position);
    cartesian_length_mm = ArmCartesianPositionDistance(&waypoint_position,
                                                       target);
    if (!isfinite(cartesian_length_mm) ||
        cartesian_length_mm <= ARM_LINEAR_MIN_DISTANCE_MM) {
        ArmCartesianRecordRejected(ARM_IK_INVALID_ARGUMENT);
        return ARM_MOTION_RESULT_INVALID;
    }
    cartesian_interval_count = (uint16_t)ceilf(
        cartesian_length_mm / ARM_LINEAR_SAMPLE_SPACING_MM);
    if (cartesian_interval_count < 1u) {
        cartesian_interval_count = 1u;
    }
    total_sample_count = (uint16_t)(1u + joint_interval_count +
                                    cartesian_interval_count);
    if (total_sample_count > ARM_LINEAR_MAX_SAMPLES) {
        ArmCartesianRecordRejected(ARM_IK_INVALID_ARGUMENT);
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }

    arm_cartesian_runtime.sample_count = total_sample_count;
    arm_cartesian_runtime.active_control_point =
        ARM_CONTROL_POINT_WRIST_CENTER;
    memcpy(arm_cartesian_runtime.sample_q_deg[0], start_q_deg,
           sizeof(start_q_deg));
    for (uint16_t i = 1u; i <= joint_interval_count; ++i) {
        float ratio = (float)i / (float)joint_interval_count;
        float *sample_q_deg = arm_cartesian_runtime.sample_q_deg[i];

        sample_q_deg[0] = ArmCartesianWrapTo180(start_q_deg[0] +
            ratio * ArmCartesianWrapTo180(waypoint_q_deg[0] -
                                           start_q_deg[0]));
        sample_q_deg[1] = start_q_deg[1] +
            ratio * (waypoint_q_deg[1] - start_q_deg[1]);
        sample_q_deg[2] = start_q_deg[2] +
            ratio * (waypoint_q_deg[2] - start_q_deg[2]);
        if (!ArmJointPoseWithinSoftLimits(sample_q_deg) ||
            !ArmCartesianAutoPoseIsSafe(sample_q_deg) ||
            !ArmCartesianToolPitchAllowedForQ(sample_q_deg)) {
            ArmCartesianRecordRejected(ARM_IK_COLLISION_RISK);
            return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
        }
    }

    memcpy(previous_q_deg, waypoint_q_deg, sizeof(previous_q_deg));
    for (uint16_t i = 1u; i <= cartesian_interval_count; ++i) {
        float ratio = (float)i / (float)cartesian_interval_count;
        uint16_t sample_index = (uint16_t)(joint_interval_count + i);

        sample_position.x_mm = waypoint_position.x_mm +
            ratio * (target->x_mm - waypoint_position.x_mm);
        sample_position.y_mm = waypoint_position.y_mm +
            ratio * (target->y_mm - waypoint_position.y_mm);
        sample_position.z_mm = waypoint_position.z_mm +
            ratio * (target->z_mm - waypoint_position.z_mm);
        if (ArmInverseKinematics3DOF(&sample_position, previous_q_deg,
                                    &ik_result) != ARM_IK_OK ||
            ik_result.position_error_mm > ARM_LINEAR_FK_ERROR_MAX_MM ||
            !ArmJointPoseWithinSoftLimits(ik_result.q_deg) ||
            !ArmCartesianAutoPoseIsSafe(ik_result.q_deg) ||
            !ArmCartesianToolPitchAllowedForQ(ik_result.q_deg) ||
            !ArmCartesianJointStepContinuous(previous_q_deg,
                                             ik_result.q_deg)) {
            ArmCartesianRecordRejected(ik_result.status);
            return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
        }
        memcpy(arm_cartesian_runtime.sample_q_deg[sample_index],
               ik_result.q_deg, sizeof(ik_result.q_deg));
        memcpy(previous_q_deg, ik_result.q_deg, sizeof(previous_q_deg));
    }

    /*
     * 精确经过安全姿态会把“主要转大臂”和“主要展小臂”
     * 两个不同切向拼成尖角。用局部三次Bezier圆角替换尖角，
     * 保留安全区域的必经约束，但不强制电机在该点换向或降速。
     */
    if (!ArmCartesianBlendCompositeWaypoint(joint_interval_count,
                                             total_sample_count)) {
        ArmCartesianRecordRejected(ARM_IK_COLLISION_RISK);
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }
    if (!ArmWorkspaceSampleBufferSafe(total_sample_count)) {
        ArmCartesianRecordRejected(ARM_IK_COLLISION_RISK);
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }

    /*
     * 两段样本共用一个五次时间轴。用等效路径长度补偿关节过渡
     * 占用的样本比例，保证后半段笛卡尔速度不超过设定值。
     */
    effective_length_mm = cartesian_length_mm *
        (float)(total_sample_count - 1u) /
        (float)cartesian_interval_count;
    duration_ms = ArmCartesianDurationMs(effective_length_mm,
        max_speed_mm_s, ARM_LINEAR_MAX_ACCEL_MM_S2, total_sample_count);
    if (!ArmBeginJointMove(previous_q_deg)) {
        ArmCartesianRecordRejected(ARM_IK_INVALID_ARGUMENT);
        return ARM_MOTION_RESULT_NOT_READY;
    }
    arm_cartesian_runtime.start_position = g_arm_state.wrist_center;
    ArmCartesianStartPreparedTrajectory(
        target, duration_ms, max_speed_mm_s, ARM_LINEAR_MAX_ACCEL_MM_S2,
        ARM_TRAJECTORY_PATH_JOINT_STAGING,
        ARM_MOTION_RUNNING, now_ms);
    return ARM_MOTION_RESULT_OK;
}

Arm_Motion_Result_e ArmTrajectoryStageCartesianCommand(
    const Arm_Cartesian_Command_s *command)
{
    if (command == NULL) {
        return ARM_MOTION_RESULT_INVALID;
    }
    if (ArmTrajectoryIsBusy()) {
        return ARM_MOTION_RESULT_BUSY;
    }
    if (!ArmCartesianMotorsReady()) {
        return ARM_MOTION_RESULT_NOT_READY;
    }
    if (command->tool_yaw_valid != 0u ||
        (command->safety_profile != ARM_CARTESIAN_SAFETY_NORMAL &&
         command->safety_profile != ARM_CARTESIAN_SAFETY_AC_SIDE_PICK)) {
        return ARM_MOTION_RESULT_INVALID;
    }
    arm_cartesian_runtime.safety_profile = command->safety_profile;
    if (!ArmCartesianResolveToolPitch(command->tool_pitch_valid,
                                      command->tool_pitch_deg)) {
        return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
    }
    if (command->move_type == ARM_MOVE_DIRECT) {
        if (command->control_point == ARM_CONTROL_POINT_TOOL_CENTER) {
            Arm_Position_s start_center;

            if (!ArmForwardKinematicsToolCenter(
                    g_arm_state.q_feedback_deg,
                    arm_cartesian_runtime.active_tool_pitch_deg,
                    &start_center)) {
                return ARM_MOTION_RESULT_PREFLIGHT_FAILED;
            }
            return ArmWorkspaceCrossesBoundary(&start_center,
                                                &command->target_mm) ?
                ArmMoveLinearToolCenterWithSafetyProfile(
                    &command->target_mm,
                    command->max_speed_mm_s > 0.0f ?
                        command->max_speed_mm_s :
                        ARM_LINEAR_DEFAULT_SPEED_MM_S,
                    command->safety_profile) :
                ArmSetToolCenterTarget(&command->target_mm, NULL);
        }
        return ArmSetCartesianTarget(&command->target_mm, NULL);
    }
    if (command->control_point == ARM_CONTROL_POINT_TOOL_CENTER) {
        return ArmMoveLinearToolCenterWithSafetyProfile(
            &command->target_mm,
            command->max_speed_mm_s > 0.0f ? command->max_speed_mm_s :
                                             ARM_LINEAR_DEFAULT_SPEED_MM_S,
            command->safety_profile);
    }
    if (command->control_point == ARM_CONTROL_POINT_WRIST_CENTER) {
        return ArmMoveLinear(&command->target_mm,
            command->max_speed_mm_s > 0.0f ? command->max_speed_mm_s :
                                             ARM_LINEAR_DEFAULT_SPEED_MM_S);
    }
    return ARM_MOTION_RESULT_INVALID;
}

void ArmTrajectoryInit(void)
{
    memset(&arm_cartesian_runtime, 0, sizeof(arm_cartesian_runtime));
    memset(&g_arm_motion_debug, 0, sizeof(g_arm_motion_debug));
    g_arm_motion_debug.motion_state = ARM_MOTION_IDLE;
    g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_NONE;
    arm_cartesian_runtime.last_task_tick = HAL_GetTick();
    arm_cartesian_runtime.kinematics_self_test_passed =
        ArmKinematicsSelfTest(NULL);
    if (!arm_cartesian_runtime.kinematics_self_test_passed) {
        g_arm_motion_debug.motion_state = ARM_MOTION_ERROR_IK;
        g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_SELF_TEST;
    }
}

static void ArmCartesianRunPreparedTrajectory(uint32_t now_ms)
{
#if ARM_WORKSPACE_SAFETY_ENABLE != 0u && \
    ARM_FRONT_BARRIER_SHOULDER_LIMIT_ENABLE != 0u
    Arm_Position_s safety_tool_center;
#endif
    Arm_Motion_Fault_e tool_pitch_fault;
    float normalized_time;
    float progress;
    float reference_q_deg[3];
    uint32_t elapsed_ms =
        (uint32_t)(now_ms - arm_cartesian_runtime.trajectory_start_tick);

#if ARM_WORKSPACE_SAFETY_ENABLE != 0u && \
    ARM_FRONT_BARRIER_SHOULDER_LIMIT_ENABLE != 0u
    /*
     * 区域保护按当前关节反馈和持续俯仰目标计算。实际舵机反馈有机械滞后，
     * 不能用其瞬时偏差把后方姿态误判成正前方；舵机越界/离线仍由
     * ArmCartesianTrackActiveToolPitch()独立中止。
     */
    if (!ArmForwardKinematicsToolCenter(
            g_arm_state.q_feedback_deg,
            ArmCartesianToolPitchForQ(g_arm_state.q_feedback_deg),
            &safety_tool_center)) {
        ArmAbortMotion(ARM_MOTION_FAULT_COLLISION);
        return;
    }
    if (ArmWorkspaceBaseFacesFront(
            g_arm_state.q_feedback_deg[ARM_JOINT_BASE_YAW]) &&
        safety_tool_center.x_mm >
            ARM_FRONT_BARRIER_TOOL_X_MARGIN_MM &&
        g_arm_state.q_feedback_deg[ARM_JOINT_SHOULDER] >
            ARM_FRONT_BARRIER_SHOULDER_Q2_MAX_DEG) {
        g_arm_motion_debug.workspace_safety_result =
            ARM_WORKSPACE_SAFETY_FRONT_SHOULDER_LIMIT;
        ArmTrajectoryCancel();
        g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_COLLISION;
        g_arm_motion_debug.motion_state = ARM_MOTION_ABORTED;
        g_arm_motion_debug.command_accepted = 0u;
        return;
    }
#endif

    if (elapsed_ms >= arm_cartesian_runtime.trajectory_duration_ms) {
        elapsed_ms = arm_cartesian_runtime.trajectory_duration_ms;
    }
    normalized_time = arm_cartesian_runtime.trajectory_duration_ms > 0u ?
        (float)elapsed_ms /
            (float)arm_cartesian_runtime.trajectory_duration_ms : 1.0f;
    progress = ArmCartesianQuintic(normalized_time);
    if (arm_cartesian_runtime.safety_route_enabled != 0u ||
        arm_cartesian_runtime.route_segment_count > 1u) {
        if (arm_cartesian_runtime.safety_route_enabled != 0u &&
            arm_cartesian_runtime.route_active_segment > 0u &&
            g_arm_state.tool_tip.z_mm < ARM_REAR_CROSSING_ABORT_Z_MM) {
            g_arm_motion_debug.workspace_safety_result =
                ARM_WORKSPACE_SAFETY_RUNTIME_HEIGHT;
            ArmTrajectoryCancel();
            g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_COLLISION;
            g_arm_motion_debug.motion_state = ARM_MOTION_ABORTED;
            g_arm_motion_debug.command_accepted = 0u;
            return;
        }
        ArmCartesianInterpolateRouteSegment(progress, reference_q_deg);
    } else if (arm_cartesian_runtime.path_type ==
        ARM_TRAJECTORY_PATH_CARTESIAN_LINEAR) {
        if (!arm_cartesian_runtime.online_ik_valid ||
            (uint32_t)(now_ms - arm_cartesian_runtime.last_ik_tick) >=
                ARM_LINEAR_IK_UPDATE_MS) {
            /* 在线IK失败时冻结上一合法参考，电机继续角度闭环保持。 */
            if (!ArmCartesianSolveOnlineIK(now_ms, progress) &&
                !arm_cartesian_runtime.online_ik_valid) {
                arm_cartesian_runtime.trajectory_start_tick++;
                return;
            }
        }
        ArmCartesianInterpolateOnlineIK(now_ms, reference_q_deg);
    } else {
        ArmCartesianInterpolateJointSamples(progress, reference_q_deg);
    }
    if (!ArmUpdateJointReference(reference_q_deg)) {
        if (!arm_cartesian_runtime.reference_update_rejected) {
            g_arm_motion_debug.command_reject_count++;
            arm_cartesian_runtime.reference_update_rejected = 1u;
        }
        g_arm_motion_debug.command_accepted = 0u;
        arm_cartesian_runtime.trajectory_start_tick++;
        return;
    }
    tool_pitch_fault = ArmCartesianTrackActiveToolPitch(
        reference_q_deg, now_ms);
    if (tool_pitch_fault != ARM_MOTION_FAULT_NONE) {
        ArmAbortMotion(tool_pitch_fault);
        return;
    }

    arm_cartesian_runtime.reference_update_rejected = 0u;
    g_arm_motion_debug.command_accepted = 1u;
    g_arm_motion_debug.trajectory_elapsed_ms = elapsed_ms;
    g_arm_motion_debug.trajectory_progress = progress;
    memcpy(g_arm_motion_debug.trajectory_q_deg, reference_q_deg,
           sizeof(g_arm_motion_debug.trajectory_q_deg));

    if (elapsed_ms >= arm_cartesian_runtime.trajectory_duration_ms) {
        uint16_t last_index = arm_cartesian_runtime.route_segment_count > 0u ?
            arm_cartesian_runtime.route_segment_end[
                arm_cartesian_runtime.route_active_segment] :
            (uint16_t)(arm_cartesian_runtime.sample_count - 1u);

        ArmUpdateJointReference(
            arm_cartesian_runtime.sample_q_deg[last_index]);
        tool_pitch_fault = ArmCartesianTrackActiveToolPitch(
            arm_cartesian_runtime.sample_q_deg[last_index], now_ms);
        if (tool_pitch_fault != ARM_MOTION_FAULT_NONE) {
            ArmAbortMotion(tool_pitch_fault);
            return;
        }
        memcpy(g_arm_motion_debug.trajectory_q_deg,
               arm_cartesian_runtime.sample_q_deg[last_index],
               sizeof(g_arm_motion_debug.trajectory_q_deg));
        g_arm_motion_debug.trajectory_progress = 1.0f;
        /*
         * 参考曲线结束不等于机构实际到位。保持最终关节目标并进入反馈
         * 收敛阶段；只有三轴误差和速度连续稳定后才允许Host完成上报。
         */
        arm_cartesian_runtime.arrival_stable_tick = 0u;
        ArmCartesianSetState(ARM_MOTION_SETTLING, now_ms);
    }
}

static void ArmCartesianRunSettling(uint32_t now_ms)
{
    Arm_Motion_Fault_e tool_pitch_fault;
    uint16_t last_index;
    uint8_t intermediate_route_waypoint;
    float arrival_error_limit_deg = ARM_ARRIVAL_ERROR_DEG;
    float arrival_speed_limit_deg_s = ARM_ARRIVAL_SPEED_DEG_S;
    uint32_t arrival_stable_ms = ARM_ARRIVAL_STABLE_MS;

    if (arm_cartesian_runtime.sample_count == 0u) {
        g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_TIMEOUT;
        g_arm_motion_debug.command_accepted = 0u;
        ArmCartesianSetState(ARM_MOTION_ERROR_TIMEOUT, now_ms);
        return;
    }

    intermediate_route_waypoint =
        arm_cartesian_runtime.route_segment_count > 1u &&
        (uint8_t)(arm_cartesian_runtime.route_active_segment + 1u) <
            arm_cartesian_runtime.route_segment_count;
    if (intermediate_route_waypoint != 0u) {
        /*
         * route中间点只承担避障/绕行职责，不是作业终点。用宽松判定
         * 提前切段，避免每个安全位都等到全局2deg/5degps/120ms后
         * 才继续运动；最终段仍使用严格到位判定。
         */
        arrival_error_limit_deg = ARM_ROUTE_WAYPOINT_ARRIVAL_ERROR_DEG;
        arrival_speed_limit_deg_s =
            ARM_ROUTE_WAYPOINT_ARRIVAL_SPEED_DEG_S;
        arrival_stable_ms = ARM_ROUTE_WAYPOINT_ARRIVAL_STABLE_MS;
    }

    last_index = arm_cartesian_runtime.route_segment_count > 0u ?
        arm_cartesian_runtime.route_segment_end[
            arm_cartesian_runtime.route_active_segment] :
        (uint16_t)(arm_cartesian_runtime.sample_count - 1u);
    if (!ArmUpdateJointReference(
            arm_cartesian_runtime.sample_q_deg[last_index])) {
        arm_cartesian_runtime.arrival_stable_tick = 0u;
    } else {
        tool_pitch_fault = ArmCartesianTrackActiveToolPitch(
            arm_cartesian_runtime.sample_q_deg[last_index], now_ms);
        if (tool_pitch_fault != ARM_MOTION_FAULT_NONE) {
            ArmAbortMotion(tool_pitch_fault);
            return;
        }
        if (ArmCartesianFeedbackWithinArrivalLimits(
                arrival_error_limit_deg, arrival_speed_limit_deg_s)) {
            if (arm_cartesian_runtime.arrival_stable_tick == 0u) {
                arm_cartesian_runtime.arrival_stable_tick = now_ms;
            }
            if ((uint32_t)(now_ms -
                    arm_cartesian_runtime.arrival_stable_tick) >=
                arrival_stable_ms) {
                if (arm_cartesian_runtime.route_segment_count > 1u &&
                    (uint8_t)(
                        arm_cartesian_runtime.route_active_segment + 1u) <
                        arm_cartesian_runtime.route_segment_count) {
                    uint8_t next_segment = (uint8_t)(
                        arm_cartesian_runtime.route_active_segment + 1u);

                    if (next_segment ==
                            arm_cartesian_runtime.route_crossing_segment &&
                        g_arm_state.tool_tip.z_mm <
                            ARM_REAR_CROSSING_ACTUAL_GATE_Z_MM) {
                        arm_cartesian_runtime.arrival_stable_tick = 0u;
                        return;
                    }
                    ArmCartesianStartRouteSegment(next_segment, now_ms);
                    return;
                }
                g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_NONE;
                g_arm_motion_debug.command_accepted = 1u;
                ArmCartesianSetState(ARM_MOTION_HOLDING, now_ms);
                return;
            }
        } else {
            arm_cartesian_runtime.arrival_stable_tick = 0u;
        }
    }

    if ((uint32_t)(now_ms - arm_cartesian_runtime.state_start_tick) >=
        ARM_TRAJECTORY_SETTLE_TIMEOUT_MS) {
        g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_TIMEOUT;
        g_arm_motion_debug.command_accepted = 0u;
        ArmCartesianSetState(ARM_MOTION_ERROR_TIMEOUT, now_ms);
    }
}

static void ArmCartesianRunRealtime(uint32_t now_ms, uint32_t task_delta_ms)
{
    Arm_IK_Result_s result;
    Arm_Motion_Fault_e tool_pitch_fault;
    Arm_Position_s candidate_position;
    float dt_s = (float)task_delta_ms * 0.001f;
    float position_error[3];
    float desired_velocity[3];
    float acceleration[3];
    float remaining_distance;
    float reference_step[3];
    float reference_q_deg[3];

    memcpy(arm_cartesian_runtime.realtime_last_velocity_mm_s,
           arm_cartesian_runtime.realtime_velocity_mm_s,
           sizeof(arm_cartesian_runtime.realtime_last_velocity_mm_s));

    if ((uint32_t)(now_ms -
                   arm_cartesian_runtime.realtime_last_command_tick) >
        ARM_REALTIME_COMMAND_TIMEOUT_MS) {
        arm_cartesian_runtime.realtime_timed_out = 1u;
        memset(arm_cartesian_runtime.realtime_velocity_mm_s, 0,
               sizeof(arm_cartesian_runtime.realtime_velocity_mm_s));
        /* 断流冻结当前生成参考，角度环保持，不退出实时模式也不失能。 */
        return;
    }
    arm_cartesian_runtime.realtime_timed_out = 0u;
    if (dt_s <= 0.0f) {
        return;
    }

    position_error[0] = arm_cartesian_runtime.realtime_target.target_mm.x_mm -
        arm_cartesian_runtime.realtime_reference_position.x_mm;
    position_error[1] = arm_cartesian_runtime.realtime_target.target_mm.y_mm -
        arm_cartesian_runtime.realtime_reference_position.y_mm;
    position_error[2] = arm_cartesian_runtime.realtime_target.target_mm.z_mm -
        arm_cartesian_runtime.realtime_reference_position.z_mm;
    remaining_distance = ArmCartesianVectorLength(position_error);
    {
        float braking_speed = sqrtf(2.0f *
            arm_cartesian_runtime.realtime_target.max_acceleration_mm_s2 *
            remaining_distance);
        float desired_speed = fminf(
            arm_cartesian_runtime.realtime_target.max_speed_mm_s,
            braking_speed);

        memcpy(desired_velocity, position_error, sizeof(desired_velocity));
        ArmCartesianSetVectorLength(desired_velocity, desired_speed);
    }
    acceleration[0] = (desired_velocity[0] -
        arm_cartesian_runtime.realtime_velocity_mm_s[0]) / dt_s;
    acceleration[1] = (desired_velocity[1] -
        arm_cartesian_runtime.realtime_velocity_mm_s[1]) / dt_s;
    acceleration[2] = (desired_velocity[2] -
        arm_cartesian_runtime.realtime_velocity_mm_s[2]) / dt_s;
    ArmCartesianLimitVector(acceleration,
        arm_cartesian_runtime.realtime_target.max_acceleration_mm_s2);
    for (uint8_t axis = 0u; axis < 3u; ++axis) {
        arm_cartesian_runtime.realtime_velocity_mm_s[axis] +=
            acceleration[axis] * dt_s;
    }
    ArmCartesianLimitVector(arm_cartesian_runtime.realtime_velocity_mm_s,
        arm_cartesian_runtime.realtime_target.max_speed_mm_s);

    reference_step[0] = arm_cartesian_runtime.realtime_velocity_mm_s[0] *
        dt_s;
    reference_step[1] = arm_cartesian_runtime.realtime_velocity_mm_s[1] *
        dt_s;
    reference_step[2] = arm_cartesian_runtime.realtime_velocity_mm_s[2] *
        dt_s;
    /*
     * 最新目标覆盖模式下，参考点是用速度积分生成的。若已经到达或下一步
     * 会越过目标，直接夹到目标点并清速度，避免积分冲出工作空间后触发
     * IK_OUT_OF_REACH，看起来像“到一半停住”。
     */
    if (remaining_distance <= 0.05f ||
        reference_step[0] * position_error[0] +
        reference_step[1] * position_error[1] +
        reference_step[2] * position_error[2] >=
            remaining_distance * remaining_distance) {
        arm_cartesian_runtime.realtime_reference_position =
            arm_cartesian_runtime.realtime_target.target_mm;
        memset(arm_cartesian_runtime.realtime_velocity_mm_s, 0,
               sizeof(arm_cartesian_runtime.realtime_velocity_mm_s));
    } else {
        arm_cartesian_runtime.realtime_reference_position.x_mm +=
            reference_step[0];
        arm_cartesian_runtime.realtime_reference_position.y_mm +=
            reference_step[1];
        arm_cartesian_runtime.realtime_reference_position.z_mm +=
            reference_step[2];
    }
    candidate_position = arm_cartesian_runtime.realtime_reference_position;
    if ((uint32_t)(now_ms - arm_cartesian_runtime.last_ik_tick) >=
        ARM_LINEAR_IK_UPDATE_MS) {
        Arm_Realtime_Reject_Reason_e reject_reason =
            ARM_REALTIME_REJECT_NONE;

        memset(&result, 0, sizeof(result));
        if (ArmCartesianSolveControlPoint(
                arm_cartesian_runtime.realtime_control_point,
                &candidate_position,
                arm_cartesian_runtime.online_ik_next_q_deg,
                &result, NULL) != ARM_IK_OK) {
            reject_reason = ARM_REALTIME_REJECT_IK;
        } else if (result.position_error_mm > ARM_LINEAR_FK_ERROR_MAX_MM) {
            reject_reason = ARM_REALTIME_REJECT_FK_ERROR;
        } else if (!ArmJointPoseWithinSoftLimits(result.q_deg)) {
            reject_reason = ARM_REALTIME_REJECT_SOFT_LIMIT;
        } else if (!ArmCartesianAutoPoseIsSafe(result.q_deg)) {
            reject_reason = ARM_REALTIME_REJECT_AUTO_REGION;
        } else if (!ArmCartesianToolPitchAllowedForQ(result.q_deg)) {
            reject_reason = ARM_REALTIME_REJECT_AUTO_REGION;
        } else if (!ArmWorkspacePoseSafe(result.q_deg, 0u, NULL)) {
            reject_reason = ARM_REALTIME_REJECT_WORKSPACE_SAFETY;
        } else if (!ArmCartesianJointStepContinuous(
                       arm_cartesian_runtime.online_ik_next_q_deg,
                       result.q_deg)) {
            reject_reason = ARM_REALTIME_REJECT_CONTINUITY;
        }

        if (reject_reason == ARM_REALTIME_REJECT_NONE) {
            memcpy(arm_cartesian_runtime.online_ik_previous_q_deg,
                   arm_cartesian_runtime.online_ik_next_q_deg,
                   sizeof(arm_cartesian_runtime.online_ik_previous_q_deg));
            memcpy(arm_cartesian_runtime.online_ik_next_q_deg,
                   result.q_deg, sizeof(result.q_deg));
            arm_cartesian_runtime.realtime_last_valid_position =
                candidate_position;
            if (arm_cartesian_runtime.realtime_control_point ==
                ARM_CONTROL_POINT_TOOL_CENTER) {
                arm_cartesian_runtime.realtime_last_valid_tool_tip =
                    candidate_position;
            }
            arm_cartesian_runtime.online_ik_interval_start_tick = now_ms;
            arm_cartesian_runtime.online_ik_interval_ms =
                ARM_LINEAR_IK_UPDATE_MS;
            arm_cartesian_runtime.last_ik_tick = now_ms;
            g_arm_motion_debug.ik_status = ARM_IK_OK;
            g_arm_motion_debug.command_accepted = 1u;
            g_arm_motion_debug.realtime_reject_reason =
                ARM_REALTIME_REJECT_NONE;
        } else {
            /* 局部IK拒绝时保留上一合法位置和角度参考。 */
            memset(arm_cartesian_runtime.realtime_velocity_mm_s, 0,
                   sizeof(arm_cartesian_runtime.realtime_velocity_mm_s));
            arm_cartesian_runtime.realtime_reference_position =
                arm_cartesian_runtime.realtime_last_valid_position;
            g_arm_motion_debug.ik_status = result.status;
            g_arm_motion_debug.command_accepted = 0u;
            g_arm_motion_debug.realtime_reject_reason = reject_reason;
            g_arm_motion_debug.last_realtime_reject_reason = reject_reason;
            g_arm_motion_debug.command_reject_count++;
        }
    }

    ArmCartesianInterpolateOnlineIK(now_ms, reference_q_deg);
    if (ArmUpdateJointReference(reference_q_deg)) {
        memcpy(g_arm_motion_debug.trajectory_q_deg, reference_q_deg,
               sizeof(reference_q_deg));
        tool_pitch_fault = ArmCartesianTrackActiveToolPitch(
            reference_q_deg, now_ms);
        if (tool_pitch_fault != ARM_MOTION_FAULT_NONE) {
            ArmAbortMotion(tool_pitch_fault);
        }
    }
    g_arm_motion_debug.trajectory_elapsed_ms =
        (uint32_t)(now_ms - arm_cartesian_runtime.trajectory_start_tick);
    g_arm_motion_debug.trajectory_progress = 0.0f;
}

void ArmTrajectoryTask(uint32_t now_ms)
{
    uint32_t task_delta_ms =
        (uint32_t)(now_ms - arm_cartesian_runtime.last_task_tick);

    arm_cartesian_runtime.last_task_tick = now_ms;
    if (!arm_cartesian_runtime.kinematics_self_test_passed) {
        ArmCartesianUpdateDebug();
        ArmCartesianUpdateControlDebug(now_ms, task_delta_ms);
        return;
    }
    if (!ArmCartesianMotorsReady()) {
        /* Pause trajectory time and keep the previous references. */
        if (g_arm_motion_debug.motion_state == ARM_MOTION_STAGING ||
            g_arm_motion_debug.motion_state == ARM_MOTION_RUNNING) {
            arm_cartesian_runtime.trajectory_start_tick += task_delta_ms;
        } else if (g_arm_motion_debug.motion_state == ARM_MOTION_HOLDING) {
            arm_cartesian_runtime.state_start_tick += task_delta_ms;
        }
        g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_OFFLINE;
        g_arm_motion_debug.command_accepted = 0u;
        ArmCartesianUpdateDebug();
        ArmCartesianUpdateControlDebug(now_ms, task_delta_ms);
        return;
    }
    if (g_arm_motion_debug.fault_code == ARM_MOTION_FAULT_OFFLINE) {
        g_arm_motion_debug.fault_code = ARM_MOTION_FAULT_NONE;
    }

    if (arm_cartesian_runtime.realtime_active) {
        ArmCartesianRunRealtime(now_ms, task_delta_ms);
        ArmCartesianUpdateDebug();
        ArmCartesianUpdateControlDebug(now_ms, task_delta_ms);
        return;
    }

    switch (g_arm_motion_debug.motion_state) {
        case ARM_MOTION_IDLE:
        case ARM_MOTION_BOOT_DELAY:
            ArmCartesianSetState(ARM_MOTION_HOLDING, now_ms);
            break;

        case ARM_MOTION_STAGING:
        case ARM_MOTION_RUNNING:
            ArmCartesianRunPreparedTrajectory(now_ms);
            break;

        case ARM_MOTION_SETTLING:
            ArmCartesianRunSettling(now_ms);
            break;

        case ARM_MOTION_HOLDING:
            break;

        case ARM_MOTION_ABORTED:
        case ARM_MOTION_COMPLETE:
        case ARM_MOTION_ERROR_IK:
            break;

        default:
            /* A rejected command holds the last valid target and stays enabled. */
            ArmCartesianSetState(ARM_MOTION_HOLDING, now_ms);
            break;
    }
    ArmCartesianUpdateDebug();
    ArmCartesianUpdateControlDebug(now_ms, task_delta_ms);
}
