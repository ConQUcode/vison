/**
 * @file arm_kinematics.h
 * @brief 三自由度机械臂正逆解、自检和关节限位检查接口。
 */

#ifndef __ARM_KINEMATICS_H__
#define __ARM_KINEMATICS_H__

#include "arm.h"
#include "arm_config.h"

/** 执行固定姿态 FK/IK 往返测试，error_mm 返回最大位置误差。 */
uint8_t ArmKinematicsSelfTest(float *error_mm);
/** 检查三关节角是否位于正常软件限位内。 */
uint8_t ArmJointPoseWithinSoftLimits(const float q_deg[3]);
/** 检查姿态是否允许自动轨迹使用，不包含启动脱困边界。 */
uint8_t ArmAutoPoseIsSafe(const float q_deg[3]);
uint8_t ArmAutoPoseIsSafeWithQ1Limits(const float q_deg[3],
                                      float q1_min_deg,
                                      float q1_max_deg);

/** 组合主臂FK和117mm工具偏移，得到夹爪中心；q1正角朝世界Y正侧。 */
uint8_t ArmForwardKinematicsToolCenter(
    const float q_deg[3], float tool_pitch_deg,
    Arm_Position_s *tool_center_mm);
/**
 * 夹爪中心逆解。检查目标方位及反向方位，最终按seed连续性选择，
 * 并以夹爪中心FK往返误差验证结果。
 */
Arm_IK_Status_e ArmInverseKinematicsToolCenter(
    const Arm_Position_s *target_center_mm,
    float tool_pitch_deg,
    const float seed_q_deg[3],
    Arm_Tool_Center_IK_Result_s *result);

/**
 * 返回夹爪中心目标的全部合法关节候选，不提前按seed丢弃其他分支。
 * seed只用于工具中心位于底座轴线时提供方位参考；返回候选顺序固定。
 */
Arm_IK_Status_e ArmInverseKinematicsToolCenterAll(
    const Arm_Position_s *target_center_mm,
    float tool_pitch_deg,
    const float seed_q_deg[3],
    Arm_Tool_Center_IK_Candidate_s candidates[
        ARM_TOOL_CENTER_IK_MAX_CANDIDATES],
    uint8_t *candidate_count);
Arm_IK_Status_e ArmInverseKinematicsToolCenterAllWithQ1Limits(
    const Arm_Position_s *target_center_mm,
    float tool_pitch_deg,
    const float seed_q_deg[3],
    float q1_min_deg,
    float q1_max_deg,
    Arm_Tool_Center_IK_Candidate_s candidates[
        ARM_TOOL_CENTER_IK_MAX_CANDIDATES],
    uint8_t *candidate_count);

Arm_IK_Status_e ArmInverseKinematicsToolCenterWithQ1Limits(
    const Arm_Position_s *target_center_mm,
    float tool_pitch_deg,
    const float seed_q_deg[3],
    float q1_min_deg,
    float q1_max_deg,
    Arm_Tool_Center_IK_Result_s *result);

#endif
