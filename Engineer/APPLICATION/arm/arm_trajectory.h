/**
 * @file arm_trajectory.h
 * @brief 关节、笛卡尔直线和实时目标的非阻塞轨迹接口。
 */

#ifndef __ARM_TRAJECTORY_H__
#define __ARM_TRAJECTORY_H__

#include "arm.h"
#include "arm_path_planner.h"

/** 初始化轨迹缓存和所有权状态。 */
void ArmTrajectoryInit(void);
/** 1 kHz 推进当前轨迹、到位稳定窗口和实时目标超时。 */
void ArmTrajectoryTask(uint32_t now_ms);
uint8_t ArmTrajectoryMotorHoldAllowed(void);
uint8_t ArmTrajectoryOwnsControl(void);
uint8_t ArmTrajectoryIsBusy(void);
uint8_t ArmTrajectoryRealtimeActive(void);
/** 取消轨迹并释放控制权，不负责解除机械臂硬故障。 */
void ArmTrajectoryCancel(void);
/**
 * 使用正式轨迹的CCM工作区执行只读工具中心单段预检，不启动运动。
 * 仅允许在轨迹空闲时调用；结果和正式工具中心轨迹使用同一规划内核。
 */
uint8_t ArmTrajectoryPreflightToolCenterSegment(
    const Arm_Path_Plan_Request_s *request,
    Arm_Path_Plan_Result_s *result);
uint8_t ArmTrajectorySelectReachableToolCenterAdvance(
    const Arm_Path_Advance_Request_s *request,
    Arm_Path_Advance_Result_s *result);
Arm_Motion_Result_e ArmTrajectoryMoveJoint(const float target_q_deg[3]);
Arm_Motion_Result_e ArmTrajectoryMoveJointWithRelativeToolPitch(
    const float target_q_deg[3],
    uint8_t relative_pitch_valid,
    float relative_pitch_deg);
Arm_Motion_Result_e ArmTrajectoryMoveJointWithOptions(
    const float target_q_deg[3],
    uint8_t waypoint_valid,
    const float waypoint_q_deg[3],
    uint8_t relative_pitch_valid,
    float relative_pitch_deg);
Arm_Motion_Result_e ArmTrajectoryMoveJointThenLinear(
    const float waypoint_q_deg[3],
    const Arm_Position_s *target,
    float max_speed_mm_s);
Arm_Motion_Result_e ArmTrajectorySetJointDirect(const float target_q_deg[3]);
Arm_Motion_Result_e ArmTrajectoryStageCartesianCommand(
    const Arm_Cartesian_Command_s *command);
Arm_Command_Result_e ArmTrajectorySubmitRealtimeTarget(
    const Arm_Realtime_Cartesian_Target_s *target);
void ArmTrajectoryStopRealtime(void);

#endif
