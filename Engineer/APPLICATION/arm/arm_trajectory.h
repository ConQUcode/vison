/**
 * @file arm_trajectory.h
 * @brief 关节、笛卡尔直线和实时目标的非阻塞轨迹接口。
 */

#ifndef __ARM_TRAJECTORY_H__
#define __ARM_TRAJECTORY_H__

#include "arm.h"

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
