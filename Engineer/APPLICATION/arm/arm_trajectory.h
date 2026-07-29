#ifndef __ARM_TRAJECTORY_H__
#define __ARM_TRAJECTORY_H__

#include "arm.h"

void ArmTrajectoryInit(void);
void ArmTrajectoryTask(uint32_t now_ms);
uint8_t ArmTrajectoryMotorHoldAllowed(void);
uint8_t ArmTrajectoryOwnsControl(void);
uint8_t ArmTrajectoryIsBusy(void);
void ArmTrajectoryCancel(void);
Arm_Motion_Result_e ArmTrajectoryMoveJoint(const float target_q_deg[3]);
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
