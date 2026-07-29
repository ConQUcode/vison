#ifndef __ARM_INTERNAL_H__
#define __ARM_INTERNAL_H__

#include "arm.h"

/*
 * 机械臂模块内部接口，仅供arm.c和arm_trajectory.c使用。
 * 应用命令层不得直接调用这些函数，否则会绕过模式、忙状态和路径预检。
 */
uint8_t ArmBeginJointMove(const float target_q_deg[3]);
uint8_t ArmUpdateJointReference(const float reference_q_deg[3]);
uint8_t ArmSetJointTargetDeg(float q1_deg, float q2_deg, float q3_deg);
void ArmUpdateControllerDebugSnapshot(Arm_Control_Debug_s *debug);
void ArmMotionStopMotors(void);
void ArmAbortMotion(Arm_Motion_Fault_e reason);
Arm_Motion_Result_e ArmSetCartesianTarget(const Arm_Position_s *target,
                                          Arm_IK_Result_s *result);
Arm_Motion_Result_e ArmMoveLinear(const Arm_Position_s *target,
                                  float max_speed_mm_s);

#endif
