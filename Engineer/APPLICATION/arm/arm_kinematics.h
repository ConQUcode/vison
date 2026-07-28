#ifndef __ARM_KINEMATICS_H__
#define __ARM_KINEMATICS_H__

#include "arm.h"

#define ARM_BASE_HEIGHT_MM               34.0f
#define ARM_LINK_1_MM                   150.0f
#define ARM_LINK_2_MM                   179.0f
#define ARM_SHOULDER_OFFSET_FORWARD_MM  (-29.0f)
#define ARM_SHOULDER_OFFSET_LEFT_MM      (-7.6f)

#define ARM_AUTO_Q1_MIN_DEG             (-50.0f)
#define ARM_AUTO_Q1_MAX_DEG               50.0f
#define ARM_AUTO_Q2_MIN_DEG                5.0f
#define ARM_AUTO_Q2_MAX_DEG              175.0f
#define ARM_AUTO_Q3_MIN_DEG             (-175.0f)
#define ARM_AUTO_Q3_MAX_DEG              (-90.0f)

uint8_t ArmKinematicsSelfTest(float *error_mm);
uint8_t ArmJointPoseWithinSoftLimits(const float q_deg[3]);
uint8_t ArmAutoPoseIsSafe(const float q_deg[3]);

#endif
