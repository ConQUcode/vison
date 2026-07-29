#ifndef __ARM_KINEMATICS_H__
#define __ARM_KINEMATICS_H__

#include "arm.h"
#include "arm_config.h"

uint8_t ArmKinematicsSelfTest(float *error_mm);
uint8_t ArmJointPoseWithinSoftLimits(const float q_deg[3]);
uint8_t ArmAutoPoseIsSafe(const float q_deg[3]);

#endif
