#ifndef __ARM_TRAJECTORY_H__
#define __ARM_TRAJECTORY_H__

#include "arm.h"

void ArmTrajectoryInit(void);
void ArmTrajectoryTask(uint32_t now_ms);
uint8_t ArmTrajectoryMotorHoldAllowed(void);
uint8_t ArmTrajectoryOwnsControl(void);

#endif
