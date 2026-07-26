#include "DJI_motor.h"
#include "Test.h"
#include "arm.h"


void all_init_Task(void)
{
	ArmInit();
}

void all_cmd_Task(void)
{
	ArmTask();
	DJIMotorControl();
}
