#include "robot_cmd.h"
#include "bsp_dwt.h"


void RobotCMDInit(void)
{
	DWT_Init(168);
}
