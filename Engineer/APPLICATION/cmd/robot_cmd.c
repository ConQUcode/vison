#include "robot_cmd.h"
#include "dmmotor.h"
#include "DJI_motor.h"
#include "bsp_dwt.h"
#include "chassis.h"
#include "ins_task.h"




void RobotCMDInit(void)
{
	//开启高精度计数计时器
	DWT_Init(168);
	
	//开启遥控器接收

	chassis_ctrl_cmd.Chassis_IMU_data                 = INS_Init(); // 底盘IMU初始化

	 	
}

	

