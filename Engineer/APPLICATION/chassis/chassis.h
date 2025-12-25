#ifndef __CHASSIS_H_
#define __CHASSIS_H_
#include "robot_cmd.h"
#include "robot_def.h"
#include "nac.h"
void ChassisInit(void);
void ChassisTask(void);
void Jiaozhun(void);
void SteeringWheelKinematics(float vx, float vy, float vw);
void SteeringWheelKinematics_old(float vx, float vy, float vw);
extern  float V;
extern  RC_ctrl_t *rc_cmd;
extern Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;
extern Nac_Recv_s	*nac_ctrl;
#endif 
