#ifndef __SHOOT_H_
#define __SHOOT_H_
#include "nac.h"
void ShootInit(void);
void ShootTask(void);
void ShootTask2(void);
void Shoot_task(void);
void Shoot_speed(void);
void Shoot_angle(void);
void ShootangleInit(void);
void Shoot_Stop(void);
extern Nac_Recv_s	*nac_ctrl;
#endif // DRIBBLE_H
