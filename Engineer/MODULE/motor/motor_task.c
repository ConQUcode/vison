/**
 * @file motor_task.c
 * @brief DJI 电机统一周期控制入口；新应用优先通过 app_runtime 调度。
 */

#include "motor_task.h"
#include "dji_motor.h"

void MotorControlTask(void)
{
    // static uint8_t cnt = 0; 设定不同电机的任务频率
    // if(cnt%5==0) //200hz
    // if(cnt%10==0) //100hz
    DJIMotorControl();

}
