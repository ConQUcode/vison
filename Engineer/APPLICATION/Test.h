#ifndef __TEST_H
#define __TEST_H

#include <stdint.h>

/*
 * 应用运行模式：各专项测试必须互斥。
 * 当前恢复完整机械臂：三台达妙、USART6双舵机和水果观察桥正常运行；
 * 底盘与INS/IMU仍保持禁用。
 */
#define CHASSIS_ONE_METER_TEST_ONLY    0u
/* 仅用于排查舵机通信；置1时不会初始化机械臂或发送舵机动作。 */
#define HUANER_SERVO_DUAL_FEEDBACK_TEST_ONLY 0u

#if CHASSIS_ONE_METER_TEST_ONLY != 0u && \
    HUANER_SERVO_DUAL_FEEDBACK_TEST_ONLY != 0u
#error "Chassis test and Huaner servo feedback test cannot run together"
#endif

/* 两个台架模式都关闭时，运行完整机械臂和水果识别观察链。 */
#define APPLICATION_ARM_RUN_ENABLE \
    ((CHASSIS_ONE_METER_TEST_ONLY == 0u) && \
     (HUANER_SERVO_DUAL_FEEDBACK_TEST_ONLY == 0u))

typedef struct {
    uint8_t driver_initialized;       /* USART6舵机驱动注册成功。 */
    uint8_t servo1_communication_ok;  /* ID1位置反馈在100ms内有效。 */
    uint8_t servo2_communication_ok;  /* ID2位置反馈在100ms内有效。 */
    uint16_t servo1_position;         /* ID1当前控制值，范围0~1000。 */
    uint16_t servo2_position;         /* ID2当前控制值，范围0~1000。 */
    float servo1_angle_deg;           /* ID1当前轴角，范围0~240deg。 */
    float servo2_angle_deg;           /* ID2当前轴角，范围0~240deg。 */
} Huaner_Dual_Servo_Debug_s;

extern Huaner_Dual_Servo_Debug_s g_huaner_dual_servo_debug;

/* 调度器启动前调用一次，根据上面的模式宏注册对应设备。 */
void all_init_Task(void);
/* 由1 kHz综合控制任务调用，内部只运行当前选中的控制链。 */
void all_cmd_Task(void);

#endif
