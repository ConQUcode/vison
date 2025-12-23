/**
 * @file robot_def.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief   机器人定义,包含机器人的各种参数
 * @version 0.1
 * @date 2024-01-09
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __ROBOT_DEF_H__
#define __ROBOT_DEF_H__

#include "stdint.h"
#include "ins_task.h"

#define RAD_2_DEGREE        57.2957795f    // 180/pi

// 机器人几何尺寸定义
// 长410mm, 宽320mm. 假设长为前后方向(轴距), 宽为左右方向(轮距)
#define CHASSIS_WHEEL_BASE      0.410f      // 轴距(前后轮距离, X轴方向)
#define CHASSIS_WHEEL_TRACK     0.320f      // 轮距(左右轮距离, Y轴方向)
#define CHASSIS_HALF_BASE       (CHASSIS_WHEEL_BASE / 2.0f)
#define CHASSIS_HALF_TRACK      (CHASSIS_WHEEL_TRACK / 2.0f)

// 电机速度转换相关宏定义 
#define MOTOR_REDUCTION_RATIO    19.2032f     // 电机减速比 1:19.2302
#define WHEEL_RADIUS_M           0.081f     // 轮子半径(米) - 根据实际轮子尺寸调整

// 线速度转换为3508电机速度环参数的转换系数
// 转换公式: 电机RPM = 线速度(m/s) * 转换系数
// 推导过程:
// 1. 轮子转速(rpm) = 线速度(m/s) * 60 / (2 * π * 轮子半径)
// 2. 电机转速(rpm) = 轮子转速(rpm) * 减速比
// 3. 转换系数 = 60 * 减速比 / (2 * π * 轮子半径)
#define LINEAR_VELOCITY_TO_MOTOR_RPM    (60.0f * MOTOR_REDUCTION_RATIO / (2.0f * 3.14159f * WHEEL_RADIUS_M)) // 约等于 8732.24

// 反向转换：从电机速度转换为线速度 (用于速度反馈显示等)
#define MOTOR_SPEED_TO_LINEAR_VEL(motor_speed)   ((motor_speed) / LINEAR_VELOCITY_TO_MOTOR_RPM)

// DJ6020电机角速度转换相关宏定义
#define GM6020_REDUCTION_RATIO    1.0f        // GM6020通常为直驱，减速比1:1
#define GM6020_ECD_RANGE          8192.0f     // 编码器范围 0-8191

// 角速度(rad/s)转换为GM6020电机RPM
// 转换公式: 电机RPM = 角速度(rad/s) * 60 / (2 * π) * 减速比
#define ANGULAR_VELOCITY_TO_GM6020_RPM    (60.0f * GM6020_REDUCTION_RATIO / (2.0f * 3.14159f))

// 反向转换：从GM6020电机RPM转换为角速度(rad/s)
#define GM6020_RPM_TO_ANGULAR_VEL(rpm)    ((rpm) / ANGULAR_VELOCITY_TO_GM6020_RPM)


#define STEERING_CHASSIS_ALIGN_ECD_LF   5800// 舵电机 A 4编码器值，若有机械改动需要修改7848-
#define STEERING_CHASSIS_ALIGN_ECD_LB   5610 // 舵电机 B 2编码器值，若有机械改动需要修改3562+
#define STEERING_CHASSIS_ALIGN_ECD_RF   1734// 舵电机 C 1编码器值，若有机械改动需要修改7878+
#define STEERING_CHASSIS_ALIGN_ECD_RB   4389 // 舵电机 D 3编码器值，若有机械改动需要修改6437-

#define STEERING_CHASSIS_ALIGN_ANGLE_LF STEERING_CHASSIS_ALIGN_ECD_LF / 8192.f * 360.f // 舵轮 A 对齐角度
#define STEERING_CHASSIS_ALIGN_ANGLE_LB STEERING_CHASSIS_ALIGN_ECD_LB / 8192.f * 360.f // 舵轮 B 对齐角度
#define STEERING_CHASSIS_ALIGN_ANGLE_RF STEERING_CHASSIS_ALIGN_ECD_RF / 8192.f * 360.f // 舵轮 C 对齐角度
#define STEERING_CHASSIS_ALIGN_ANGLE_RB STEERING_CHASSIS_ALIGN_ECD_RB / 8192.f * 360.f // 舵轮 D 对齐角度

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输


/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef enum {
    IMU_CORRECT_NONE = 0,      // 不校准
    IMU_CORRECT_STRAIGHT,      // 直线校准（当前实现）
    IMU_CORRECT_ROTATION,      // 转弯校准（新增）
    IMU_CORRECT_HYBRID         // 混合校准（高级）
} IMU_Correct_Mode_e;

typedef struct
{
    float vx;
    float vy;
    float vw;
    float last_yaw;
    float offset_w;
    
    float target_yaw;               // 目标角度（用于转弯控制）
    IMU_Correct_Mode_e correct_mode; // 校准模式
    uint8_t imu_enable;             // IMU使能标志
    
    float real_vx;
    float real_vy;
    float real_wz;
    attitude_t *Chassis_IMU_data;
} Chassis_Ctrl_Cmd_s;
#pragma pack() // 取消压缩
#endif

