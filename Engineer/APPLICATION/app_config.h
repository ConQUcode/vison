/**
 * @file app_config.h
 * @brief 选择整机运行模式。一次只能启用一种模式，修改后需要重新编译固件。
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 完整机械臂：三台达妙、两台幻儿舵机和水果识别观察链。 */
#define APP_MODE_ARM                    0u
/* 底盘台架：应用层通过通用底盘接口执行1m/右转90度循环动作。 */
#define APP_MODE_CHASSIS_ONE_METER      1u
/* 舵机台架：只轮询 ID1/ID2 反馈，不发送机械臂动作。 */
#define APP_MODE_HUANER_FEEDBACK        2u
/* 机械臂打点：三台达妙保持失能，双舵机只读反馈，Watch显示角度和坐标。 */
#define APP_MODE_ARM_TEACH_POINT         3u

/* 当前固件运行完整机械臂模式；上电HOME后执行A区三次抓放测试。 */
#define APP_MODE APP_MODE_ARM

/*
 * A区任务顺序、点位、安全点和放置路线集中在app_fruit_task_config.h；
 * app_config.h只保留跨区域通用的机械臂子流程参数。
 */
#define APP_ARM_TOOL_CENTER_TEST_ENABLE              1u
#define APP_ARM_PICK_PLACE_COMMAND_ID_BASE    0xA11B0000u
/*
 * 底座对准限幅：对准旋转在工具中心z约203mm的低位进行，若q1到达
 * +/-90deg会让工具中心跨过X=0触发210mm跨区高度拒绝；限制在89.5deg
 * 内让工具中心保持X>0，剩余角度由随后的教导位姿联合命令补齐
 * （该段跨X=0时工具中心已接近教导高度，高于210mm跨区线）。
 */
#define APP_ARM_PICK_BASE_AIM_MAX_ABS_Q1_DEG            89.5f
#define APP_ARM_TOOL_CENTER_TEST_SPEED_MM_S           100.0f
#define APP_ARM_PICK_DWELL_MS                         1000u
#define APP_ARM_POST_GRIP_DWELL_MS                    1000u
/* 俯仰反馈误差不超过2deg并连续稳定200ms后，才允许主臂开始运动。 */
#define APP_ARM_TOOL_CENTER_PITCH_TOLERANCE_DEG         2.0f
#define APP_ARM_TOOL_CENTER_PITCH_STABLE_MS            200u

#if APP_MODE != APP_MODE_ARM && \
    APP_MODE != APP_MODE_CHASSIS_ONE_METER && \
    APP_MODE != APP_MODE_HUANER_FEEDBACK && \
    APP_MODE != APP_MODE_ARM_TEACH_POINT
#error "APP_MODE is invalid"
#endif

#define APP_ARM_ENABLED \
    ((APP_MODE) == APP_MODE_ARM)
#define APP_CHASSIS_ONE_METER_ENABLED \
    ((APP_MODE) == APP_MODE_CHASSIS_ONE_METER)
#define APP_HUANER_FEEDBACK_ENABLED \
    ((APP_MODE) == APP_MODE_HUANER_FEEDBACK)
#define APP_ARM_TEACH_POINT_ENABLED \
    ((APP_MODE) == APP_MODE_ARM_TEACH_POINT)
#define APP_ARM_CORE_ENABLED \
    (APP_ARM_ENABLED || APP_ARM_TEACH_POINT_ENABLED)

#endif
