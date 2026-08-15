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
/* 机械臂专项：正常HOME后在原地持续执行A左/A右镜像抓放，不运行底盘。 */
#define APP_MODE_ARM_POSTURE_TEST        4u
/* MG995台架：只启动TIM8双路PWM并保持左右摄像头水平0deg。 */
#define APP_MODE_MG995_TEST              5u

/* 当前固件仅测试左右摄像头水平0deg，不初始化机械臂、底盘或IMU。 */
#define APP_MODE APP_MODE_MG995_TEST

/* 所有固件内部机械臂调用者共享同一递增序列，禁止再划分模块私有区间。 */
#define APP_ARM_COMMAND_ID_SEED                0xA1100000u
/* 左侧为基准；右侧只对Y取负，X/Z、俯仰、速度和流程完全一致。 */
#define APP_ARM_POSTURE_TEST_X_MM                      0.0f
#define APP_ARM_POSTURE_TEST_Z_MM                   (-150.0f)
#define APP_ARM_POSTURE_TEST_LEFT_Y_MM               340.0f
#define APP_ARM_POSTURE_TEST_RIGHT_Y_MM \
    (-APP_ARM_POSTURE_TEST_LEFT_Y_MM)
/* 接近点到位后保持X/Z和俯仰不变，沿当前侧Y方向直线推进140mm。 */
#define APP_ARM_POSTURE_TEST_ADVANCE_X_MM              0.0f
#define APP_ARM_POSTURE_TEST_ADVANCE_Z_MM           (-150.0f)
#define APP_ARM_POSTURE_TEST_LEFT_ADVANCE_Y_MM        480.0f
#define APP_ARM_POSTURE_TEST_RIGHT_ADVANCE_Y_MM \
    (-APP_ARM_POSTURE_TEST_LEFT_ADVANCE_Y_MM)
#define APP_ARM_POSTURE_TEST_TOOL_PITCH_DEG           (-5.0f)
#define APP_ARM_POSTURE_TEST_SPEED_MM_S              100.0f

/*
 * A区任务顺序、点位、安全点和放置路线集中在app_fruit_task_config.h；
 * app_config.h只保留跨区域通用的机械臂子流程参数。
 */
#define APP_ARM_TOOL_CENTER_TEST_ENABLE              1u
/*
 * 底座对准限幅：关节预对准若直接到+/-90deg，会让工具中心落在X=0
 * 跨区边界；限制在89.5deg内保持X>0，随后由工具中心轨迹完成剩余
 * 约0.5deg和伸臂动作。
 */
#define APP_ARM_PICK_BASE_AIM_MAX_ABS_Q1_DEG            89.5f
/*
 * 下一次抓取预对准时同步进入俯仰可达姿态。该姿态小臂绝对俯仰为-10deg，
 * 后续夹爪绝对俯仰-90deg所需的ID1相对角为-80deg，给关节到位误差留出
 * 俯仰限位余量，不能把准备姿态配置在-90deg相对角的精确边界上。
 */
#define APP_ARM_PICK_STAGING_Q2_DEG                      80.0f
#define APP_ARM_PICK_STAGING_Q3_DEG                    (-90.0f)
/* 准备阶段ID1与DM三轴同时运动；该相对角对应绝对俯仰-90deg。 */
#define APP_ARM_PICK_STAGING_TOOL_RELATIVE_PITCH_DEG   (-80.0f)
/* 仅提高水果抓取工具中心轨迹速度，不修改底层全局关节安全限速。 */
#define APP_ARM_TOOL_CENTER_TEST_SPEED_MM_S           200.0f
#define APP_ARM_PICK_DWELL_MS                         1000u
#define APP_ARM_POST_GRIP_DWELL_MS                    1000u
/* 俯仰反馈误差不超过2deg并连续稳定200ms后，才允许闭合夹爪。 */
#define APP_ARM_TOOL_CENTER_PITCH_TOLERANCE_DEG         2.0f
#define APP_ARM_TOOL_CENTER_PITCH_STABLE_MS            200u

#if APP_MODE != APP_MODE_ARM && \
    APP_MODE != APP_MODE_CHASSIS_ONE_METER && \
    APP_MODE != APP_MODE_HUANER_FEEDBACK && \
    APP_MODE != APP_MODE_ARM_TEACH_POINT && \
    APP_MODE != APP_MODE_ARM_POSTURE_TEST && \
    APP_MODE != APP_MODE_MG995_TEST
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
#define APP_ARM_POSTURE_TEST_ENABLED \
    ((APP_MODE) == APP_MODE_ARM_POSTURE_TEST)
#define APP_MG995_TEST_ENABLED \
    ((APP_MODE) == APP_MODE_MG995_TEST)
#define APP_ARM_CORE_ENABLED \
    (APP_ARM_ENABLED || APP_ARM_TEACH_POINT_ENABLED || \
     APP_ARM_POSTURE_TEST_ENABLED)

#endif
