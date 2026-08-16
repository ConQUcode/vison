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
/* AC区机械臂专项：正常HOME后持续执行左/右镜像抓放，不运行底盘。 */
#define APP_MODE_ARM_POSTURE_TEST        4u
/* MG995台架：只启动TIM8双路PWM并保持左右摄像头水平0deg。 */
#define APP_MODE_MG995_TEST              5u
/* 上位机控制：USB协议统一控制底盘速度、夹爪和双摄像头舵机。 */
#define APP_MODE_HOST_CONTROL            6u
/* BD区观察位：正常HOME后单次移动到右侧树上水果观察点并保持。 */
#define APP_MODE_ARM_BD_OBSERVATION_TEST 7u

/* 当前固件用于BD右观察点测试：HOME后主臂与底座协同转向，再进入目标。 */
#ifndef APP_MODE
#define APP_MODE APP_MODE_ARM_BD_OBSERVATION_TEST
#endif

/* 所有固件内部机械臂调用者共享同一递增序列，禁止再划分模块私有区间。 */
#define APP_ARM_COMMAND_ID_SEED                0xA1100000u
/*
 * 以下坐标、俯仰和分段速度均属于AC区（A/C两区）专项抓取流程。
 * 左侧为基准，右侧只对Y取负；后续BD区测试必须新增独立参数和profile，
 * 禁止直接覆盖或复用本组已验证参数。
 */
#define APP_ARM_POSTURE_TEST_X_MM                      0.0f
#define APP_ARM_POSTURE_TEST_Z_MM                   (-150.0f)
#define APP_ARM_POSTURE_TEST_LEFT_Y_MM               380.0f
#define APP_ARM_POSTURE_TEST_RIGHT_Y_MM \
    (-APP_ARM_POSTURE_TEST_LEFT_Y_MM)
/* 接近点到位后保持X/Z和俯仰不变，沿当前侧Y方向直线推进100mm。 */
#define APP_ARM_POSTURE_TEST_ADVANCE_X_MM              0.0f
#define APP_ARM_POSTURE_TEST_ADVANCE_Z_MM           (-150.0f)
#define APP_ARM_POSTURE_TEST_LEFT_ADVANCE_Y_MM        480.0f
#define APP_ARM_POSTURE_TEST_RIGHT_ADVANCE_Y_MM \
    (-APP_ARM_POSTURE_TEST_LEFT_ADVANCE_Y_MM)
#define APP_ARM_POSTURE_TEST_TOOL_PITCH_DEG           (-5.0f)
/* 先快速到接近点，再低速推进最后100mm，避免夹爪把水果推离抓取位。 */
#define APP_ARM_POSTURE_TEST_APPROACH_SPEED_MM_S      600.0f
#define APP_ARM_POSTURE_TEST_GRIP_ADVANCE_SPEED_MM_S 150.0f

/*
 * BD区左右树上水果观察点，坐标均为夹爪中心mm，不能当作后续抓取点。
 * +Y/q1=+90deg固定表示物理左侧，-Y/q1=-90deg固定表示物理右侧。
 * 两侧共享q2/q3、绝对俯仰和速度；右侧点由左侧严格镜像，禁止分别调漂。
 * 当前上传固件选择RIGHT。后续上位机视觉选边时只切ACTIVE_SIDE对应入口，
 * 不得覆盖这两组已验证点，也不得复用AC区抓取或放置参数。
 */
#define APP_ARM_BD_OBSERVATION_SIDE_LEFT                  1u
#define APP_ARM_BD_OBSERVATION_SIDE_RIGHT                 2u
#define APP_ARM_BD_OBSERVATION_ACTIVE_SIDE \
    APP_ARM_BD_OBSERVATION_SIDE_RIGHT

/* 左观察点：工具中心[0,+57,210]mm，底座朝物理左侧。 */
#define APP_ARM_BD_OBSERVATION_LEFT_BASE_Q1_DEG         90.0f
#define APP_ARM_BD_OBSERVATION_LEFT_X_MM                 0.0f
#define APP_ARM_BD_OBSERVATION_LEFT_Y_MM                57.0f
#define APP_ARM_BD_OBSERVATION_LEFT_Z_MM               210.0f

/* 右观察点：工具中心[0,-57,210]mm，底座朝物理右侧。 */
#define APP_ARM_BD_OBSERVATION_RIGHT_BASE_Q1_DEG \
    (-APP_ARM_BD_OBSERVATION_LEFT_BASE_Q1_DEG)
#define APP_ARM_BD_OBSERVATION_RIGHT_X_MM \
    APP_ARM_BD_OBSERVATION_LEFT_X_MM
#define APP_ARM_BD_OBSERVATION_RIGHT_Y_MM \
    (-APP_ARM_BD_OBSERVATION_LEFT_Y_MM)
#define APP_ARM_BD_OBSERVATION_RIGHT_Z_MM \
    APP_ARM_BD_OBSERVATION_LEFT_Z_MM

/* 两侧共同的同步收拢姿态、路径限制、最终俯仰和请求速度。 */
#define APP_ARM_BD_OBSERVATION_STAGING_Q2_DEG           120.0f
#define APP_ARM_BD_OBSERVATION_STAGING_Q3_DEG          (-48.0f)
#define APP_ARM_BD_OBSERVATION_PATH_Y_MAX_MM             260.0f
#define APP_ARM_BD_OBSERVATION_TOOL_PITCH_DEG         (-30.0f)
#define APP_ARM_BD_OBSERVATION_SPEED_MM_S             150.0f

/* 状态机只读取以下活动点别名；切侧时只修改ACTIVE_SIDE。 */
#if APP_ARM_BD_OBSERVATION_ACTIVE_SIDE == \
        APP_ARM_BD_OBSERVATION_SIDE_LEFT
#define APP_ARM_BD_OBSERVATION_BASE_Q1_DEG \
    APP_ARM_BD_OBSERVATION_LEFT_BASE_Q1_DEG
#define APP_ARM_BD_OBSERVATION_X_MM APP_ARM_BD_OBSERVATION_LEFT_X_MM
#define APP_ARM_BD_OBSERVATION_Y_MM APP_ARM_BD_OBSERVATION_LEFT_Y_MM
#define APP_ARM_BD_OBSERVATION_Z_MM APP_ARM_BD_OBSERVATION_LEFT_Z_MM
#elif APP_ARM_BD_OBSERVATION_ACTIVE_SIDE == \
          APP_ARM_BD_OBSERVATION_SIDE_RIGHT
#define APP_ARM_BD_OBSERVATION_BASE_Q1_DEG \
    APP_ARM_BD_OBSERVATION_RIGHT_BASE_Q1_DEG
#define APP_ARM_BD_OBSERVATION_X_MM APP_ARM_BD_OBSERVATION_RIGHT_X_MM
#define APP_ARM_BD_OBSERVATION_Y_MM APP_ARM_BD_OBSERVATION_RIGHT_Y_MM
#define APP_ARM_BD_OBSERVATION_Z_MM APP_ARM_BD_OBSERVATION_RIGHT_Z_MM
#else
#error "APP_ARM_BD_OBSERVATION_ACTIVE_SIDE is invalid"
#endif

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
#define APP_ARM_PICK_DWELL_MS                          100u
#define APP_ARM_POST_GRIP_DWELL_MS                     500u
/* 俯仰反馈误差不超过2deg并连续稳定200ms后，才允许闭合夹爪。 */
#define APP_ARM_TOOL_CENTER_PITCH_TOLERANCE_DEG         2.0f
#define APP_ARM_TOOL_CENTER_PITCH_STABLE_MS            200u

#if APP_MODE != APP_MODE_ARM && \
    APP_MODE != APP_MODE_CHASSIS_ONE_METER && \
    APP_MODE != APP_MODE_HUANER_FEEDBACK && \
    APP_MODE != APP_MODE_ARM_TEACH_POINT && \
    APP_MODE != APP_MODE_ARM_POSTURE_TEST && \
    APP_MODE != APP_MODE_MG995_TEST && \
    APP_MODE != APP_MODE_HOST_CONTROL && \
    APP_MODE != APP_MODE_ARM_BD_OBSERVATION_TEST
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
#define APP_HOST_CONTROL_ENABLED \
    ((APP_MODE) == APP_MODE_HOST_CONTROL)
#define APP_ARM_BD_OBSERVATION_TEST_ENABLED \
    ((APP_MODE) == APP_MODE_ARM_BD_OBSERVATION_TEST)
#define APP_CHASSIS_ENABLED \
    (APP_CHASSIS_ONE_METER_ENABLED || APP_ARM_ENABLED || \
     APP_HOST_CONTROL_ENABLED)
#define APP_USB_ENABLED \
    (APP_CHASSIS_ONE_METER_ENABLED || APP_ARM_ENABLED || \
     APP_HOST_CONTROL_ENABLED)
#define APP_MG995_ENABLED \
    (APP_MG995_TEST_ENABLED || APP_HOST_CONTROL_ENABLED)
#define APP_ARM_CORE_ENABLED \
    (APP_ARM_ENABLED || APP_ARM_TEACH_POINT_ENABLED || \
     APP_ARM_POSTURE_TEST_ENABLED || APP_HOST_CONTROL_ENABLED || \
     APP_ARM_BD_OBSERVATION_TEST_ENABLED)

#endif
