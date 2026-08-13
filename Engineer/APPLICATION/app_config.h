/**
 * @file app_config.h
 * @brief 选择整机运行模式。一次只能启用一种模式，修改后需要重新编译固件。
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 完整机械臂：三台达妙、两台幻儿舵机和水果识别观察链。 */
#define APP_MODE_ARM                    0u
/* 底盘台架：BMI088、双 M3508、里程计以及1m/右转90度循环动作。 */
#define APP_MODE_CHASSIS_ONE_METER      1u
/* 舵机台架：只轮询 ID1/ID2 反馈，不发送机械臂动作。 */
#define APP_MODE_HUANER_FEEDBACK        2u
/* 机械臂打点：三台达妙保持失能，双舵机只读反馈，Watch显示角度和坐标。 */
#define APP_MODE_ARM_TEACH_POINT         3u

/* 当前固件运行完整机械臂模式；上电HOME后执行抓放循环测试。 */
#define APP_MODE APP_MODE_ARM

/*
 * 夹爪中心抓放循环：HOME后先单独将ID1调到抓取俯仰(见下方PITCH宏)，
 * 然后移动到抓取点。抓取后进入固定姿态：底座朝前、大臂竖直、小臂
 * 水平；随后底座逆时针转180deg到后方，大臂向实际前方倾60deg、小臂
 * 向下45deg后释放。释放后依次恢复小臂、大臂和底座，再继续下一轮。
 * 每次提交使用递增command_id，避免循环命令被机械臂邮箱判为重复命令。
 */
#define APP_ARM_TOOL_CENTER_TEST_ENABLE              1u
#define APP_ARM_PICK_PLACE_COMMAND_ID_BASE    0xA11B0000u
/* 抓取循环开始前，底盘只向前移动一次585mm；停车误差窗口为3mm。 */
#define APP_ARM_PRE_PICK_CHASSIS_DISTANCE_M          0.585f
#define APP_ARM_PRE_PICK_CHASSIS_TOLERANCE_M         0.003f
/* 前两次完成放置后前进500mm；总共执行点1、点2、点1三次抓放。 */
#define APP_ARM_BETWEEN_PICK_CHASSIS_DISTANCE_M      0.500f
#define APP_ARM_TEST_PICK_COUNT                         3u
/*
 * 两个采摘打点的完整教导位姿（2026-08-13无力打点实测+耦合修正）。
 * 点1在底座正左(q1约+90deg)，点2在正右(q1约-90deg)。
 * 当时打点固件未激活小臂同步带补偿，q3显示为电机原始角，已按
 * q3 = 原始角 + (q2 - 180) 修正：点1 38.60->-112.88，点2 41.81->-107.53。
 * q1使用准确的+/-90deg；抓取前的低位底座对准仍限幅在+/-89.5deg，
 * 最后由高位联合抓取命令补到+/-90deg，避免低位跨区拒绝。
 * 教导ID1相对小臂约-41.5deg，修正后绝对俯仰约-80deg（接近垂直向下）。
 * 2026-08-13：夹爪闭合与地面干涉，q2/q3按平面几何重解；两次各
 * 抬高10mm后再降低5mm、提高3mm，当前工具中心保持原教导径向并
 * 净抬高18mm。
 * 工具中心坐标为修正后FK换算值，仅用于Watch误差显示，不参与规划。
 */
#define APP_ARM_PICK_POINT_1_Q1_DEG                    90.0f
#define APP_ARM_PICK_POINT_1_Q2_DEG                    31.64f
#define APP_ARM_PICK_POINT_1_Q3_DEG                 (-111.21f)
#define APP_ARM_PICK_POINT_1_X_MM                      0.0f
#define APP_ARM_PICK_POINT_1_Y_MM                   (-451.6f)
#define APP_ARM_PICK_POINT_1_Z_MM                    (-73.3f)
#define APP_ARM_PICK_POINT_2_Q1_DEG                  (-90.0f)
#define APP_ARM_PICK_POINT_2_Q2_DEG                    33.88f
#define APP_ARM_PICK_POINT_2_Q3_DEG                 (-105.85f)
#define APP_ARM_PICK_POINT_2_X_MM                      0.0f
#define APP_ARM_PICK_POINT_2_Y_MM                    431.0f
#define APP_ARM_PICK_POINT_2_Z_MM                    (-76.9f)
/* 教导抓取姿态的ID1相对小臂角；两个打点实测均约-41.5deg。 */
#define APP_ARM_PICK_TOOL_RELATIVE_PITCH_DEG          (-41.5f)
/*
 * 底座对准限幅：对准旋转在工具中心z约203mm的低位进行，若q1到达
 * +/-90deg会让工具中心跨过X=0触发210mm跨区高度拒绝；限制在89.5deg
 * 内让工具中心保持X>0，剩余角度由随后的教导位姿联合命令补齐
 * （该段跨X=0时工具中心已接近教导高度，高于210mm跨区线）。
 */
#define APP_ARM_PICK_BASE_AIM_MAX_ABS_Q1_DEG            89.5f
/* 固定放置姿态完成后，释放前让ID1转到相对当前小臂-45deg。 */
#define APP_ARM_RELEASE_TOOL_RELATIVE_PITCH_DEG        (-45.0f)
#define APP_ARM_TOOL_CENTER_TEST_SPEED_MM_S           100.0f
/*
 * 左右抓取后的安全点：底座保持对应抓取侧，大臂竖直，小臂上抬10deg。
 * 点1位于左侧，点2位于右侧；底座角与各自抓取点相同，不先回HOME。
 */
#define APP_ARM_LEFT_SAFE_BASE_Q1_DEG                    90.0f
#define APP_ARM_RIGHT_SAFE_BASE_Q1_DEG                 (-90.0f)
#define APP_ARM_TRANSFER_SHOULDER_Q2_DEG                90.0f
/* 释放大臂角。原150deg会干涉（2026-08-13实测），收到120deg。 */
#define APP_ARM_RELEASE_SHOULDER_Q2_DEG                 120.0f
/* q2=90deg时q3=-100deg，小臂绝对俯仰为+10deg，与释放姿态一致。 */
#define APP_ARM_TRANSFER_ELBOW_Q3_DEG                 (-100.0f)
/* 释放时小臂由-60deg继续上抬10deg到-70deg，增加后区净空。 */
#define APP_ARM_RELEASE_ELBOW_DOWN_Q3_DEG               (-70.0f)
/*
 * 左侧沿正角度（逆时针）到+180deg，右侧沿负角度（顺时针）到
 * -180deg；+/-180deg物理上是同一个后方放置方向。
 */
#define APP_ARM_LEFT_PLACE_BASE_MID_Q1_DEG              135.0f
#define APP_ARM_RIGHT_PLACE_BASE_MID_Q1_DEG           (-135.0f)
#define APP_ARM_LEFT_PLACE_BASE_Q1_DEG                  180.0f
#define APP_ARM_RIGHT_PLACE_BASE_Q1_DEG               (-180.0f)
#define APP_ARM_PICK_BASE_Q1_DEG                         0.0f
#define APP_ARM_PICK_DWELL_MS                         1000u
#define APP_ARM_POST_GRIP_DWELL_MS                    1000u
/* 联合放置动作结束后只等待ID1实际到位；超时判故障，不作为固定延时。 */
#define APP_ARM_RELEASE_PITCH_WAIT_TIMEOUT_MS         1500u
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
