/**
 * @file app_fruit_task_config.h
 * @brief A 区实测点位、机构偏移和放置 profile 参数。
 */

#ifndef APP_FRUIT_TASK_CONFIG_H
#define APP_FRUIT_TASK_CONFIG_H

/* 临时原地联调：不提交底盘命令，业务点1/点2完整抓放并无限交替。 */
#define APP_FRUIT_TEST_IN_PLACE_ALTERNATING_ENABLE          1u

/* A区沿行进方向有4组水果，每组左右各1个；纵向组间距均为500 mm。 */
#define APP_FRUIT_AREA_A_FIRST_POSITION_MM              500.0f
/* 启动时机械臂中心在物理车头后方85 mm，仅首段需要补偿该安装偏移。 */
#define APP_FRUIT_ARM_CENTER_BEHIND_NOSE_MM               85.0f
#define APP_FRUIT_AREA_A_FIRST_MOVE_MM \
    (APP_FRUIT_AREA_A_FIRST_POSITION_MM + \
     APP_FRUIT_ARM_CENTER_BEHIND_NOSE_MM)
/* 到达第一组后，相邻水果组之间不再叠加85 mm安装偏移。 */
#define APP_FRUIT_AREA_A_GROUP_SPACING_MM                  500.0f
#define APP_FRUIT_CHASSIS_TOLERANCE_MM                       3.0f
#define APP_FRUIT_CHASSIS_COMMAND_ID_BASE             0xA12C0000u

/* 业务点1：物理左侧Y>0，实机q1=+90deg一侧。 */
#define APP_FRUIT_A_LEFT_X_MM                               0.0f
#define APP_FRUIT_A_LEFT_Y_MM                            400.0f
#define APP_FRUIT_A_LEFT_Z_MM                            (-100.0f)

/* 业务点2：物理右侧Y<0，实机q1=-90deg一侧。 */
#define APP_FRUIT_A_RIGHT_X_MM                              0.0f
#define APP_FRUIT_A_RIGHT_Y_MM                         (-400.0f)
#define APP_FRUIT_A_RIGHT_Z_MM                           (-100.0f)
/* 世界绝对俯仰-90deg：夹爪中心线始终竖直向下。 */
#define APP_FRUIT_A_PICK_TOOL_PITCH_DEG                   (-90.0f)

#define APP_FRUIT_A_TRANSFER_Q2_DEG                         90.0f
#define APP_FRUIT_A_TRANSFER_Q3_DEG                       (-100.0f)
#define APP_FRUIT_A_RELEASE_Q2_DEG                         120.0f
#define APP_FRUIT_A_RELEASE_Q3_DEG                        (-70.0f)
/* ID2张开后保持大臂不动，小臂绝对俯仰由+10deg再上抬到+20deg。 */
#define APP_FRUIT_A_RELEASE_CLEARANCE_Q3_DEG              (-80.0f)
#define APP_FRUIT_A_RELEASE_TOOL_RELATIVE_PITCH_DEG       (-45.0f)
#define APP_FRUIT_A_RELEASE_PITCH_WAIT_TIMEOUT_MS          1500u

/* 点1由Y>0解算到q1=+90deg，沿正角方向转到后方并避开+180deg翻边。 */
#define APP_FRUIT_A_LEFT_SAFE_Q1_DEG                       90.0f
#define APP_FRUIT_A_LEFT_PLACE_WAYPOINT_Q1_DEG            135.0f
#define APP_FRUIT_A_LEFT_PLACE_Q1_DEG                      178.0f
#define APP_FRUIT_A_LEFT_FRONT_WAYPOINT_Q1_DEG             90.0f
#define APP_FRUIT_A_LEFT_FRONT_Q1_DEG                         0.0f

/* 点2由Y<0解算到q1=-90deg，沿负角方向转到后方并避开-180deg翻边。 */
#define APP_FRUIT_A_RIGHT_SAFE_Q1_DEG                    (-90.0f)
#define APP_FRUIT_A_RIGHT_PLACE_WAYPOINT_Q1_DEG         (-135.0f)
#define APP_FRUIT_A_RIGHT_PLACE_Q1_DEG                   (-178.0f)
#define APP_FRUIT_A_RIGHT_FRONT_WAYPOINT_Q1_DEG           (-90.0f)
#define APP_FRUIT_A_RIGHT_FRONT_Q1_DEG                       0.0f

#endif
