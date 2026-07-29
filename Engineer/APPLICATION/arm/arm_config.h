#ifndef __ARM_CONFIG_H__
#define __ARM_CONFIG_H__

/* 坐标：+X车头、+Y车体左侧、+Z向上；长度mm、角度deg、时间ms。 */

/*
 * 当前为三轴联调模式：复用已验证的三电机使能/当前位置保持流程，
 * 然后通过g_arm_dm_debug.auto_init按小臂/大臂/底座顺序回初始化姿态。
 * 三轴方向确认后，将ARM_BOOT_MODE切到ARM_BOOT_MODE_NORMAL即可进入
 * 自动脱困和小臂/大臂/底座顺序回安全姿态。
 */
#define ARM_BOOT_MODE_NORMAL               0u
#define ARM_BOOT_MODE_DM_SINGLE_AXIS_TEST  1u
#define ARM_BOOT_MODE_TEACH_POINT          2u
#define ARM_BOOT_MODE_DM_ENABLE_ONLY       3u
#ifndef ARM_BOOT_MODE
#define ARM_BOOT_MODE ARM_BOOT_MODE_DM_SINGLE_AXIS_TEST
#endif

#define ARM_DM_TEST_NONE      0u
#define ARM_DM_TEST_BASE      1u
#define ARM_DM_TEST_SHOULDER  2u
#define ARM_DM_TEST_ELBOW     3u
#ifndef ARM_DM_TEST_AXIS
#define ARM_DM_TEST_AXIS ARM_DM_TEST_NONE
#endif

/* 旧自动点位和上位机模拟在达妙整机验证完成前一律关闭。 */
#define ARM_API_POINT_TEST_ENABLE          0u
#define ARM_REALTIME_HOST_SIM_ENABLE       0u
#define ARM_API_POINT_TEST_X_MM          260.0f
#define ARM_API_POINT_TEST_Y_MM            0.0f
#define ARM_API_POINT_TEST_Z_MM          510.0f
#define ARM_API_POINT_TEST_SPEED_MM_S     20.0f
#define ARM_REALTIME_HOST_SIM_PERIOD_MS    10u
#define ARM_REALTIME_HOST_SIM_CYCLE_MS   6000u
#define ARM_REALTIME_HOST_SIM_SPEED_MM_S   20.0f
#define ARM_REALTIME_HOST_SIM_ACCEL_MM_S2  40.0f
#define ARM_REALTIME_HOST_SIM_FIXED_POINT_ENABLE 0u
#define ARM_REALTIME_HOST_SIM_FIXED_X_MM 260.0f
#define ARM_REALTIME_HOST_SIM_FIXED_Y_MM   0.0f
#define ARM_REALTIME_HOST_SIM_FIXED_Z_MM 510.0f
#define ARM_REALTIME_HOST_SIM_CENTER_Q1_DEG  0.0f
#define ARM_REALTIME_HOST_SIM_CENTER_Q2_DEG 90.0f
#define ARM_REALTIME_HOST_SIM_CENTER_Q3_DEG (-90.0f)
#define ARM_REALTIME_HOST_SIM_AMPLITUDE_Q1_DEG  2.0f
#define ARM_REALTIME_HOST_SIM_AMPLITUDE_Q2_DEG  2.0f
#define ARM_REALTIME_HOST_SIM_AMPLITUDE_Q3_DEG  2.0f

/* 三台电机均使用CAN1、标准帧、DLC8、位置速度模式。 */
#define ARM_BASE_MOTOR_ID          3u
#define ARM_BASE_MASTER_ID      0x13u
#define ARM_BASE_COMMAND_ID    0x103u
#define ARM_SHOULDER_MOTOR_ID      2u
#define ARM_SHOULDER_MASTER_ID  0x12u
#define ARM_SHOULDER_COMMAND_ID 0x102u
#define ARM_ELBOW_MOTOR_ID         1u
#define ARM_ELBOW_MASTER_ID     0x11u
#define ARM_ELBOW_COMMAND_ID   0x101u

/* 电机零位已由上位机永久保存；固件不再发送清零命令。 */
#define ARM_BASE_MOTOR_ZERO_TRIM_RAD       0.0f
#define ARM_SHOULDER_MOTOR_ZERO_TRIM_RAD   0.0f
#define ARM_ELBOW_MOTOR_ZERO_TRIM_RAD      0.0f
#define ARM_BASE_LOGICAL_ZERO_DEG          0.0f
#define ARM_SHOULDER_LOGICAL_ZERO_DEG    180.0f
#define ARM_ELBOW_LOGICAL_ZERO_DEG       (-90.0f)
#define ARM_BASE_DIRECTION                 1.0f
#define ARM_SHOULDER_DIRECTION             1.0f
#define ARM_ELBOW_DIRECTION                1.0f
/*
 * 小臂当前为1:1同步带传动，大臂转动会机械耦合小臂相对角。
 * 逻辑模型：q3 = elbow_motor_logical + k * (q2 - reference)。
 * 当前实机验证补偿方向为+1.0f。
 */
#define ARM_ELBOW_SHOULDER_COUPLING_ENABLE 1u
#define ARM_ELBOW_SHOULDER_COUPLING        1.0f
#define ARM_ELBOW_COUPLING_REFERENCE_DEG   ARM_SHOULDER_LOGICAL_ZERO_DEG

/* 三自由度腕部轴心几何。 */
#define ARM_BASE_HEIGHT_MM                62.0f
#define ARM_LINK_1_MM                    260.0f
#define ARM_LINK_2_MM                    230.0f
#define ARM_SHOULDER_OFFSET_FORWARD_MM     0.0f
#define ARM_SHOULDER_OFFSET_LEFT_MM        0.0f

/* 正常软件限位及仅供启动脱困使用的硬边界。 */
#define ARM_LIMIT_TOLERANCE_DEG             0.2f
#define ARM_Q1_SOFT_MIN_DEG               (-90.0f)
#define ARM_Q1_SOFT_MAX_DEG                 90.0f
/* 临时放宽到35deg，用于验证(230,0,150)低位点；正式版按机构安全范围回收。 */
#define ARM_Q2_SOFT_MIN_DEG                 35.0f
#define ARM_Q2_SOFT_MAX_DEG                180.0f
/* q3采用机械定义：q3 = -两杆物理内夹角。
 * 正常物理夹角40deg~190deg，对应q3=-190deg~-40deg。 */
#define ARM_Q3_SOFT_MIN_DEG              (-190.0f)
#define ARM_Q3_SOFT_MAX_DEG               (-40.0f)
#define ARM_Q1_ESCAPE_MIN_DEG            (-110.0f)
#define ARM_Q1_ESCAPE_MAX_DEG              110.0f
#define ARM_Q2_ESCAPE_MIN_DEG               20.0f
#define ARM_Q2_ESCAPE_MAX_DEG              200.0f
/* 脱困物理夹角20deg~210deg，对应q3=-210deg~-20deg。 */
#define ARM_Q3_ESCAPE_MIN_DEG             (-210.0f)
#define ARM_Q3_ESCAPE_MAX_DEG              (-20.0f)

/* IK/轨迹层只允许正常软件限位，不能使用脱困边界。 */
#define ARM_AUTO_Q1_MIN_DEG ARM_Q1_SOFT_MIN_DEG
#define ARM_AUTO_Q1_MAX_DEG ARM_Q1_SOFT_MAX_DEG
/* 临时放宽到35deg，用于验证(230,0,150)低位点；实机确认后再固化正式下限。 */
#define ARM_AUTO_Q2_MIN_DEG                 35.0f
#define ARM_AUTO_Q2_MAX_DEG ARM_Q2_SOFT_MAX_DEG
#define ARM_AUTO_Q3_MIN_DEG ARM_Q3_SOFT_MIN_DEG
#define ARM_AUTO_Q3_MAX_DEG ARM_Q3_SOFT_MAX_DEG

#define ARM_SAFE_Q1_DEG                     0.0f
#define ARM_SAFE_Q2_DEG                    90.0f
#define ARM_SAFE_Q3_DEG                  (-90.0f)

/* 启动、脱困、回位和保护参数。 */
#define ARM_PASSIVE_FEEDBACK_WAIT_MS       300u
#define ARM_ENTER_MODE_FEEDBACK_WAIT_MS    500u
#define ARM_DM_ENABLE_REFRESH_MS            100u
#define ARM_FEEDBACK_TIMEOUT_MS            100u
#define ARM_FEEDBACK_STABLE_MS               20u
#define ARM_ESCAPE_SPEED_DEG_S               5.0f
#define ARM_RETURN_SPEED_DEG_S               10.0f
#define ARM_ESCAPE_TIMEOUT_MS              8000u
#define ARM_RETURN_TIMEOUT_MS             15000u
#define ARM_ARRIVAL_ERROR_DEG                 1.0f
#define ARM_ARRIVAL_SPEED_DEG_S               2.0f
#define ARM_ARRIVAL_STABLE_MS               300u
#define ARM_WRONG_DIRECTION_DELTA_DEG          0.5f
#define ARM_WRONG_DIRECTION_TIME_MS           200u
#define ARM_TEMPERATURE_HOLD_C                 70.0f
#define ARM_TEMPERATURE_DISABLE_C              80.0f
#define ARM_DM_TX_FAIL_LIMIT                    5u

/* 三轴自动初始化姿态验证：三轴同时运动到目标姿态。 */
#define ARM_DM_AUTO_INIT_ENABLE                  1u
#define ARM_DM_AUTO_INIT_START_DELAY_MS       1000u
#define ARM_DM_AUTO_INIT_STEP_TIMEOUT_MS     25000u
#define ARM_DM_AUTO_INIT_SPEED_DEG_S            10.0f
#define ARM_DM_AUTO_INIT_BASE_Q_DEG              0.0f
#define ARM_DM_AUTO_INIT_SHOULDER_Q_DEG        180.0f
#define ARM_DM_AUTO_INIT_ELBOW_Q_DEG          (-90.0f)

/* 初始化完成后的单点IK验证。 */
#define ARM_DM_AUTO_POINT_ENABLE                 1u
#define ARM_DM_AUTO_POINT_X_MM                 250.0f
#define ARM_DM_AUTO_POINT_Y_MM                   0.0f
#define ARM_DM_AUTO_POINT_Z_MM                  120.0f
#define ARM_DM_AUTO_POINT_SPEED_DEG_S           10.0f
#define ARM_DM_AUTO_POINT_STEP_TIMEOUT_MS    25000u

/* 达妙直驱第一版保守轨迹参数。 */
#define ARM_JOINT_COMMAND_SPEED_DEG_S            10.0f
#define ARM_LINEAR_DEFAULT_SPEED_MM_S            20.0f
#define ARM_LINEAR_MAX_SPEED_MM_S                60.0f
#define ARM_LINEAR_MAX_ACCEL_MM_S2              100.0f
#define ARM_LINEAR_Q1_MAX_SPEED_DEG_S             15.0f
#define ARM_LINEAR_Q2_MAX_SPEED_DEG_S             12.0f
#define ARM_LINEAR_Q3_MAX_SPEED_DEG_S             15.0f
#define ARM_LINEAR_Q1_MAX_ACCEL_DEG_S2            30.0f
#define ARM_LINEAR_Q2_MAX_ACCEL_DEG_S2            24.0f
#define ARM_LINEAR_Q3_MAX_ACCEL_DEG_S2            30.0f
#define ARM_LINEAR_SAMPLE_SPACING_MM               2.0f
#define ARM_LINEAR_MAX_SAMPLES                    384u
#define ARM_LINEAR_FK_ERROR_MAX_MM                  0.5f
#define ARM_LINEAR_HOLD_MS                        1500u
#define ARM_LINEAR_IK_UPDATE_MS                      5u
#define ARM_REALTIME_COMMAND_TIMEOUT_MS            100u
#define ARM_REALTIME_DEFAULT_ACCEL_MM_S2           80.0f
#define ARM_TRACKING_ERROR_WARN_DEG                  5.0f

/* 腕部本轮不接管。 */
#define ARM_WRIST_ENABLE                            0u
#define ARM_TOOL_MODEL_ENABLE                       0u
#define ARM_WRIST_PWM_MIN_US                      1000u
#define ARM_WRIST_PWM_MID_US                      1500u
#define ARM_WRIST_PWM_MAX_US                      2000u
#define ARM_WRIST_MIN_ANGLE_DEG                  (-90.0f)
#define ARM_WRIST_MAX_ANGLE_DEG                    90.0f
#define ARM_WRIST_ZERO_OFFSET_DEG                   0.0f
#define ARM_WRIST_DIRECTION                         1.0f
#define ARM_TOOL_LENGTH_MM                           0.0f

#endif
