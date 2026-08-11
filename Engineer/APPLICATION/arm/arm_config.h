#ifndef __ARM_CONFIG_H__
#define __ARM_CONFIG_H__

/* 坐标：+X车头、+Y车体左侧、+Z向上；长度mm、角度deg、时间ms。 */

/*
 * 完整初始化复用已经实机验证的组合流程：先闭环初始化ID1/ID2舵机，
 * 再使能三台达妙并同步当前位置，随后将ARM_USB_HOME_*作为ID1俯仰
 * 舵机轴心坐标执行IK归正；最后ID2回到默认位置550才发布整机READY。
 * 历史名称DM_SINGLE_AXIS_TEST仅作源码兼容，不再表示单轴测试。
 */
#define ARM_BOOT_MODE_NORMAL               0u
#define ARM_BOOT_MODE_FULL_INIT            1u
#define ARM_BOOT_MODE_DM_SINGLE_AXIS_TEST  ARM_BOOT_MODE_FULL_INIT
#define ARM_BOOT_MODE_TEACH_POINT          2u
#define ARM_BOOT_MODE_DM_ENABLE_ONLY       3u
#define ARM_BOOT_MODE_TOOL_SERVO_INIT_ONLY 4u
#ifndef ARM_BOOT_MODE
#define ARM_BOOT_MODE ARM_BOOT_MODE_FULL_INIT
#endif

#define ARM_DM_TEST_NONE      0u
#define ARM_DM_TEST_BASE      1u
#define ARM_DM_TEST_SHOULDER  2u
#define ARM_DM_TEST_ELBOW     3u
#ifndef ARM_DM_TEST_AXIS
#define ARM_DM_TEST_AXIS ARM_DM_TEST_NONE
#endif

#define ARM_REALTIME_HOST_SIM_ENABLE       0u
#define ARM_BOOT_STABILIZE_MS             500u
#define ARM_BOOT_TOOL_INIT_TIMEOUT_MS    8000u
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

/*
 * 电机零位已由达妙上位机永久保存；固件不发送清零命令。
 * 底座必须先物理朝向用户定义的+X方向，再把该姿态保存为0deg。
 * 固件保持q1=0deg对应+X，不在FK/IK中反转X，也不增加180deg软件偏置。
 */
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
#define ARM_LINK_2_MM                    260.0f
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
 * 正常物理夹角35deg~190deg，对应q3=-190deg~-35deg；
 * 上限小幅放宽，为当前HOME及近端低位运动保留解算余量。 */
#define ARM_Q3_SOFT_MIN_DEG              (-190.0f)
#define ARM_Q3_SOFT_MAX_DEG               (-35.0f)
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

/*
 * HOME解析目标：q1=0deg，物理大臂=-90deg对应内部q2=+90deg，
 * 两杆物理夹角60deg对应内部q3=-60deg。ARM_SAFE_Q*既用于HOME的IK
 * 参考/自检，也供旧NORMAL顺序回位流程使用；电机永久零位参数不改变。
 */
#define ARM_HOME_SHOULDER_PHYSICAL_DEG     (-90.0f)
#define ARM_HOME_ELBOW_INCLUDED_DEG          60.0f
#define ARM_SAFE_Q1_DEG                       0.0f
#define ARM_SAFE_Q2_DEG  (-(ARM_HOME_SHOULDER_PHYSICAL_DEG))
#define ARM_SAFE_Q3_DEG  (-(ARM_HOME_ELBOW_INCLUDED_DEG))

/* 启动、脱困、回位和保护参数。 */
/* 主控与达妙同时上电时，先留出电机自身启动时间，再进入原初始化流程。 */
#define ARM_DM_POWER_ON_DELAY_MS          1000u
#define ARM_PASSIVE_FEEDBACK_WAIT_MS       300u
#define ARM_ENTER_MODE_FEEDBACK_WAIT_MS    500u
#define ARM_DM_ENABLE_REFRESH_MS            100u
#define ARM_FEEDBACK_TIMEOUT_MS            100u
#define ARM_FEEDBACK_STABLE_MS               20u
#define ARM_ESCAPE_SPEED_DEG_S               5.0f
#define ARM_RETURN_SPEED_DEG_S               10.0f
#define ARM_ESCAPE_TIMEOUT_MS              8000u
#define ARM_RETURN_TIMEOUT_MS             15000u
#define ARM_ARRIVAL_ERROR_DEG                 2.0f
#define ARM_ARRIVAL_SPEED_DEG_S               5.0f
#define ARM_ARRIVAL_STABLE_MS               120u
#define ARM_WRONG_DIRECTION_DELTA_DEG          0.5f
#define ARM_WRONG_DIRECTION_TIME_MS           200u
#define ARM_TEMPERATURE_HOLD_C                 70.0f
#define ARM_TEMPERATURE_DISABLE_C              80.0f
#define ARM_DM_TX_FAIL_LIMIT                    5u

/*
 * 三轴上电初始化：反馈和使能就绪后，直接解算并低速移动到HOME末端点。
 * HOME坐标统一复用ARM_USB_HOME_*，不再先经过固定关节初始化姿态。
 */
#define ARM_DM_AUTO_INIT_ENABLE                  1u
#define ARM_DM_AUTO_INIT_START_DELAY_MS       1000u
#define ARM_DM_AUTO_INIT_STEP_TIMEOUT_MS     25000u
#define ARM_DM_AUTO_INIT_SPEED_DEG_S            60.0f

/* 四点循环仅保留为台架测试；正式上位机接口版默认关闭。 */

/* 达妙三轴第二档快速轨迹：700mm/s，短行程同时依靠更高加速度提速。 */
#define ARM_JOINT_COMMAND_SPEED_DEG_S           420.0f
#define ARM_LINEAR_DEFAULT_SPEED_MM_S           700.0f
#define ARM_LINEAR_MAX_SPEED_MM_S               700.0f
#define ARM_LINEAR_MAX_ACCEL_MM_S2             5000.0f
#define ARM_LINEAR_Q1_MAX_SPEED_DEG_S            150.0f
#define ARM_LINEAR_Q2_MAX_SPEED_DEG_S            250.0f
#define ARM_LINEAR_Q3_MAX_SPEED_DEG_S            260.0f
#define ARM_LINEAR_Q1_MAX_ACCEL_DEG_S2          1400.0f
#define ARM_LINEAR_Q2_MAX_ACCEL_DEG_S2          1600.0f
#define ARM_LINEAR_Q3_MAX_ACCEL_DEG_S2          1800.0f
#define ARM_LINEAR_SAMPLE_SPACING_MM               1.0f
#define ARM_LINEAR_MAX_SAMPLES                    384u
#define ARM_LINEAR_FK_ERROR_MAX_MM                  0.5f
#define ARM_LINEAR_HOLD_MS                        1500u
#define ARM_LINEAR_IK_UPDATE_MS                      2u
/* 参考轨迹结束后等待三轴实际到位；超时向Host报告运动超时。 */
#define ARM_TRAJECTORY_SETTLE_TIMEOUT_MS           8000u
#define ARM_REALTIME_COMMAND_TIMEOUT_MS            100u
#define ARM_REALTIME_DEFAULT_ACCEL_MM_S2           80.0f
#define ARM_TRACKING_ERROR_WARN_DEG                  5.0f

/* 腕部PWM本轮不接管；末端工具由USART6总线舵机和PB12电磁铁管理。 */
#define ARM_WRIST_ENABLE                            0u
#define ARM_WRIST_PWM_MIN_US                      1000u
#define ARM_WRIST_PWM_MID_US                      1500u
#define ARM_WRIST_PWM_MAX_US                      2000u
#define ARM_WRIST_MIN_ANGLE_DEG                  (-90.0f)
#define ARM_WRIST_MAX_ANGLE_DEG                    90.0f
#define ARM_WRIST_ZERO_OFFSET_DEG                   0.0f
#define ARM_WRIST_DIRECTION                         1.0f

/* 恢复USART6上的ID1俯仰舵机和ID2夹爪舵机闭环初始化及反馈轮询。 */
#define ARM_TOOL_ENABLE                               1u
#define ARM_TOOL_SERVO_RANGE_DEG                    240.0f
#define ARM_TOOL_SERVO_POS_MIN                        0u
#define ARM_TOOL_SERVO_POS_MAX                     1000u

#define ARM_TOOL_PITCH_SERVO_ID                       1u
#define ARM_TOOL_PITCH_NEUTRAL_POS                  500u
#define ARM_TOOL_PITCH_SERVO_MIN_POS                125u
#define ARM_TOOL_PITCH_SERVO_MAX_POS                875u
#define ARM_TOOL_PITCH_RELATIVE_MIN_DEG            (-90.0f)
#define ARM_TOOL_PITCH_RELATIVE_MAX_DEG              90.0f
#define ARM_TOOL_PITCH_DIRECTION                      1.0f
#define ARM_TOOL_PITCH_AXIS_TO_CENTER_MM             30.0f
#define ARM_TOOL_PITCH_UPDATE_PERIOD_MS              20u
#define ARM_TOOL_PITCH_COMMAND_DEADBAND_POS            2u
#define ARM_TOOL_PITCH_TRACK_TIME_MS                   0u

#define ARM_GRIPPER_SERVO_ID                           2u
#define ARM_GRIPPER_SERVO_MIN_POS                    550u
#define ARM_GRIPPER_SERVO_MAX_POS                    630u
/* ID2上电、等待抓取和释放均回到默认张开位置550。 */
#define ARM_GRIPPER_DEFAULT_POS                      550u
#define ARM_GRIPPER_BOOT_POS          ARM_GRIPPER_DEFAULT_POS
#define ARM_GRIPPER_READY_POS         ARM_GRIPPER_DEFAULT_POS
#define ARM_GRIPPER_OPEN_POS          ARM_GRIPPER_DEFAULT_POS
/* 收到抓取命令后，ID2向控制值增大方向闭合到630。 */
#define ARM_GRIPPER_CLOSE_POS                        630u
#define ARM_GRIPPER_MOVE_TIME_MS                     500u
#define ARM_GRIPPER_BOOT_MOVE_TIME_MS               1000u
#define ARM_GRIPPER_RELIEF_POS                        10u
#define ARM_GRIPPER_RELIEF_MOVE_TIME_MS              150u
#define ARM_GRIPPER_SETTLE_MS                        200u
#define ARM_GRIPPER_STALL_START_IGNORE_MS            300u
#define ARM_GRIPPER_STALL_ERROR_POS                   20u
#define ARM_GRIPPER_STALL_WINDOW_MS                  300u
#define ARM_GRIPPER_STALL_MAX_POSITION_SPAN            3u
#define ARM_GRIPPER_STALL_MIN_CLOSE_TRAVEL_POS        20u
#define ARM_GRIPPER_CLOSE_DEADLINE_MS               1500u
#define ARM_GRIPPER_BOOT_DEADLINE_MS                2000u

#define ARM_TOOL_BOOT_START_DELAY_MS                 300u
#define ARM_TOOL_BOOT_MOVE_TIME_MS                  1000u
#define ARM_TOOL_ACTION_DEADLINE_MARGIN_MS           500u

/* Compatibility IDs remain aliases while callers migrate to tool semantics. */
#define ARM_TOOL_SERVO1_ID ARM_TOOL_PITCH_SERVO_ID
#define ARM_TOOL_SERVO2_ID ARM_GRIPPER_SERVO_ID

/*
 * USB末端高度随X方向线性标定（上位机坐标单位均为mm）：
 *   X <= 240mm: 默认高度32mm，抓取/释放高度20mm
 *   X >= 450mm: 默认高度35mm，抓取/释放高度23mm
 *   中间区间按两个端点线性插值。
 */
#define ARM_USB_Z_MAP_X_MIN_MM                     240.0f
#define ARM_USB_Z_MAP_X_MAX_MM                     450.0f
#define ARM_USB_MOVE_Z_AT_X_MIN_MM                  37.2f
#define ARM_USB_MOVE_Z_AT_X_MAX_MM                  51.7f
#define ARM_USB_GRIPPER_Z_AT_X_MIN_MM               21.6f
#define ARM_USB_GRIPPER_Z_AT_X_MAX_MM               21.8f
#define ARM_USB_TARGET_TRAVEL_Z_MM                   45.0f
#define ARM_USB_MOVE_SPEED_MM_S                    700.0f
#define ARM_USB_GRIPPER_Z_SPEED_MM_S                100.0f
/* 上位机/HOME的XYZ均表示ID1俯仰舵机轴心，不包含轴外30mm夹爪长度。 */
#define ARM_USB_HOME_X_MM                          225.1666f
#define ARM_USB_HOME_Y_MM                            0.0f
#define ARM_USB_HOME_Z_MM                          192.0f
#define ARM_USB_HOME_SPEED_MM_S                    200.0f
#define ARM_USB_TOOL_PITCH_MIN_DEG                (-180.0f)
#define ARM_USB_TOOL_PITCH_MAX_DEG                  180.0f
#define ARM_USB_TOOL_PITCH_MOVE_TIME_MS             500u
#define ARM_USB_TOOL_SETTLE_MS                      200u

#endif
