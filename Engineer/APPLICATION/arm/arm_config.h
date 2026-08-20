/**
 * @file arm_config.h
 * @brief 三轴达妙机械臂和双幻儿舵机工具端的几何、限位与时序参数。
 */

#ifndef __ARM_CONFIG_H__
#define __ARM_CONFIG_H__

#include "app_config.h"

/* 坐标：+X车头、+Y物理左侧、-Y物理右侧、+Z向上；长度mm、角度deg、时间ms。 */

/*
 * 完整初始化先使能三台达妙并同步当前位置，将ARM_USB_HOME_*作为ID1
 * 俯仰舵机轴心坐标执行IK；q2/q3同步HOME、q1随后HOME，最后才初始化
 * ID1/ID2并让ID2回到默认位置，然后发布整机READY。
 * 历史名称DM_SINGLE_AXIS_TEST仅作源码兼容，不再表示单轴测试。
 */
#define ARM_BOOT_MODE_NORMAL               0u
#define ARM_BOOT_MODE_FULL_INIT            1u
#define ARM_BOOT_MODE_DM_SINGLE_AXIS_TEST  ARM_BOOT_MODE_FULL_INIT
#define ARM_BOOT_MODE_TEACH_POINT          2u
#define ARM_BOOT_MODE_DM_ENABLE_ONLY       3u
#define ARM_BOOT_MODE_TOOL_SERVO_INIT_ONLY 4u
#ifndef ARM_BOOT_MODE
#if APP_ARM_TEACH_POINT_ENABLED
#define ARM_BOOT_MODE ARM_BOOT_MODE_TEACH_POINT
#else
#define ARM_BOOT_MODE ARM_BOOT_MODE_FULL_INIT
#endif
#endif

#define ARM_DM_TEST_NONE      0u
#define ARM_DM_TEST_BASE      1u
#define ARM_DM_TEST_SHOULDER  2u
#define ARM_DM_TEST_ELBOW     3u
/* Watch专用：上电HOME阶段大臂和小臂由同一联合位姿命令驱动。 */
#define ARM_DM_TEST_SHOULDER_ELBOW 4u
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

/* 关节命令软件限位；底座允许固定释放序列转到正后方。 */
#define ARM_LIMIT_TOLERANCE_DEG             0.2f
#define ARM_Q1_SOFT_MIN_DEG              (-180.0f)
#define ARM_Q1_SOFT_MAX_DEG                180.0f
/*
 * 大臂硬件和普通运动范围均为0deg~180deg。
 * 朝正前方时仍由ARM_FRONT_BARRIER_SHOULDER_Q2_MAX_DEG独立限制栏框侧动作。
 */
#define ARM_Q2_SOFT_MIN_DEG                  0.0f
#define ARM_Q2_SOFT_MAX_DEG                180.0f
/* q3采用机械定义：q3 = -两杆物理内夹角。
 * 正常物理夹角35deg~190deg，对应q3=-190deg~-35deg；
 * 上限小幅放宽，为当前HOME及近端低位运动保留解算余量。 */
#define ARM_Q3_SOFT_MIN_DEG              (-190.0f)
#define ARM_Q3_SOFT_MAX_DEG               (-35.0f)
/* 底座启动脱困/硬边界与关节命令范围一致；普通IK仍由ARM_AUTO_Q1限制。 */
#define ARM_Q1_ESCAPE_MIN_DEG            (-180.0f)
#define ARM_Q1_ESCAPE_MAX_DEG              180.0f
/* q2脱困边界与普通软件限位一致，均使用完整硬件角度范围。 */
#define ARM_Q2_ESCAPE_MIN_DEG                0.0f
#define ARM_Q2_ESCAPE_MAX_DEG              180.0f
/* 脱困物理夹角20deg~210deg，对应q3=-210deg~-20deg。 */
#define ARM_Q3_ESCAPE_MIN_DEG             (-210.0f)
#define ARM_Q3_ESCAPE_MAX_DEG              (-20.0f)

/* IK/轨迹层只允许正常软件限位，不能使用脱困边界。 */
/* 普通FK/IK和笛卡尔轨迹仍限制在前方+/-90deg；后方180deg只供明确关节序列。 */
#define ARM_AUTO_Q1_MIN_DEG                (-90.0f)
#define ARM_AUTO_Q1_MAX_DEG                  90.0f
/* 仅供显式标记的AC低位侧抓命令使用；其他自动轨迹仍保持+/-90deg。 */
#define ARM_AC_SIDE_PICK_Q1_MIN_DEG        (-115.0f)
#define ARM_AC_SIDE_PICK_Q1_MAX_DEG          115.0f
/* 自动轨迹沿用q2完整的0deg~180deg普通软件限位。 */
#define ARM_AUTO_Q2_MIN_DEG ARM_Q2_SOFT_MIN_DEG
#define ARM_AUTO_Q2_MAX_DEG ARM_Q2_SOFT_MAX_DEG
#define ARM_AUTO_Q3_MIN_DEG ARM_Q3_SOFT_MIN_DEG
#define ARM_AUTO_Q3_MAX_DEG ARM_Q3_SOFT_MAX_DEG

/*
 * HOME解析目标：q1=0deg，物理大臂由-90deg往后靠20deg到-110deg，
 * 对应内部q2=+110deg；小臂按两杆物理夹角收至40deg，对应内部q3=-40deg。
 * ARM_SAFE_Q*既用于HOME的IK
 * 参考/自检，也供旧NORMAL顺序回位流程使用；电机永久零位参数不改变。
 */
#define ARM_HOME_SHOULDER_PHYSICAL_DEG    (-110.0f)
#define ARM_HOME_ELBOW_INCLUDED_DEG          40.0f
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
/*
 * 多段route的中间waypoint只用于避障/绕行，不是最终作业点。
 * 放宽中间点到位窗口，允许轨迹更早切到下一段，减少抓取后去放置时
 * 在安全过渡点明显停顿；最终目标仍使用上面的严格到位判定。
 */
#define ARM_ROUTE_WAYPOINT_ARRIVAL_ERROR_DEG  12.0f
#define ARM_ROUTE_WAYPOINT_ARRIVAL_SPEED_DEG_S 100.0f
#define ARM_ROUTE_WAYPOINT_ARRIVAL_STABLE_MS   0u
#define ARM_WRONG_DIRECTION_DELTA_DEG          0.5f
#define ARM_WRONG_DIRECTION_TIME_MS           200u
#define ARM_TEMPERATURE_HOLD_C                 70.0f
#define ARM_TEMPERATURE_DISABLE_C              80.0f
#define ARM_DM_TX_FAIL_LIMIT                    5u

/*
 * 三轴上电初始化：反馈和使能就绪后，先让大臂/小臂同步到HOME，再让
 * 底座单独到HOME；HOME坐标统一复用ARM_USB_HOME_*。
 */
#define ARM_DM_AUTO_INIT_ENABLE                  1u
#define ARM_DM_AUTO_INIT_START_DELAY_MS       1000u
#define ARM_DM_AUTO_INIT_STEP_TIMEOUT_MS     25000u
#define ARM_DM_AUTO_INIT_SPEED_DEG_S            30.0f

/* 四点循环仅保留为台架测试；正式上位机接口版默认关闭。 */

/* 正常轨迹逐轴达妙命令上限；底座Q1独立提速，Q2/Q3保持保守档位。 */
#define ARM_JOINT_Q1_COMMAND_SPEED_DEG_S        200.0f
#define ARM_JOINT_Q2_COMMAND_SPEED_DEG_S        150.0f
#define ARM_JOINT_Q3_COMMAND_SPEED_DEG_S        150.0f
#define ARM_LINEAR_DEFAULT_SPEED_MM_S           700.0f
#define ARM_LINEAR_MAX_SPEED_MM_S               700.0f
#define ARM_LINEAR_MAX_ACCEL_MM_S2             5000.0f
#define ARM_LINEAR_Q1_MAX_SPEED_DEG_S            200.0f
#define ARM_LINEAR_Q2_MAX_SPEED_DEG_S            150.0f
#define ARM_LINEAR_Q3_MAX_SPEED_DEG_S            150.0f
#define ARM_LINEAR_Q1_MAX_ACCEL_DEG_S2           800.0f
#define ARM_LINEAR_Q2_MAX_ACCEL_DEG_S2           800.0f
#define ARM_LINEAR_Q3_MAX_ACCEL_DEG_S2           800.0f
#define ARM_LINEAR_SAMPLE_SPACING_MM               1.0f
#define ARM_LINEAR_MAX_SAMPLES                   1536u
/* 相邻笛卡尔IK样本允许的最大关节变化，规划器和执行审计共用。 */
#define ARM_LINEAR_Q1_STEP_MAX_DEG                  5.0f
#define ARM_LINEAR_Q2_STEP_MAX_DEG                  2.0f
#define ARM_LINEAR_Q3_STEP_MAX_DEG                  2.0f

/*
 * 夹爪中心工作区安全参数，单位均为mm。
 * 负X区域位于后框一侧：目标及路径必须高于160；进入后区前先抬到210。
 * X=0两侧各保留2mm死区，避免q1=+/-90deg时因浮点误差误判跨区。
 * 205是开始水平跨区的实测放行线，200是运行时立即中止保护线。
 */
#define ARM_WORKSPACE_SAFETY_ENABLE                 1u
#define ARM_REAR_ZONE_X_BOUNDARY_MM                 0.0f
#define ARM_REAR_ZONE_X_MARGIN_MM                   2.0f
#define ARM_REAR_ZONE_MIN_TOOL_Z_MM               160.0f
/*
 * 前方栏框肩关节保护为旧机械结构遗留限制。当前实机前方无遮挡，关闭该
 * 保护；后方框仍由REAR_ZONE/REAR_CROSSING系列高度保护负责。
 */
#define ARM_FRONT_BARRIER_SHOULDER_LIMIT_ENABLE      0u
/*
 * 大臂从实机零点朝栏框方向最多允许到120deg。仅在
 * ARM_FRONT_BARRIER_SHOULDER_LIMIT_ENABLE打开时生效。
 */
#define ARM_FRONT_BARRIER_SHOULDER_Q2_MAX_DEG      120.0f
/* 底座逻辑角绝对值不超过90deg时，认为机械臂仍面向前方栏框。 */
#define ARM_FRONT_BARRIER_BASE_Q1_ABS_MAX_DEG       90.0f
/* 浮点与反馈噪声容差：工具中心X>2mm才视为仍在正前方栏框区域。 */
#define ARM_FRONT_BARRIER_TOOL_X_MARGIN_MM            2.0f
#define ARM_REAR_CROSSING_TOOL_Z_MM               210.0f
#define ARM_REAR_CROSSING_ACTUAL_GATE_Z_MM        205.0f
#define ARM_REAR_CROSSING_ABORT_Z_MM              200.0f
/* 正负X绕行包含抬升、两段15deg圆弧和侧向径向转换，最多预留16段。 */
#define ARM_TRAJECTORY_MAX_ROUTE_SEGMENTS           16u
#define ARM_REAR_BYPASS_ARC_STEP_DEG                15.0f
#define ARM_LINEAR_FK_ERROR_MAX_MM                  0.5f
#define ARM_LINEAR_HOLD_MS                        1500u
#define ARM_LINEAR_IK_UPDATE_MS                      2u
/* 参考轨迹结束后等待三轴实际到位；超时向Host报告运动超时。 */
#define ARM_TRAJECTORY_SETTLE_TIMEOUT_MS           8000u
#define ARM_REALTIME_COMMAND_TIMEOUT_MS            100u
#define ARM_REALTIME_DEFAULT_ACCEL_MM_S2           80.0f
#define ARM_TRACKING_ERROR_WARN_DEG                  5.0f

/* 末端工具由 USART6 控制板上的 ID1 俯仰舵机和 ID2 夹爪舵机管理。 */
#define ARM_TOOL_ENABLE                               1u
#define ARM_TOOL_SERVO_RANGE_DEG                    240.0f
#define ARM_TOOL_SERVO_POS_MIN                        0u
#define ARM_TOOL_SERVO_POS_MAX                     1000u

#define ARM_TOOL_PITCH_SERVO_ID                       1u
#define ARM_TOOL_PITCH_NEUTRAL_POS                  500u
#define ARM_TOOL_PITCH_SERVO_MIN_POS                115u
#define ARM_TOOL_PITCH_SERVO_MAX_POS                875u
#define ARM_TOOL_PITCH_RELATIVE_MIN_DEG            (-90.0f)
/* 控制值115按当前反向映射对应ID1相对小臂最大俯仰+92.4deg。 */
#define ARM_TOOL_PITCH_RELATIVE_MAX_DEG              92.4f
/*
 * ID1实机安装方向：控制值增大时夹爪相对小臂向下转，因此绝对俯仰
 * 增大必须使控制值减小。该符号同时用于目标换算和反馈反算，禁止只改一侧。
 */
#define ARM_TOOL_PITCH_DIRECTION                     (-1.0f)
/* ID1输出轴中心到夹爪中心的实测距离；沿夹爪绝对俯仰方向参与工具坐标换算。 */
#define ARM_TOOL_PITCH_AXIS_TO_CENTER_MM            117.0f
#define ARM_TOOL_PITCH_UPDATE_PERIOD_MS              20u
#define ARM_TOOL_PITCH_COMMAND_DEADBAND_POS            2u
#define ARM_TOOL_PITCH_TRACK_TIME_MS                   0u
/*
 * 9600波特率总线上的位置查询可能偶发超过100ms新鲜度窗口。轨迹期间允许
 * ID1沿用最近一次有效反馈短暂继续跟踪；超过该时限才按离线故障中止。
 */
#define ARM_TOOL_PITCH_FEEDBACK_ABORT_MS             300u

#define ARM_GRIPPER_SERVO_ID                           2u
#define ARM_GRIPPER_SERVO_MIN_POS                    450u
#define ARM_GRIPPER_SERVO_MAX_POS                    660u
/* ID2上电、等待抓取和释放均回到默认张开位置450。 */
#define ARM_GRIPPER_DEFAULT_POS                      450u
#define ARM_GRIPPER_BOOT_POS          ARM_GRIPPER_DEFAULT_POS
#define ARM_GRIPPER_READY_POS         ARM_GRIPPER_DEFAULT_POS
#define ARM_GRIPPER_OPEN_POS          ARM_GRIPPER_DEFAULT_POS
/* 收到抓取命令后，ID2向控制值增大方向探测闭合到660。 */
#define ARM_GRIPPER_CLOSE_POS                        660u
#define ARM_GRIPPER_MOVE_TIME_MS                     500u
#define ARM_GRIPPER_BOOT_MOVE_TIME_MS               1000u
/* ID2张开/回等待位允许短时反馈中断；恢复后仍须由新鲜反馈确认到位。 */
#define ARM_GRIPPER_FEEDBACK_RECOVERY_TIMEOUT_MS     500u
/*
 * ID2闭合使用独立的小到位窗口，避免通用舵机+/-15容差吞掉堵转判定。
 * 误差0..5视为正常到位，误差>=6才允许进入停滞检测，两者无空档。
 */
#define ARM_GRIPPER_CLOSE_ARRIVAL_ERROR_POS            5u
#define ARM_GRIPPER_CLOSE_ARRIVAL_STABLE_MS           120u
#define ARM_GRIPPER_STALL_ERROR_POS                     6u
/*
 * 检测到接触后每次向张开方向回退10，最多回退4次（累计最多40）。
 * 任一次回退到位并稳定后立即停止；4次均不能跟随则强制按抓取完成。
 */
#define ARM_GRIPPER_RELIEF_STEP_POS                   10u
#define ARM_GRIPPER_RELIEF_MAX_ATTEMPTS                4u
#define ARM_GRIPPER_RELIEF_ARRIVAL_ERROR_POS            5u
#define ARM_GRIPPER_RELIEF_MOVE_TIME_MS              150u
#define ARM_GRIPPER_RELIEF_ATTEMPT_TIMEOUT_MS         400u
#define ARM_GRIPPER_SETTLE_MS                        200u
#define ARM_GRIPPER_STALL_START_IGNORE_MS            300u
#define ARM_GRIPPER_STALL_WINDOW_MS                  300u
#define ARM_GRIPPER_STALL_MAX_POSITION_SPAN            3u
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
/*
 * HOME继续使用ID1俯仰舵机输出轴轴心，便于机构重装后独立校准主臂。
 * 夹爪中心命令必须显式选择ARM_CONTROL_POINT_TOOL_CENTER，固件会按
 * ARM_TOOL_PITCH_AXIS_TO_CENTER_MM和绝对俯仰角反算ID1轴心目标。
 */
#define ARM_USB_HOME_X_MM                          136.2414f
#define ARM_USB_HOME_Y_MM                            0.0f
#define ARM_USB_HOME_Z_MM                          176.3204f
#define ARM_USB_HOME_SPEED_MM_S                    200.0f
#define ARM_USB_TOOL_PITCH_MIN_DEG                (-180.0f)
#define ARM_USB_TOOL_PITCH_MAX_DEG                  180.0f
#define ARM_USB_TOOL_PITCH_MOVE_TIME_MS             500u
#define ARM_USB_TOOL_SETTLE_MS                      200u

#endif
