#ifndef __ARM_CONFIG_H__
#define __ARM_CONFIG_H__

/*
 * 机械臂集中配置
 * 坐标：+X车头、+Y车体左侧、+Z向上；长度mm，角度deg，时间ms。
 * 当前控制点为腕部舵机安装轴心，不包含舵机之后的末端长度。
 */

/*
 * 上电模式：所有模式互斥，只修改ARM_BOOT_MODE这一行。
 * NORMAL完成初始化后保持使能等待命令；AUTO_TEST循环三点轨迹；
 * TEACH_POINT完成同样初始化后失能，Watch只观察g_arm_teach_point；
 * FULL_CALIBRATION和SHOULDER_RATIO_TEST仅用于机构维护。
 */
#define ARM_BOOT_MODE_NORMAL               0u
#define ARM_BOOT_MODE_AUTO_TEST            1u
#define ARM_BOOT_MODE_TEACH_POINT          2u
#define ARM_BOOT_MODE_FULL_CALIBRATION     3u
#define ARM_BOOT_MODE_SHOULDER_RATIO_TEST  4u
#define ARM_BOOT_MODE_SHOULDER_FF_IDENTIFY 5u
#ifndef ARM_BOOT_MODE
#define ARM_BOOT_MODE ARM_BOOT_MODE_TEACH_POINT
#endif

/*
 * 一次性公开接口验证：仅用于本次打点坐标回放。
 * 初始化完成后，Test.c会调用一次ArmSubmitCartesianCommand()；完成实机验证后
 * 将ARM_API_POINT_TEST_ENABLE改回0，正式命令层即可接管同一个公开接口。
 */
#define ARM_API_POINT_TEST_ENABLE             0u
#define ARM_API_POINT_TEST_X_MM             103.8259070f
#define ARM_API_POINT_TEST_Y_MM              (-4.79646767f)
#define ARM_API_POINT_TEST_Z_MM              (-26.6742554f)
#define ARM_API_POINT_TEST_SPEED_MM_S        100.0f

/*
 * 临时100Hz上位机模拟：初始化后先移动到安全中心姿态，再每10ms提交一个
 * 新的腕部轴心坐标。目标由下列关节正弦轨迹经过FK生成，因此天然可达；
 * 正式连接USB/串口上位机前将ENABLE改回0即可，不需要删除Test.c代码。
 */
#define ARM_REALTIME_HOST_SIM_ENABLE             1u
#define ARM_REALTIME_HOST_SIM_PERIOD_MS          10u
#define ARM_REALTIME_HOST_SIM_CYCLE_MS         6000u
#define ARM_REALTIME_HOST_SIM_SPEED_MM_S       250.0f
#define ARM_REALTIME_HOST_SIM_ACCEL_MM_S2     1000.0f
/* 模型精度打点：1=持续发送固定XYZ，0=执行下方三关节正弦测试。 */
#define ARM_REALTIME_HOST_SIM_FIXED_POINT_ENABLE  1u
#define ARM_REALTIME_HOST_SIM_FIXED_X_MM         190.0f
#define ARM_REALTIME_HOST_SIM_FIXED_Y_MM          0.0f
#define ARM_REALTIME_HOST_SIM_FIXED_Z_MM          60.0f
#define ARM_REALTIME_HOST_SIM_CENTER_Q1_DEG      0.0f
#define ARM_REALTIME_HOST_SIM_CENTER_Q2_DEG     60.0f
#define ARM_REALTIME_HOST_SIM_CENTER_Q3_DEG    (-95.0f)
#define ARM_REALTIME_HOST_SIM_AMPLITUDE_Q1_DEG  25.0f
#define ARM_REALTIME_HOST_SIM_AMPLITUDE_Q2_DEG  12.0f
#define ARM_REALTIME_HOST_SIM_AMPLITUDE_Q3_DEG  10.0f

/* 三台电机均挂载在 CAN1。 */
#define ARM_BASE_MOTOR_CAN_ID          5u  /* GM6020，底座。 */
#define ARM_SHOULDER_MOTOR_CAN_ID      2u  /* M3508，大臂。 */
#define ARM_ELBOW_MOTOR_CAN_ID         3u  /* M2006，小臂。 */

/* 三自由度腕部轴心几何模型。 */
#define ARM_BASE_HEIGHT_MM               93.0f
#define ARM_LINK_1_MM                   150.0f
#define ARM_LINK_2_MM                   141.0f
#define ARM_SHOULDER_OFFSET_FORWARD_MM     0.0f
#define ARM_SHOULDER_OFFSET_LEFT_MM       (-4.0f)

/* GM6020为绝对编码器：朝车头时原始角190.369736deg，无需碰限位清零。 */
#define ARM_BASE_FRONT_RAW_DEG          190.369736f
#define ARM_BASE_DIRECTION                1.0f
#define ARM_BASE_ZERO_CONFIGURED          1u

/* 3508/2006模型端点和三次完整扫描得到的平均电机跨度。 */
#define ARM_SHOULDER_REFERENCE_DEG      180.0f
#define ARM_SHOULDER_OPPOSITE_DEG         0.0f
/* q3 follows the standard 2-link signed elbow definition used by FK/IK:
 * q3=0 means L2 points in the same direction as L1; q3=-180 means L2
 * points opposite to L1. It is not the travel angle from the reference stop.
 *
 * 机械上用量角器更容易测到“两杆夹角”：初始化端约49deg，另一端约143deg。
 * 该夹角越接近180deg，L1/L2越接近同向，机械臂伸得越远。
 * 固件FK/IK使用的标准q3 = 机械夹角 - 180deg，因此：
 *   49deg  -> q3 = -131deg
 *   143deg -> q3 =  -37deg
 */
#define ARM_ELBOW_REFERENCE_DEG        (-131.0f)
#define ARM_ELBOW_OPPOSITE_DEG          (-37.0f)
#define ARM_SHOULDER_MEASURED_SPAN_DEG  (-3461.26563f)
#define ARM_ELBOW_MEASURED_SPAN_DEG     (-3548.73088f)

/* 独立软件限位：相对机械硬限位每端缩进5deg。 */
#define ARM_SOFT_LIMIT_ENABLE             1u
#define ARM_SOFT_LIMIT_MARGIN_DEG          5.0f
#define ARM_AUTO_Q1_MIN_DEG             (-50.0f)
#define ARM_AUTO_Q1_MAX_DEG               50.0f
#define ARM_AUTO_Q2_MIN_DEG                5.0f
#define ARM_AUTO_Q2_MAX_DEG              175.0f
#define ARM_AUTO_Q3_MIN_DEG             (-126.0f)
#define ARM_AUTO_Q3_MAX_DEG              (-42.0f)

/*
 * 单边堵转初始化参数。
 * 判据为“电流超过阈值且速度低于阈值”；启动屏蔽避免加速电流误判，
 * 连续确认抑制毛刺。确认后立即失能，等待1s释放弹性，再清total_angle。
 */
#define ARM_AUTO_START_ONLINE_MS          10u
#define ARM_SHOULDER_HOMING_SPEED_DPS    450.0f
#define ARM_SHOULDER_STALL_CURRENT      600.0f
#define ARM_ELBOW_HOMING_SPEED_DPS      800.0f
#define ARM_ELBOW_STALL_CURRENT         1000.0f
#define ARM_HOMING_STALL_SPEED_DPS        20.0f
#define ARM_HOMING_SPINUP_MS             500u
#define ARM_HOMING_STALL_CONFIRM_MS        8u
#define ARM_STOP_SETTLE_MS              1000u
#define ARM_SHOULDER_MIN_DIRECTION         1.0f
#define ARM_ELBOW_MIN_DIRECTION            1.0f

/* 等效比只用于扫描释放距离和行程保护，不用于正常关节角映射。 */
#define ARM_SHOULDER_EFFECTIVE_RATIO      19.22925f
#define ARM_ELBOW_EFFECTIVE_RATIO         37.35506f
#define ARM_RELEASE_JOINT_DEG               1.0f
#define ARM_RELEASE_MIN_MS                500u
#define ARM_SHOULDER_STAGE_TIMEOUT_MS   45000u
#define ARM_ELBOW_STAGE_TIMEOUT_MS      80000u
#define ARM_SHOULDER_MAX_TRAVEL_DEG       360.0f
#define ARM_ELBOW_MAX_TRAVEL_DEG          360.0f

/* GM6020回车头零位及初始化保护。 */
#define ARM_BASE_INIT_CURRENT           10000.0f
#define ARM_BASE_INIT_TARGET_TOLERANCE_DEG  1.0f
#define ARM_SOFT_LIMIT_TARGET_SPEED_DPS     20.0f
#define ARM_SOFT_LIMIT_OVERCURRENT_MS      200u
#define ARM_SOFT_LIMIT_SETTLE_MS            300u
#define ARM_SOFT_LIMIT_TIMEOUT_MS          45000u

/*
 * 默认空间速度200mm/s，命令允许的硬上限400mm/s。局部IK若超过关节
 * 速度或加速度限制，轨迹会自动延长，不会直接削减目标位置。
 * 提高限速会缩短动作，但也会增大跟随误差、超调和结构振动。
 */
#define ARM_LINEAR_DEFAULT_SPEED_MM_S     200.0f
#define ARM_LINEAR_MAX_SPEED_MM_S         400.0f
#define ARM_LINEAR_MAX_ACCEL_MM_S2       1200.0f
#define ARM_LINEAR_Q1_MAX_SPEED_DEG_S     240.0f
#define ARM_LINEAR_Q2_MAX_SPEED_DEG_S      60.0f
#define ARM_LINEAR_Q3_MAX_SPEED_DEG_S     110.0f
#define ARM_LINEAR_Q1_MAX_ACCEL_DEG_S2    720.0f
#define ARM_LINEAR_Q2_MAX_ACCEL_DEG_S2    180.0f
#define ARM_LINEAR_Q3_MAX_ACCEL_DEG_S2    330.0f
#define ARM_LINEAR_SAMPLE_SPACING_MM        2.0f /* 只影响预检密度，不决定速度。 */
#define ARM_LINEAR_MAX_SAMPLES             384u
#define ARM_LINEAR_FK_ERROR_MAX_MM           0.5f
#define ARM_LINEAR_HOLD_MS                 1500u
#define ARM_LINEAR_IK_UPDATE_MS               5u

/* 100Hz上位机最新目标覆盖。超过100ms未刷新时冻结当前参考并保持使能。 */
#define ARM_REALTIME_COMMAND_TIMEOUT_MS      100u
#define ARM_REALTIME_DEFAULT_ACCEL_MM_S2  1200.0f
#define ARM_TRACKING_ERROR_WARN_DEG            5.0f

/* 自动测试参数只在ARM_BOOT_MODE_AUTO_TEST下生效。 */
#define ARM_AUTO_TEST_LOOP                   1u
#define ARM_AUTO_TEST_POINT_1_X_MM          56.1272f
#define ARM_AUTO_TEST_POINT_1_Y_MM          (-4.0000f)
#define ARM_AUTO_TEST_POINT_1_Z_MM         120.0365f
#define ARM_AUTO_TEST_POINT_2_X_MM         143.7071f
#define ARM_AUTO_TEST_POINT_2_Y_MM          95.7417f
#define ARM_AUTO_TEST_POINT_2_Z_MM        (-117.2108f)
#define ARM_AUTO_TEST_POINT_3_X_MM          31.8877f
#define ARM_AUTO_TEST_POINT_3_Y_MM         (-37.5448f)
#define ARM_AUTO_TEST_POINT_3_Z_MM         159.0000f

/* 3508减速比维护测试。 */
#define ARM_3508_RATIO_TEST_RATIO           19.0f
#define ARM_3508_RATIO_TEST_OUTPUT_DEG       90.0f
#define ARM_3508_RATIO_TEST_DIRECTION         1.0f
#define ARM_3508_RATIO_TEST_SPEED_DPS        342.0f
#define ARM_3508_RATIO_TEST_SETTLE_MS       2000u
#define ARM_3508_RATIO_TEST_TIMEOUT_MS     30000u
#define ARM_3508_RATIO_TEST_STALL_MS         200u
#define ARM_3508_RATIO_TEST_OVERRUN_DEG       90.0f

/*
 * M3508重力前馈维护辨识。固定q1=0、q3=-95，依次扫描6个已确认安全的
 * q2姿态；每点稳定500ms后采样1s。结果只写RAM，由Watch确认后再固化。
 */
#define ARM_FF_IDENT_POINT_COUNT               6u
#define ARM_FF_IDENT_Q1_DEG                    0.0f
#define ARM_FF_IDENT_Q3_DEG                  (-95.0f)
#define ARM_FF_IDENT_SETTLE_TOL_DEG             1.5f
#define ARM_FF_IDENT_SETTLE_SPEED_DEG_S         3.0f
#define ARM_FF_IDENT_SETTLE_MS                 500u
#define ARM_FF_IDENT_FF_OFF_SETTLE_MS          300u
#define ARM_FF_IDENT_SAMPLE_MS                1000u
#define ARM_FF_IDENT_MOVE_TIMEOUT_MS         8000u
#define ARM_FF_IDENT_MIN_RESIDUAL_CURRENT     150.0f
#define ARM_FF_IDENT_RESIDUAL_RATIO             0.15f
#define ARM_FF_IDENT_VERIFY_DEGRADE_RATIO        1.20f

/*
 * PWM舵机预留：PE9/TIM1_CH1/50Hz。机构零位和末端长度未确认，默认关闭。
 * 关闭时不启动PWM，也不写TIM1比较寄存器。
 */
#ifndef ARM_WRIST_ENABLE
#define ARM_WRIST_ENABLE                      0u
#endif
#ifndef ARM_TOOL_MODEL_ENABLE
#define ARM_TOOL_MODEL_ENABLE                 0u
#endif
#define ARM_WRIST_PWM_MIN_US                1000u
#define ARM_WRIST_PWM_MID_US                1500u
#define ARM_WRIST_PWM_MAX_US                2000u
#define ARM_WRIST_MIN_ANGLE_DEG             (-90.0f)
#define ARM_WRIST_MAX_ANGLE_DEG               90.0f
#define ARM_WRIST_ZERO_OFFSET_DEG              0.0f
#define ARM_WRIST_DIRECTION                    1.0f
#define ARM_TOOL_LENGTH_MM                      0.0f

#endif
