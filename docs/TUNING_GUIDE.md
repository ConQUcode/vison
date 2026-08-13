# 参数位置和调参说明

修改参数后必须重新编译。轮子和机械臂第一次动作均应架空测试；方向参数没有实测确认前不要提高速度、电流或加速度。

## 运行模式

文件：`Engineer/APPLICATION/app_config.h`

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `APP_MODE` | `APP_MODE_ARM` | 当前运行完整机械臂和单次坐标精度测试 |

可切换为 `APP_MODE_ARM`、`APP_MODE_CHASSIS_ONE_METER` 或 `APP_MODE_HUANER_FEEDBACK`。一次只能选择一种模式。

## 底盘直行、右转和循环测试

文件：`Engineer/APPLICATION/chassis/chassis_config.h`

| 参数 | 当前值 | 单位 | 调整说明 |
|---|---:|---:|---|
| `CHASSIS_WHEEL_RADIUS_M` | 0.0475 | m | 有效轮径；距离整体偏小时增大，偏大时减小 |
| `CHASSIS_REDUCTION_RATIO` | 19.2032 | 比值 | M3508减速比，通常不改 |
| `CHASSIS_TRACK_WIDTH_M` | 0.320 | m | 左右主动轮中心距，需实测；主要影响轮式Yaw |
| `CHASSIS_LEFT_COMMAND_SIGN` | +1 | - | 左轮目标方向 |
| `CHASSIS_RIGHT_COMMAND_SIGN` | -1 | - | 右轮目标方向 |
| `CHASSIS_LEFT_FEEDBACK_SIGN` | +1 | - | 左轮向前时距离必须增加 |
| `CHASSIS_RIGHT_FEEDBACK_SIGN` | -1 | - | 右轮向前时距离必须增加 |
| `CHASSIS_IMU_YAW_SIGN` | +1 | - | 逆时针转车时逻辑Yaw应增加，方向不对只改这一项 |
| `CHASSIS_TEST_DISTANCE_M` | 1.000 | m | 每一段直行的目标距离 |
| `CHASSIS_TEST_MAX_SPEED_M_S` | 0.200 | m/s | 首次测试不要提高 |
| `CHASSIS_TEST_MIN_SPEED_M_S` | 0.060 | m/s | 接近终点时的最低运行速度 |
| `CHASSIS_TEST_DECEL_DISTANCE_M` | 0.250 | m | 增大可更早减速，停车更平缓 |
| `CHASSIS_MAX_LINEAR_ACCEL_M_S2` | 0.35 | m/s² | 增大响应更快，但电流冲击更大 |
| `CHASSIS_ACTION_WAIT_MS` | 3000 | ms | 每次直行或转弯停稳后的等待时间 |
| `CHASSIS_HEADING_KP_RAD_S_PER_DEG` | 0.150 | rad/s/deg | 直行P；增大纠偏更强，过大会左右摆动 |
| `CHASSIS_HEADING_KI_RAD_S_PER_DEG_S` | 0.020 | rad/s/(deg*s) | 直行I；消除长期小偏差，过大会慢速来回摆动 |
| `CHASSIS_HEADING_KD` | 0.150 | - | 直行D；使用IMU角速度抑制摆动 |
| `CHASSIS_HEADING_INTEGRAL_LIMIT_DEG_S` | 12 | deg*s | 直行积分限幅，防止积分累积过大 |
| `CHASSIS_HEADING_MAX_OUTPUT_RAD_S` | 1.00 | rad/s | 直行纠偏角速度上限 |
| `CHASSIS_TURN_ANGLE_DEG` | -90 | deg | 右转为负角度 |
| `CHASSIS_TURN_KP_RAD_S_PER_DEG` | 0.035 | rad/s/deg | 转角P，决定接近目标的速度 |
| `CHASSIS_TURN_KI_RAD_S_PER_DEG_S` | 0.003 | rad/s/(deg*s) | 转角I，只在15度误差内积分 |
| `CHASSIS_TURN_KD` | 0.180 | - | 转角D，使用IMU角速度抑制过冲 |
| `CHASSIS_TURN_MAX_RATE_RAD_S` | 0.80 | rad/s | 原地转弯最大角速度 |
| `CHASSIS_TURN_ERROR_TOLERANCE_DEG` | 1.5 | deg | 转角到位误差阈值 |
| `CHASSIS_MOTOR_CURRENT_LIMIT` | 10000 | 驱动值 | 未完成架空测试前不要提高 |

完整PID中的D项不是对离散Yaw误差直接差分，而是读取BMI088的Z轴角速度，噪声更小。积分只在小误差区工作，带积分限幅；输出饱和时会撤销本周期积分。

底盘调参顺序：反馈方向、命令方向、IMU方向、有效轮径、轮距、电机速度环、直行P、直行D、直行I、转角P、转角D、转角I。每次只改一项。

测试时优先观察 `g_chassis_debug`：

- `state`：4为第一段直行，7为右转，10为第二段直行，其他动作态为停车或等待。
- `segment_distance_m`：当前直行段距离，每段开始时清零。
- `heading_pid_p/i/d_rad_s`：直行PID三项输出。
- `turn_target_yaw_deg/turn_error_deg`：连续Yaw转角目标和剩余误差。
- `turn_pid_p/i/d_rad_s`：转弯PID三项输出。
- `cycle_count`：完成整套动作的次数。

## 机械臂

文件：`Engineer/APPLICATION/arm/arm_config.h`

| 参数组 | 主要参数 | 说明 |
|---|---|---|
| 几何 | `ARM_BASE_HEIGHT_MM`、`ARM_LINK_1_MM`、`ARM_LINK_2_MM` | FK/IK尺寸，必须按转轴中心测量 |
| 电机零位 | `*_LOGICAL_ZERO_DEG`、`*_DIRECTION` | 固件坐标映射；底座物理+X应先在达妙上位机保存为零点 |
| 耦合 | `ARM_ELBOW_SHOULDER_COUPLING` | 大臂与小臂同步带补偿，当前为1:1 |
| 软件限位 | `ARM_Q1/Q2/Q3_SOFT_MIN/MAX_DEG` | 正常轨迹边界，不要用脱困边界替代 |
| HOME | `ARM_SAFE_Q1/Q2/Q3_DEG`、`ARM_USB_HOME_*` | 当前为 `[0,90,-60]` 和约 `(225.1666,0,192)` mm |
| 轨迹 | `ARM_LINEAR_MAX_SPEED_MM_S`、`ARM_LINEAR_MAX_ACCEL_MM_S2` | 增大可提速，但会增加跟踪误差和冲击 |
| 到位 | `ARM_ARRIVAL_ERROR_DEG`、`ARM_ARRIVAL_SPEED_DEG_S` | 减小会更严格，也更容易等待超时 |
| 温度 | `ARM_TEMPERATURE_HOLD_C`、`ARM_TEMPERATURE_DISABLE_C` | 当前70°C保持、80°C禁用 |

机械长度必须在本文件组对应的 `arm_config.h` 修改：

- `ARM_BASE_HEIGHT_MM`：底座安装基准平面到大臂肩轴中心的垂直高度，当前 `62.0 mm`。
- `ARM_LINK_1_MM`：大臂肩轴中心到小臂肘轴中心的中心距，当前 `260.0 mm`。
- `ARM_LINK_2_MM`：小臂肘轴中心到 ID1 俯仰舵机输出轴中心的中心距，当前 `260.0 mm`。
- `ARM_SHOULDER_OFFSET_FORWARD_MM`：底座旋转轴到肩轴中心沿逻辑 `+X` 的偏移，当前 `0.0 mm`。
- `ARM_SHOULDER_OFFSET_LEFT_MM`：底座旋转轴到肩轴中心沿逻辑 `+Y` 的偏移，当前 `0.0 mm`。
- `ARM_TOOL_PITCH_AXIS_TO_CENTER_MM`：ID1输出轴中心到夹爪中心的距离，当前约测为 `117.0 mm`；已经进入夹爪中心正解、逆解和 `TOOL_CENTER` 轨迹预检。

前三项连杆长度应测量“转轴中心到转轴中心”，不要测外壳边缘，也不要通过修改 `ARM_USB_HOME_X/Y/Z_MM` 补偿杆长误差。工具长度应沿夹爪中心线测量“ID1输出轴中心到实际夹爪中心”。修改长度后必须重新编译，并先架空检查 HOME、软件限位和目标点 IK。

当前有两套明确坐标：

- `ARM_CONTROL_POINT_WRIST_CENTER`：ID1输出轴中心，供主臂精度标定和机械结构重装；主臂基础 FK/IK 始终保留这一基准。
- `ARM_CONTROL_POINT_TOOL_CENTER`：夹爪中心；固件按工具长度和绝对俯仰角换算后，再使用同一套主臂 FK/IK。`TOOL_TIP` 仅为旧源码兼容名。

当前自动测试使用 `TOOL_CENTER`：抓取点 `(250,0,200) mm`，放置点 `(-22,0,230) mm`，俯仰 `0°`。应用参数位于 `Engineer/APPLICATION/app_config.h` 的 `APP_ARM_TOOL_CENTER_TEST_*`、`APP_ARM_PLACE_CENTER_*` 和等待时间宏。

负 X 安全参数也位于 `arm_config.h`：

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `ARM_REAR_ZONE_MIN_TOOL_Z_MM` | 160 mm | 负X目标和路径最低夹爪中心高度 |
| `ARM_FRONT_BARRIER_SHOULDER_Q2_MAX_DEG` | 120 deg | 实机确认的大臂防栏框硬边界；高位跨区也不豁免。底座保持朝前时，当前负X释放点无法在该边界内连续到达 |
| `ARM_FRONT_BARRIER_TOOL_X_MARGIN_MM` | 2 mm | 正X前方栏框区域的反馈噪声容差；工具中心必须先绕到X接近0或负X，才允许大臂超过120deg |
| `ARM_REAR_CROSSING_TOOL_Z_MM` | 210 mm | 跨越X=0时的规划高度 |
| `ARM_REAR_CROSSING_ACTUAL_GATE_Z_MM` | 205 mm | 水平跨区前的实际反馈放行高度 |
| `ARM_REAR_CROSSING_ABORT_Z_MM` | 200 mm | 水平跨区运行时的中止保护线 |
| `ARM_LINEAR_MAX_SAMPLES` | 1536 | 全路径预检最大样本数；约25KB缓存在CCM RAM |

不要为了让负 X 近原点位置可达而放宽关节限位。当前机构和限位会形成不可达空区，必须先用工具中心 IK 检查具体抓取点和放置点。

## ID1 俯仰和 ID2 夹爪

文件：`Engineer/APPLICATION/arm/arm_config.h`

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `ARM_TOOL_PITCH_NEUTRAL_POS` | 500 | ID1与小臂同向 |
| `ARM_TOOL_PITCH_DIRECTION` | -1.0 | ID1实机安装方向；目标换算和反馈反算共用，不能只修改一侧 |
| `ARM_TOOL_PITCH_SERVO_MIN_POS` | 125 | 当前安装方向下对应相对小臂 +90° |
| `ARM_TOOL_PITCH_SERVO_MAX_POS` | 875 | 当前安装方向下对应相对小臂 -90° |
| `ARM_TOOL_PITCH_AXIS_TO_CENTER_MM` | 117 mm | ID1轴心到夹爪中心，已用于工具中心正/逆坐标换算 |
| `ARM_GRIPPER_DEFAULT_POS` | 550 | ID2上电、等待和张开位置 |
| `ARM_GRIPPER_CLOSE_POS` | 660 | ID2探测闭合目标；正常到位或提前受阻完成卸力后都会继续放置流程 |
| `ARM_GRIPPER_CLOSE_ARRIVAL_ERROR_POS` | 5 | ID2闭合专用到位误差；不要放大到通用舵机的15，否则会掩盖顶压 |
| `ARM_GRIPPER_STALL_ERROR_POS` | 6 | 剩余误差达到该值才允许形成停滞判定，与闭合到位窗口连续且不重叠 |
| `ARM_GRIPPER_RELIEF_STEP_POS` | 10 | 每次分级卸力的回退量 |
| `ARM_GRIPPER_RELIEF_MAX_ATTEMPTS` | 5 | 最大分级回退次数，累计最多回退50；任一次到位稳定后立即停止 |
| `ARM_GRIPPER_RELIEF_ATTEMPT_TIMEOUT_MS` | 400 | 单次回退未跟随目标时，开始下一次回退前的等待时间 |
| `ARM_GRIPPER_STALL_START_IGNORE_MS` | 300 ms | 闭合开始后的忽略期 |
| `ARM_GRIPPER_STALL_ERROR_POS` | 20 | 距目标至少还有该误差才判断停滞 |
| `ARM_GRIPPER_STALL_WINDOW_MS` | 300 ms | 停滞观察窗口 |
| `ARM_GRIPPER_STALL_MAX_POSITION_SPAN` | 3 | 窗口内位置变化不超过该值才是停滞候选 |

位置停滞只能说明动作受到阻挡，不能证明抓到了物体，也不能代替电流或力传感器。

堵转专项调试只看 `g_arm_gripper_stall_debug`：`stall_detected`表示已判定停滞，`relief_attempt_count`表示当前已下发的回退次数，`next_stage_ready=1`表示工具层已允许进入下一阶段，`target_deg/current_deg`表示ID2目标和反馈角度。

- `stall_detected`：本次ID2动作是否已正式判定停滞，下一次夹爪动作开始时清零。
- `relief_attempt_count`：本次动作已经下发的分级回退次数，范围为0..5。
- `next_stage_ready`：工具层是否已经完成当前动作；为1时允许抓放状态机进入后续阶段。
- `target_deg`：ID2当前目标角度；闭合控制值660对应约158.4°，回退时显示当前回退目标。
- `current_deg`：ID2反馈角度；反馈无效时为`NAN`。

当前 HOME `q=[0,90,-60]°` 时，小臂绝对俯仰约为 `-30°`，命令夹爪水平 `0°` 后 ID1 目标值预计约为 `375`。夹爪中心到达 `(250,0,200) mm` 时目标关节约为 `[0,114.43,-43.26]°`，ID1 目标值预计约为 `407`。这些值用于架空核对方向，不代替实机零位标定。

## 幻儿通信

文件：`Engineer/MODULE/servo/huaner_servo.h`

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `HUANER_SERVO_CONTROLLER_BAUD_RATE` | 9600 | 必须和CubeMX USART6一致 |
| `HUANER_SERVO_DEFAULT_POLL_PERIOD_MS` | 50 ms | 每个ID的位置反馈周期 |
| `HUANER_SERVO_FEEDBACK_STALE_MS` | 100 ms | 超过后反馈不能用于闭环 |
| `HUANER_SERVO_OFFLINE_MS` | 200 ms | 超过后标记离线 |
| `HUANER_SERVO_DEFAULT_VOLTAGE_POLL_PERIOD_MS` | 500 ms | 控制板电压读取周期 |
| `HUANER_SERVO_ARRIVAL_TOLERANCE_POS` | 15 | 舵机到位误差窗口 |

不要用控制板电压判断堵转。当前协议没有舵机电流和温度反馈。

## 协议

文件：`Engineer/MODULE/protocol/protocol_runtime.c`

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `PROTOCOL_HEARTBEAT_TIMEOUT_MS` | 3000 ms | 超时后会话和旧水果结果失效 |
| `PROTOCOL_RETRY_INTERVAL_MS` | 100 ms | 可靠消息重发间隔 |
| `PROTOCOL_MAX_RETRIES` | 3 | 可靠消息最大重试次数 |

这三个参数属于 STM32 Runtime，不会被上位机生成的三个协议文件覆盖。
