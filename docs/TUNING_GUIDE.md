# 参数位置和调参说明

更新时间：2026-08-14

修改参数后需要重新编译。底盘和机械臂首次动作应架空测试；当前已实测
方向符号不得凭直觉改动，每次只调整一个参数组。

## 运行模式

文件：`Engineer/APPLICATION/app_config.h`

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `APP_MODE` | `APP_MODE_ARM` | A区三次自动抓放测试 |
| `APP_ARM_TOOL_CENTER_TEST_ENABLE` | `1` | 启用水果任务和机械臂抓放子流程 |

其他模式为底盘一米循环、幻儿反馈专项和机械臂无力打点，一次只能选择一个。

## A 区任务参数

文件：`Engineer/APPLICATION/app_fruit_task_config.h`

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `APP_FRUIT_AREA_A_FIRST_POSITION_MM` | 500 mm | 启动区边界到A区第一组水果的纵向距离 |
| `APP_FRUIT_ARM_CENTER_BEHIND_NOSE_MM` | 85 mm | 启动时机械臂中心落后物理车头的安装偏移 |
| `APP_FRUIT_AREA_A_FIRST_MOVE_MM` | 585 mm | 首段 `500+85`，只补偿一次安装偏移 |
| `APP_FRUIT_AREA_A_GROUP_SPACING_MM` | 500 mm | A区相邻两组水果的纵向距离 |
| `APP_FRUIT_CHASSIS_TOLERANCE_MM` | 3 mm | 水果任务停车距离容差 |

A区共4组、每组左右各1个水果；当前只测试连续3组，距离为
`585/500/500 mm`。A左/A右抓取点和放置角也集中在该文件。调点时必须
保留区域和侧别语义，不要把数组下标或 `q1` 正负当成业务判断。
B/C/D 未配置，不能套用A区参数。

## 底盘通用接口和方向

接口：`Engineer/APPLICATION/chassis/chassis.h`

`Chassis_Command_s.distance_mm` 使用毫米，正值固定表示物理车头向前，负值
表示后退。`CHASSIS_COMMAND_RELATIVE_TURN` 的正角表示逻辑 Yaw 增加。

公共状态：

```text
WAIT_READY -> IDLE -> RUNNING -> STOPPING -> COMPLETED
                                      +----> CANCELLED
任意明确故障 ------------------------------> FAULT
```

重复/旧命令 ID、未就绪、忙、非法参数和故障锁存分别返回独立结果。
`COMPLETED` 只会在速度低于阈值并连续稳定后发布。

文件：`Engineer/APPLICATION/chassis/chassis_config.h`

| 参数 | 当前值 | 调整说明 |
|---|---:|---|
| `CHASSIS_LEFT_COMMAND_SIGN` | `-1.0` | 已验证左轮物理方向，不要联动猜测 |
| `CHASSIS_RIGHT_COMMAND_SIGN` | `1.0` | 已验证右轮物理方向 |
| `CHASSIS_LEFT_FEEDBACK_SIGN` | `-1.0` | 物理向前时逻辑里程应增加 |
| `CHASSIS_RIGHT_FEEDBACK_SIGN` | `1.0` | 物理向前时逻辑里程应增加 |
| `CHASSIS_IMU_YAW_SIGN` | `-1.0` | 逆时针车体旋转时逻辑Yaw增加 |
| `CHASSIS_WHEEL_RADIUS_M` | 0.0475 m | 距离整体偏小时增大，偏大时减小 |
| `CHASSIS_REDUCTION_RATIO` | 19.2032 | M3508减速比，通常不改 |
| `CHASSIS_TRACK_WIDTH_M` | 0.320 m | 轮中心距，影响轮差Yaw和转动目标 |
| `CHASSIS_TEST_MAX_SPEED_M_S` | 0.200 m/s | 当前直线最大速度 |
| `CHASSIS_TEST_MIN_SPEED_M_S` | 0.060 m/s | 终点附近最低速度 |
| `CHASSIS_TEST_DECEL_DISTANCE_M` | 0.250 m | 增大可更早减速 |
| `CHASSIS_MAX_LINEAR_ACCEL_M_S2` | 0.35 m/s2 | 增大响应和电流冲击都会增加 |
| `CHASSIS_STOP_SPEED_M_S` | 0.020 m/s | 停稳速度阈值 |
| `CHASSIS_STOP_STABLE_MS` | 300 ms | 连续停稳窗口 |

直行 PID 看 `CHASSIS_HEADING_*`，转角 PID 看 `CHASSIS_TURN_*`。D 项使用
BMI088 Z 轴角速度，不是离散 Yaw 误差差分。推荐顺序：反馈方向、命令
方向、IMU方向、有效轮径、轮距、速度环、直行P/D/I、转角P/D/I。

主要 Watch：

- `state/command_id/command_type/last_submit_result`
- `target_distance_mm/actual_distance_mm/tolerance_mm`
- `heading_target_deg/heading_error_deg/heading_pid_*`
- `target_angle_deg/actual_angle_deg/turn_pid_*`
- `left_target_m_s/right_target_m_s/left_speed_m_s/right_speed_m_s`
- `fault/motor_offline_count/imu_fault_count/direction_fault_count`

### 里程计 Y 误差

直线完成条件使用左右主动轮相对里程的平均值，IMU PID 只保持起步航向。
`g_chassis_debug.x_m/y_m` 是轮里程与 IMU 航向的积分估计，不是绝对定位；
当前 `y_m` 不参与横向闭环。两轮有效直径不一致、地面打滑、从动轮扰动
和航向零偏都会使 Y 误差累计。

后续上位机闭环时，应由相机、场地线或其他外部定位输出带符号的
`lateral_error_mm`，先滤波和限幅，再将其转换为小幅航向修正；底盘内部
仍负责轮速、航向和停车。外环丢失或数据过期时必须降级为当前航向保持，
不要直接用累计的 `y_m` 作为绝对横向误差。

## 机械臂点位和放置 profile

抓取流程通用参数在 `app_config.h`，A区实际点位/profile 在
`app_fruit_task_config.h`。当前点位：

| 点位 | `q1/q2/q3` | ID1相对小臂 | 观察坐标 |
|---|---|---|---|
| A左点1 | `[90,31.64,-111.21] deg` | `-41.5 deg` | `[0,-451.6,-73.3] mm` |
| A右点2 | `[-90,33.88,-105.85] deg` | `-41.5 deg` | `[0,431.0,-76.9] mm` |

观察坐标当前只用于 Watch，其 Y 符号与左右业务名不一致。坐标规划启用前
需要重新实机确认；不要为匹配文字左右而直接修改已经验证的关节角。

低位底座预对准由 `APP_ARM_PICK_BASE_AIM_MAX_ABS_Q1_DEG=89.5` 限制，
完整抓取命令再到 `+/-90 deg`。不要直接把预对准改为 `+/-90`，否则可能
在低位跨过 `X=0` 并触发跨区保护。

A左放置：`[90,90,-100] -> +135 -> +180 -> [180,120,-70]`；
A右放置：`[-90,90,-100] -> -135 -> -180 -> [-180,120,-70]`。
释放 ID1 相对小臂均为 `-45 deg`。ID2张开后保持 `q2=120 deg`，小臂
从 `q3=-70 deg` 再上抬10deg到 `q3=-80 deg`，随后分别经 `+90/-90`
返回 `q1=0`，最后恢复转运姿态 `[0,90,-100]`。

放置参数以完整 `App_Arm_Place_Profile_s` 提交。更改某侧时应同时核对：

- 安全三轴位姿；
- 到放置点的引导角和终点角；
- 释放三轴位姿和ID1相对俯仰；
- 释放反馈等待超时；
- ID2释放后的小臂净空位姿；
- 恢复三轴位姿；
- 返回前方的引导角和终点角。

## 主臂几何和安全

文件：`Engineer/APPLICATION/arm/arm_config.h`

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `ARM_BASE_HEIGHT_MM` | 62 mm | 安装基准到肩轴高度 |
| `ARM_LINK_1_MM` | 260 mm | 肩轴到肘轴中心距 |
| `ARM_LINK_2_MM` | 260 mm | 肘轴到ID1轴中心距 |
| `ARM_TOOL_PITCH_AXIS_TO_CENTER_MM` | 117 mm | ID1轴到夹爪中心 |
| `ARM_SAFE_Q1/Q2/Q3_DEG` | `[0,90,-60]` | HOME关节目标 |
| `ARM_Q1_SOFT_MIN/MAX_DEG` | `[-180,180]` | 关节命令软限位 |
| `ARM_Q2_SOFT_MIN/MAX_DEG` | `[25,180]` | 大臂软限位 |
| `ARM_Q3_SOFT_MIN/MAX_DEG` | `[-190,-35]` | 小臂软限位 |

连杆长度必须测转轴中心到转轴中心，不要用 HOME 坐标补偿杆长误差。
负 X、跨区高度和前方栏框限制仍由 `ARM_REAR_*`、
`ARM_FRONT_BARRIER_*` 参数控制，不要为了让单个点通过而放宽软限位。

## ID1 和 ID2

文件：`Engineer/APPLICATION/arm/arm_config.h`

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `ARM_TOOL_PITCH_NEUTRAL_POS` | 500 | ID1与小臂同向 |
| `ARM_TOOL_PITCH_DIRECTION` | `-1.0` | 目标和反馈换算共用方向 |
| `ARM_GRIPPER_DEFAULT_POS` | 550 | 上电、等待和张开位置 |
| `ARM_GRIPPER_CLOSE_POS` | 660 | 探测闭合目标 |
| `ARM_GRIPPER_RELIEF_STEP_POS` | 10 | 每次向张开方向回退量 |
| `ARM_GRIPPER_RELIEF_MAX_ATTEMPTS` | 4 | 最多回退4次，累计最多40 |
| `ARM_GRIPPER_RELIEF_ATTEMPT_TIMEOUT_MS` | 400 ms | 单次回退等待上限 |

4 次均不能跟随时进入 `ARM_GRIPPER_FORCED_HELD` 并继续业务，不再进入会
卡死采摘任务的夹爪故障。这个状态只表示“容错按抓住处理”，不证明一定
抓到水果。持续离线、通信和初始化错误仍会使机械臂流程失败。

主要 Watch：

- `g_arm_tool_debug.gripper_state`
- `g_arm_tool_debug.gripper_relief_attempt_count`
- `g_arm_tool_debug.gripper_forced_held_count`
- `g_arm_tool_debug.gripper_timeout_count`
- `g_arm_tool_debug.servo_feedback_valid[1]`
- `g_arm_tool_debug.servo_online[1]`

## 协议边界

当前生成协议哈希为 `0x0EBAB184`。生成文件为 `protocol.c/.h` 和
`PROTOCOL_DOC.md`；`protocol_runtime.*`、`protocol_port.*`、
`fruit_usb_bridge.*` 是工程维护层，更新生成文件时不能覆盖。

`FruitDetection` 当前只有 `fruit_id/status`，只更新
`g_fruit_usb_debug`，不直接改变 A 区静态测试表。现有协议没有底盘命令、
横向误差或机械臂任务字段；未来协议桥应调用相同的
`ChassisSubmitCommand/ChassisGetStatus` 边界。
