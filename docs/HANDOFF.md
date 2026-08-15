# vison 工程交接文档

更新时间：2026-08-15
工程目录：`C:\Users\11737\Desktop\vison`
Keil 工程：`Engineer\MDK-ARM\Engineer.uvprojx`
当前分支：`fruit`

实时源码始终优先于本文。继续工作前先查看 `git status`、`app_config.h`、
当前任务调用链和相关 Watch；不得回退用户已有修改。

## 1. 当前默认行为

`APP_MODE=APP_MODE_MG995_TEST`。当前只运行两只MG995摄像头水平0deg测试：

```text
右侧MG995：PI6 / TIM8_CH2 / 1500 us / 舵机90 deg / 摄像头0 deg
左侧MG995：PI7 / TIM8_CH3 / 1500 us / 舵机90 deg / 摄像头0 deg
机械臂、底盘、IMU、USB业务：不初始化、不提交动作
```

MG995由 `mg995_servo.c/.h` 控制；`g_mg995_servo_debug.state` 应为
`MG995_SERVO_STATE_READY`，`initialized=1`；左右摄像头角度均为`0 deg`，
两侧均为`pulse_us=1500/angle_deg=90`。
MG995必须使用独立5~6V大电流供电并与STM32共地。

机械臂侧向夹爪左右镜像交替抓放代码保留；切回
`APP_MODE_ARM_POSTURE_TEST` 后固定先抓左侧，再抓右侧并持续循环：

```text
左侧：夹爪中心=[0,340,-150] -> [0,480,-150] mm
右侧：夹爪中心=[0,-340,-150] -> [0,-480,-150] mm
两侧夹爪世界绝对俯仰=-5 deg
请求速度=100 mm/s
左抓取/A左放置 -> 返回前方 -> 右抓取/A右放置 -> 返回前方 -> 重复
```

每轮抓取预对准均通过 `AppArmFlowBuildPickStaging()` 生成同一组准备语义：
`q1=+/-89.5 deg`、`q2=80 deg`、`q3=-90 deg`、ID1相对俯仰`-80 deg`。
左/右放置回到 `q1=0` 后，下一侧必须重新提交这条四轴联合命令；只复制
反馈姿态并覆盖 `q1` 会保留释放后的 `q2/q3=[120,-80] deg`，并在下一侧
工具中心直线的首个采样点触发前方栏框保护。

Watch 主要使用 `g_app_arm_posture_test_debug`。该模式不运行底盘任务、
不调用 `AppFruitTask()`，但会闭合ID2并按当前侧显式复用A左/A右放置profile。
`active_side=1/2`分别表示左/右；另有总计和左右完成次数。正式点1/点2及
带底盘任务表未修改，恢复 `APP_MODE_ARM` 后继续使用。

完整单侧动作已经迁入 `app_arm_side_pick_place.c/.h`：调用
`AppArmSidePickPlaceStart(APP_FRUIT_SIDE_LEFT, now_ms)` 会完成左侧准备、
接近、推进、抓取、A左释放和回正；RIGHT同理使用镜像路径和A右profile。
接口为非阻塞命令，1ms机械臂应用任务必须持续调用
`AppArmSidePickPlacePoll(now_ms)`。单侧成功后停在 `DONE`，不会由模块自行
切换另一侧；当前持续交替仅由 `app_runtime.c` 在收到 `DONE` 后提交下一侧。
运行中重复提交返回 `BUSY`，非法侧别返回 `INVALID_SIDE`，明确故障锁存
`FAILED`。新版协议尚未定义左右侧完整抓放命令，因此该接口当前仍只供专项
测试调用；后续增加侧别消息时应直接调用它，不复制内部状态机。

完整上电初始化顺序为 `q2大臂+q3小臂同步 -> q1底座 -> ID1/ID2`。三台
达妙会先使能并原位保持；q2/q3由同一联合位姿命令驱动，耦合补偿仍生效。
观察 `g_arm_dm_debug.auto_init`：`step=1, axis=4` 表示大臂/小臂同步，
`step=2, axis=1` 表示底座；进入 `ARM_BOOT_WAIT_TOOL` 后才初始化ID1/ID2。

A区共8个水果，沿行进方向4组、每组左右各1个。启动区边界到第一组和
相邻各组之间均为 `500 mm`；`85 mm` 只是启动时车头到机械臂中心的
一次性安装补偿，因此首段 `585=500+85`，后续两段均为 `500 mm`。

任务、底盘或机械臂出现明确故障后锁存失败，原位停止，不自动重试、
不跳过当前点。每段底盘必须完成停车稳定确认后才允许机械臂动作；每次
放置必须回到前方后才允许下一段底盘运动。

## 2. 当前分层

| 文件 | 当前职责 |
|---|---|
| `app_runtime.c` | 初始化和FreeRTOS包装入口 |
| `MODULE/servo/mg995_servo.c/.h` | TIM8双路MG995角度/PWM控制和Watch |
| `app_fruit_task.c/.h` | A区静态任务表、区域/侧别/点位和调度失败锁存 |
| `app_fruit_task_config.h` | A区距离、工具中心抓取点和左右放置参数 |
| `app_arm_flow.c/.h` | 单次工具中心坐标抓取和显式profile放置 |
| `app_arm_side_pick_place.c/.h` | 可供上位机调度的LEFT/RIGHT单侧完整抓放命令 |
| `app_arm_command_id.c/.h` | 固件内部机械臂命令ID的唯一分配器 |
| `arm/` | 主臂、工具、轨迹、运动学和安全保护 |
| `chassis/chassis.c/.h` | 通用相对运动命令执行器 |
| `chassis/chassis_config.h` | 已验证方向、机械参数、PID和停车边界 |
| `fruit_usb_bridge.c/.h` | 水果识别观察；当前不驱动静态任务 |
| `upper_controller_bridge.c/.h` | 新版速度、夹爪、摄像头和ArmTarget协议适配及Watch |

完整 A 区语义和扩展步骤见 `docs/FRUIT_TASK_FLOW.md`。

## 3. A 区机械臂数据

坐标为 `+X` 车头、`+Y` 物理左侧、`-Y` 物理右侧、`+Z` 向上。

- 业务点1：工具中心 `[0,400,-100] mm`，世界绝对俯仰 `-90 deg`。
- 业务点2：工具中心 `[0,-400,-100] mm`，世界绝对俯仰 `-90 deg`。

两组坐标直接参与工具中心IK；点1对应 `q1` 约 `+90 deg`，点2对应约
`-90 deg`，当前解析关节参考分别为 `[90,32.86,-101.44] deg` 和
`[-90,32.86,-101.44] deg`。

点1安全位姿 `[90,90,-100]`，经 `+135` 到 `+180`；点2安全位姿
`[-90,90,-100]`，经 `-135` 到 `-180`。两侧释放均使用
`q2=120`、`q3=-70`、ID1相对小臂 `-45 deg` 释放；ID2张开完成后保持
`q2=120`，小臂再上抬10deg到 `q3=-80`，随后分别经 `+90/-90` 返回
`q1=0` 并结束放置。下一抓预对准时，ID1与三台达妙同步进入
`[q2,q3]=[80,-90]`、ID1相对俯仰 `-80 deg`；随后工具中心轨迹以
`200 mm/s` 请求速度运行。该准备姿态避免关节到位误差把相对角推过
`-90 deg` 软件下限。定向转到后方以后，
释放与抬臂命令保留底座实际反馈角，不再重复提交 `q1=+180/-180`。

放置流程只读取 `App_Arm_Place_Profile_s`，不读取上次抓取目标，也不按
抓取 `q1` 正负推断侧别。安全位姿显式提交三轴，释放与释放后净空仅更新
`q2/q3` 并保留实时 `q1`；返回前方后不再提交冗余恢复位姿。
B/C/D 目前未配置，禁止缺省复用 A 区 profile。

## 4. 夹爪堵转容错

ID2 默认/张开 `450`，探测闭合 `660`。接触后每次回退 `10`，最多
4 次。任一次到位稳定后完成；4 次耗尽时进入
`ARM_GRIPPER_FORCED_HELD`，记录状态但按抓取成功继续，避免卡死。

该容错不掩盖持续离线、通信或初始化故障。关键 Watch：

- `g_arm_tool_debug.gripper_state`
- `g_arm_tool_debug.gripper_relief_attempt_count`
- `g_arm_tool_debug.gripper_forced_held_count`
- `g_arm_tool_debug.gripper_timeout_count`
- `g_arm_tool_debug.servo_feedback_valid[1]`
- `g_arm_tool_debug.servo_online[1]`

## 5. 底盘公共接口

`ChassisInit(imu)` 只初始化并等待 IMU/电机，不自动运动。
`ChassisSubmitCommand()` 接受相对直线或相对转角；应用统一使用毫米，
正距离固定为物理车头向前。`ChassisGetStatus()` 返回命令 ID、状态、目标、
实测和故障。取消正常停稳后进入 `CANCELLED`，急停立即锁存 `FAULT`。

上位机速度桥调用 `ChassisSubmitVelocityCommand()`，每次提交严格递增的
`command_id`、`vx_mm_s` 和 `wz_rad_s`。正`vx`为物理车头前进，正`wz`为
逻辑Yaw增加。运行中的速度命令允许用新ID刷新；`wz=0`且`vx!=0`时锁定
当前IMU航向进行直行修正，非零`wz`以上位机目标为主，重新回到零`wz`
平移时捕获新的当前航向。300ms未刷新会平滑停车到`CANCELLED`，不锁存
通信故障；电机/IMU离线和急停仍进入`FAULT`。新版 `VelocityCommand` 已由
`upper_controller_bridge.c` 转换为该公共速度接口；
只有切到 `APP_MODE_HOST_CONTROL` 才初始化USB、底盘和IMU。当前默认MG995模式
仍不会初始化底盘。

速度接口初始限制为`|vx|<=200 mm/s`、`|wz|<=0.8 rad/s`。左右轮按
`vl=vx-wz*L/2`、`vr=vx+wz*L/2`换算；任一轮超过0.35m/s时两侧按相同
比例缩小，保持上位机给定的转弯曲率。

状态为：

```text
WAIT_READY / IDLE / RUNNING / STOPPING /
COMPLETED / CANCELLED / FAULT
```

重复或旧命令 ID、忙、未就绪、非法参数和故障锁存都有独立提交结果。
原 `ChassisInitOneShotStraight/StartOneShotStraight/OneShotDone` 已删除。
`APP_MODE_CHASSIS_ONE_METER` 在应用层通过同一公共接口复现原测试。

当前实测方向参数：

```c
CHASSIS_LEFT_COMMAND_SIGN  = -1.0f
CHASSIS_RIGHT_COMMAND_SIGN =  1.0f
CHASSIS_LEFT_FEEDBACK_SIGN = -1.0f
CHASSIS_RIGHT_FEEDBACK_SIGN=  1.0f
CHASSIS_IMU_YAW_SIGN       = -1.0f
```

不要凭直觉同时调整这些符号。需要重新校准时按反馈、命令、IMU顺序逐项
架空确认。

## 6. 硬件映射

| 功能 | 总线/接口 | 配置 |
|---|---|---|
| 机械臂底座 | CAN1达妙 | DM4310 ID3，Master `0x13` |
| 大臂 | CAN1达妙 | DM4340 ID2，Master `0x12` |
| 小臂 | CAN1达妙 | DM4310 ID1，Master `0x11` |
| ID1俯仰 | USART6控制板 | 9600 8N1 |
| ID2夹爪 | USART6控制板 | 9600 8N1 |
| 左主动轮 | CAN2 DJI | M3508 ID1 |
| 右主动轮 | CAN2 DJI | M3508 ID2 |
| IMU | SPI1 | BMI088 + INS/EKF |
| 上位机 | USB FS | CDC |

CAN1/CAN2 均为 1 Mbps。机械臂 HOME 为 `q=[0,90,-60] deg`，主臂连杆
为 `260/260 mm`，ID1轴心到夹爪中心为 `117 mm`。大臂与小臂的
1:1同步带耦合补偿仍在 `arm.c`，不能删除。

## 7. FreeRTOS 所有权

- `ImuTask`：INS 更新并通知底盘 IMU 新鲜度。
- `ChassisTask`：推进底盘命令状态机。
- `ArmControlTask`：推进机械臂和 A 区任务调度。
- `MotorControlTask`：唯一的达妙/DJI周期控制发送任务。
- `UsbTask`：USB、协议、蜂鸣器、Daemon以及当前模式对应的业务桥。

不要把电机周期发送重新塞回可能进行长时间路径预检的应用任务。

## 8. Watch

- `g_app_fruit_task_debug`：区域、侧别、点位、任务索引、距离和最终失败源。
- `g_app_arm_command_id_debug`：公共机械臂命令ID种子、最后发放值、发放/初始化/回绕计数。
- `g_chassis_debug`：公共状态、命令 ID、目标/实测、里程计、PID和故障。
- `g_app_arm_pick_place_test_debug`：子流程、profile、机械臂命令和预检。
- `g_arm_tool_debug`：双舵机通信、夹爪堵转、回退和强制完成。
- `g_arm_dm_debug`：三台达妙反馈、使能和控制发送。
- `g_fruit_usb_debug`：协议会话和水果识别快照。
- `g_protocol_runtime_debug`：握手、心跳、会话和可靠发送队列。
- `g_upper_controller_debug`：新版离散命令、底盘速度提交、执行回调和暂缓的ArmTarget。

机械臂邮箱只接受比上一条更新的命令ID。当前专项姿态测试和
`AppArmFlow` 已统一调用 `AppArmCommandIdNext()`；后续新增任何固件内部
机械臂命令生产者也必须复用该分配器，禁止新增模块私有基址或在流程初始化时
重置序列。底盘命令使用另一套邮箱，继续维护独立ID域。

## 9. 协议和定位边界

当前生成协议哈希为 `0x2588BA9A`，生成文件是
`protocol.c/.h/PROTOCOL_DOC.md`。`protocol_runtime.*`、
`protocol_port.*`、`fruit_usb_bridge.*`、`upper_controller_bridge.*` 是工程维护
文件，不能随生成文件一起覆盖。新版无需强制握手且心跳非严格，生成FSM自动
回心跳和入站可靠消息ACK；运行层只观察状态，不再重复回心跳。

`VelocityCommand` 已接到底盘连续速度接口；`StateMachineCommand` 已接到ID2
夹爪和双MG995，并使用 `ExecutionCallback` 报执行/完成。`ArmTarget`当前仅把
相机坐标米值换算为毫米并记录Watch，固定外参和拍照姿态关联完成前不执行。
`FruitDetection`仍只更新识别快照，不驱动静态任务表。

底盘直线距离取左右主动轮相对里程平均值，IMU 只闭环航向。
`g_chassis_debug.y_m` 是积分观察值，不参与横向闭环。后续上位机需要提供
带符号且有新鲜度判断的横向误差，通过低带宽外环修正航向；不能把累计
`y_m` 当成外部绝对位置。

## 10. 构建和验证边界

新版协议和桥接层已通过 ARM GCC 严格语法检查。当前MG995模式及临时
`APP_MODE_HOST_CONTROL`模式均通过Keil ArmCC 5全量构建，0错误0警告；
最终AXF/HEX已恢复为当前MG995默认模式。未烧录，也未进行上位机、底盘、
夹爪、摄像头或ArmTarget的实机协议验收。

当前先验收底盘保持不动、点1 `[0,400,-100]` 与点2
`[0,-400,-100]` 完整抓放并持续交替；抓取时ID1绝对俯仰应为 `-90 deg`。
恢复场地路线后按
`585 mm -> 抓1 -> 500 mm -> 抓2 -> 500 mm -> 抓3 -> DONE` 验收。

## 11. 工作区注意事项

- 当前分支为 `fruit`，保留所有用户已有未提交修改。
- 不使用 `git reset --hard`、`git checkout --` 或宽泛清理。
- PowerShell 可能以 `8009001d` 失败，出现后改用原生 `cmd.exe`。
- 实时 `app_config.h` 和源码优先于任何旧构建产物或历史说明。
