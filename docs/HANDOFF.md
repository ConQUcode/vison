# vison 工程交接文档

更新时间：2026-08-14
工程目录：`C:\Users\11737\Desktop\vison`
Keil 工程：`Engineer\MDK-ARM\Engineer.uvprojx`
当前分支：`fruit`

实时源码始终优先于本文。继续工作前先查看 `git status`、`app_config.h`、
当前任务调用链和相关 Watch；不得回退用户已有修改。

## 1. 当前默认行为

`APP_MODE=APP_MODE_ARM`，上电完成机械臂 HOME 和底盘就绪后自动运行：

```text
物理向前585 mm -> 第1次抓放（当前A左姿态）
-> 物理向前500 mm -> 第2次抓放（当前A右姿态）
-> 物理向前500 mm -> 第3次抓放（当前A左姿态）
-> DONE
```

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
| `app_fruit_task.c/.h` | A区静态任务表、区域/侧别/点位和调度失败锁存 |
| `app_fruit_task_config.h` | A区距离、教导点和左右放置参数 |
| `app_arm_flow.c/.h` | 单次教导抓取和显式profile放置 |
| `arm/` | 主臂、工具、轨迹、运动学和安全保护 |
| `chassis/chassis.c/.h` | 通用相对运动命令执行器 |
| `chassis/chassis_config.h` | 已验证方向、机械参数、PID和停车边界 |
| `fruit_usb_bridge.c/.h` | 水果识别观察；当前不驱动静态任务 |

完整 A 区语义和扩展步骤见 `docs/FRUIT_TASK_FLOW.md`。

## 3. A 区机械臂数据

坐标为 `+X` 车头、`+Y` 左侧、`+Z` 向上。

- A左点1：`q=[90,31.64,-111.21] deg`，ID1相对小臂 `-41.5 deg`，
  观察坐标 `[0,-451.6,-73.3] mm`。
- A右点2：`q=[-90,33.88,-105.85] deg`，ID1相对小臂 `-41.5 deg`，
  观察坐标 `[0,431.0,-76.9] mm`。

两组观察坐标当前只用于 Watch，其 Y 符号与左右业务名不一致。接入坐标
规划前必须重新实机确认，不能为匹配文字左右而改动已验证的关节姿态。

A左安全位姿 `[90,90,-100]`，经 `+135` 逆时针到 `+180`；A右安全
位姿 `[-90,90,-100]`，经 `-135` 顺时针到 `-180`。两侧释放均使用
`q2=120`、`q3=-70`、ID1相对小臂 `-45 deg` 释放；ID2张开完成后保持
`q2=120`，小臂再上抬10deg到 `q3=-80`，随后分别经 `+90/-90` 返回
`q1=0`，最后在前方恢复转运姿态 `[0,90,-100]`。

放置流程只读取 `App_Arm_Place_Profile_s`，不读取上次抓取目标，也不按
抓取 `q1` 正负推断侧别。安全、释放、释放后净空和恢复命令均显式包含
三轴目标。
B/C/D 目前未配置，禁止缺省复用 A 区 profile。

## 4. 夹爪堵转容错

ID2 默认/张开 `550`，探测闭合 `660`。接触后每次回退 `10`，最多
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
- `UsbTask`：USB、协议、蜂鸣器、Daemon和水果观察桥。

不要把电机周期发送重新塞回可能进行长时间路径预检的应用任务。

## 8. Watch

- `g_app_fruit_task_debug`：区域、侧别、点位、任务索引、距离和最终失败源。
- `g_chassis_debug`：公共状态、命令 ID、目标/实测、里程计、PID和故障。
- `g_app_arm_pick_place_test_debug`：子流程、profile、机械臂命令和预检。
- `g_arm_tool_debug`：双舵机通信、夹爪堵转、回退和强制完成。
- `g_arm_dm_debug`：三台达妙反馈、使能和控制发送。
- `g_fruit_usb_debug`：协议会话和水果识别快照。
- `g_protocol_runtime_debug`：握手、心跳、会话和可靠发送队列。

## 9. 协议和定位边界

当前生成协议哈希为 `0x0EBAB184`，生成文件是
`protocol.c/.h/PROTOCOL_DOC.md`。`protocol_runtime.*`、
`protocol_port.*`、`fruit_usb_bridge.*` 是工程维护文件，不能随生成文件
一起覆盖。`FruitDetection` 仅包含 `fruit_id/status`，目前只更新 Watch，
不驱动静态任务表，也没有底盘命令、横向误差或机械臂任务字段。

底盘直线距离取左右主动轮相对里程平均值，IMU 只闭环航向。
`g_chassis_debug.y_m` 是积分观察值，不参与横向闭环。后续上位机需要提供
带符号且有新鲜度判断的横向误差，通过低带宽外环修正航向；不能把累计
`y_m` 当成外部绝对位置。

## 10. 构建和验证边界

本次整理按用户要求没有运行 Keil/GCC 编译、没有烧录、没有硬件测试。
只执行 `git diff --check`、符号引用和旧接口/旧距离残留搜索。

实机验收顺序：`585 mm -> 抓1 -> 500 mm -> 抓2 -> 500 mm -> 抓3 -> DONE`。
当前三次机械臂教导姿态顺序仍为A左、A右、A左。
任一阶段失败时，检查 `g_app_fruit_task_debug.failure_source/failure_code`
确定唯一上层来源，再下钻底盘或机械臂 Watch。

## 11. 工作区注意事项

- 当前分支为 `fruit`，保留所有用户已有未提交修改。
- 不使用 `git reset --hard`、`git checkout --` 或宽泛清理。
- PowerShell 可能以 `8009001d` 失败，出现后改用原生 `cmd.exe`。
- 实时 `app_config.h` 和源码优先于任何旧构建产物或历史说明。
