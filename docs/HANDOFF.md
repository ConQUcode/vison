# vison 工程详细交接文档

更新时间：2026-08-13  
工程目录：`C:\Users\11737\Desktop\vison`  
Keil 工程：`Engineer\MDK-ARM\Engineer.uvprojx`  
当前 Git 分支：`fruit`  
当前提交基线：`d1135f72c7859c6d261f9aafa99edf8d9ecbb7f4`

这份文档用于开启新的 Codex 对话。新对话应先读取本文件，再读取当前源码和 `git status`；本文件描述的是交接时状态，但实时源码始终优先。

## 1. 用户习惯和协作要求

- 使用简体中文沟通。确认类问题先直接回答结论，再给源码位置或原因。
- 嵌入式问题必须基于当前工程核对，不要仅凭文件名、旧文档或之前的描述推断运行链。
- 修改前先查看实时 `APP_MODE`、任务调用链、相关配置和 Watch 结构。
- 用户重视实机反馈，通常采用“修改一个小目标 -> 用户烧录测试 -> 根据 Watch/现象继续”的方式。
- 进行即时硬件联调时，应尽量缩小活动链路，关闭不相关模块，提供少而关键的 Watch 字段。
- 不要擅自改变电机 ID、总线、机械零点、坐标方向、安全限位或已验证的动作方向。
- 不要回退用户已有修改。当前工作区非常脏，大量修改、重命名、删除和构建产物清理都属于现有工作。
- 禁止使用 `git reset --hard`、`git checkout --` 等破坏性回退命令，除非用户明确要求。
- 非计划模式下不要运行 Keil/GCC 全量构建，用户认为反复构建浪费时间。此时只做必要的源码检查和 `git diff --check`。
- 计划模式可以构建，但应在一批修改完成后的关键验收点集中构建一次，不要对同一批源码连续重复构建。
- 不自动刷写固件。构建、刷写、硬件验证是否真正执行，交接时必须明确说明。
- Windows PowerShell 在本机可能以 `8009001d` 启动失败。出现该问题后直接改用 `cmd.exe`，不要重复运行同一条失败命令。
- 搜索优先使用 `rg`。Windows `cmd` 中带 `|` 的正则容易被 shell 当作管道，应拆成多个查询或妥善处理引号。
- 手工修改文件使用 `apply_patch`，保留无关的未提交修改。
- 自研公共函数、关键状态机和可调参数应有中文注释；Watch 变量优先紧凑、易懂。

## 2. 当前最重要状态

当前默认模式在 `Engineer/APPLICATION/app_config.h`：

```c
#define APP_MODE APP_MODE_ARM_TEACH_POINT
```

即“无力打点模式”，不是完整抓放模式，也不是底盘模式。

当前期望行为：

1. 注册 CAN1 上三台达妙，读取反馈。
2. 初始化时分别向三台达妙发送一次失能命令。
3. 打点模式不运行 `DMMotorControl()`，三台达妙持续无力，可手动拖动。
4. 初始化 USART6 幻儿舵机控制板。
5. 向 ID1、ID2 发送一次多舵机卸载命令 `0x14`，让两个舵机失去保持力矩。
6. 卸载帧发送完成后，继续用 `0x15` 轮询 ID1、ID2 位置反馈。
7. Watch 持续显示三台达妙角度、两台舵机角度、ID1 输出轴中心和 117 mm 偏移后的夹爪中心。
8. 不执行 HOME，不运行抓放循环，不初始化 BMI088 或底盘。

用户已经确认：三台达妙在打点模式下确实保持失能。用户随后发现两台幻儿舵机仍有力，因此刚完成了 `0x14` 卸载支持。该舵机卸载修改目前只完成源码实现和静态检查，尚未编译、刷写或实机确认。

## 3. 当前正在进行的工作

### 3.1 幻儿舵机无力但保留反馈

控制板协议中的命令：

```text
CMD_MULT_SERVO_UNLOAD = 20 = 0x14
```

同时卸载 ID1 和 ID2 的帧：

```text
55 55 05 14 02 01 02
```

位置查询命令仍为 `0x15`。打点模式的顺序现在是：

```text
HuanerServoInit
-> HuanerServoUnload({1,2}, 2)
-> 等待DMA发送结束和控制板发送间隔
-> 开启ID1/ID2交替位置轮询
-> 持续计算角度和末端坐标
```

修改位置：

- `Engineer/MODULE/servo/huaner_servo.h`
  - 新增 `HUANER_SERVO_COMMAND_UNLOAD = 20`。
  - 新增 `HuanerServoUnload(const uint8_t *ids, uint8_t count)`。
- `Engineer/MODULE/servo/huaner_servo.c`
  - 新增无应答卸载事务。
  - 新增卸载帧生成器。
  - 协议自测精确比较 `55 55 05 14 02 01 02`。
- `Engineer/APPLICATION/arm/arm_tool.c`
  - `ArmToolInitFeedbackOnly()` 不再立即开启位置轮询。
  - `ArmToolProcessFeedbackOnlyUnload()` 先发送一次卸载，成功结束后才开启轮询。
  - 反馈模式直接返回，不推进正常夹爪、BOOT 或位置目标状态机。
- `Engineer/APPLICATION/arm/arm_tool.h`
  - 增加卸载过程调试字段。
- `Engineer/APPLICATION/app_runtime.c/.h`
  - 将关键卸载字段汇总到 `g_app_arm_teach_debug`。

需要注意：`0x14` 按协议没有应答。`servo_unload_done=1` 只表示 DMA 发送正常结束且位置轮询已经开启，不等同于控制板主动确认卸载。是否真正无力需要手动转动舵机确认；是否仍有反馈看 `servo_online[]`。

### 3.2 下一次硬件测试步骤

1. 编译并由用户刷写当前打点模式固件。
2. 上电后观察 `g_app_arm_teach_debug`。
3. 确认 `dm_all_disabled=1`，三项 `dm_enabled[]` 均为 0。
4. 确认 `servo_unload_requested=1`。
5. 确认 `servo_unload_done=1`。
6. 确认 `servo_unload_count=1`、`servo_unload_fail_count=0`。
7. 手动转动 ID1、ID2，确认都没有保持力矩。
8. 转动过程中确认 `servo_online[0..1]=1` 且 `servo_position[]/servo_angle_deg[]` 持续变化。
9. 确认 `wrist_center_mm[]` 和 `tool_center_mm[]` 随关节变化更新。

如果 `servo_unload_done=1` 但仍有力，优先用逻辑分析仪检查 USART6 是否实际发出上述 7 字节帧，并核对接入的是控制板串口而不是舵机直连协议口。不要改用旧 `HuanerServoStop()`，旧 STOP 接口在当前控制板协议中不代表卸载。

## 4. 硬件和总线映射

| 功能 | 硬件/接口 | 当前配置 |
|---|---|---|
| MCU | STM32F407IGH6 | FreeRTOS，Keil ARMCC 5.06u7 |
| 机械臂底座 | CAN1 达妙 DM4310 | Motor ID 3，Master ID `0x13`，命令 ID `0x103` |
| 机械臂大臂 | CAN1 达妙 DM4340 | Motor ID 2，Master ID `0x12`，命令 ID `0x102` |
| 机械臂小臂 | CAN1 达妙 DM4310 | Motor ID 1，Master ID `0x11`，命令 ID `0x101` |
| 末端俯仰 | USART6 控制板 | 幻儿舵机 ID1 |
| 夹爪开合 | USART6 控制板 | 幻儿舵机 ID2 |
| 左主动轮 | CAN2 DJI M3508 | ID1，反馈 `0x201`，控制帧 `0x200` 第一槽 |
| 右主动轮 | CAN2 DJI M3508 | ID2，反馈 `0x202`，控制帧 `0x200` 第二槽 |
| IMU | SPI1 | BMI088 + INS/EKF |
| 上位机 | USB FS | CDC，不使用 UART 波特率 |

CAN1 和 CAN2 在 `.ioc` 中均为 1 Mbps。USART6 当前为 `9600 8N1`、异步收发模式；驱动宏 `HUANER_SERVO_CONTROLLER_BAUD_RATE` 也是 9600。不要因为协议文档中的其他链路写了 115200 就修改 USART6。

## 5. 工程运行架构

### 5.1 应用层入口

`main.c` 在外设初始化完成后调用 `AppInit()`。应用入口位于：

- `Engineer/APPLICATION/app_runtime.c`
- `Engineer/APPLICATION/app_runtime.h`
- `Engineer/APPLICATION/app_config.h`

四个模式：

| 模式 | 值 | 作用 |
|---|---:|---|
| `APP_MODE_ARM` | 0 | 完整机械臂、双舵机、USB/水果观察和抓放测试 |
| `APP_MODE_CHASSIS_ONE_METER` | 1 | BMI088、双 M3508、里程计和直行/右转循环 |
| `APP_MODE_HUANER_FEEDBACK` | 2 | 独立舵机反馈专项模式 |
| `APP_MODE_ARM_TEACH_POINT` | 3 | 当前模式：三达妙与双舵机无力，保留反馈和坐标计算 |

`APP_ARM_TOOL_CENTER_TEST_ENABLE=1` 只在 `APP_MODE_ARM` 下生效，在当前打点模式不会运行抓放循环。

### 5.2 FreeRTOS 任务

当前 CubeMX 实际有五个任务，不是旧文档中的四个：

| 任务 | 优先级/栈 | 包装入口 | 当前打点模式行为 |
|---|---|---|---|
| `ImuTask` | AboveNormal / 512 | `AppImuTask()` | 空操作，不访问未初始化 BMI088 |
| `ChassisTask` | AboveNormal / 512 | `AppChassisTask()` | 空操作，不访问底盘电机 |
| `UsbTask` | AboveNormal / 512 | `AppUsbTask()` | 当前模式为空操作 |
| `ArmControlTask` | AboveNormal / 512 | `AppArmTask()` | 运行 `ArmTask()`，刷新被动反馈、工具反馈和 FK |
| `MotorControlTask` | High / 256 | `AppMotorControlTask()` | 不运行达妙控制；`DJIMotorControl()` 在零实例时为空操作 |

`MotorControlTask` 是后来为解决机械臂长时间路径预检期间控制帧中断而新增的 1 kHz 唯一电机发送任务。完整机械臂模式下 `DMMotorControl()` 在这里运行，不要再搬回 `ArmControlTask`。底盘模式下 `DJIMotorControl()` 也由这里运行。

USB 设备初始化位于 `UsbTask_f()` 循环前，只执行一次。`Catch_Task` 已从 CubeMX 和运行链删除。

### 5.3 目录职责

| 路径 | 作用 |
|---|---|
| `Engineer/APPLICATION/app_runtime.*` | 运行模式、初始化和 FreeRTOS 包装入口 |
| `Engineer/APPLICATION/arm/` | 机械臂状态机、运动学、轨迹、工具层和安全规划 |
| `Engineer/APPLICATION/chassis/` | 双轮差速底盘、里程计、IMU闭环和循环测试 |
| `Engineer/APPLICATION/fruit_usb_bridge.*` | 水果识别结果校验和 Watch 快照 |
| `Engineer/MODULE/motor/DMmotor/` | 达妙 CAN 驱动与控制发送 |
| `Engineer/MODULE/motor/DJImotor/` | M3508 驱动、速度/电流闭环 |
| `Engineer/MODULE/servo/` | 幻儿控制板 USART6 异步驱动 |
| `Engineer/MODULE/imu/` | BMI088、INS 和 EKF |
| `Engineer/MODULE/protocol/` | 生成协议、会话 Runtime 和 USB Port 适配 |
| `Engineer/BSP/` | CAN、USART、DWT 等板级封装 |
| `Engineer/Core/` | CubeMX 生成代码和 FreeRTOS 任务入口 |
| `docs/` | 工程概览、调参说明和本交接文档 |
| `release/` | 已归档的可回退固件及 SHA-256 |

## 6. 机械臂架构和坐标约定

### 6.1 三自由度主臂

- `q1`：底座偏航。
- `q2`：大臂角度。
- `q3`：小臂机械关节角。
- 大臂与小臂间有同步带机械耦合，换算仍在 `arm.c` 中，不能删除。
- 主臂基础 FK/IK 的末端是 ID1 俯仰舵机输出轴中心，名称为 `WRIST_CENTER`。
- 主臂基础几何在 `arm_config.h`：底座高度 62 mm，大臂 260 mm，小臂 260 mm，肩部前后/左右偏移均为 0。
- 逻辑 `+X` 由底座物理朝向和达妙上位机保存的底座零点决定。固件不主动发送达妙清零命令。
- HOME 逻辑目标为 `q1=0°、q2=90°、q3=-60°`；对应大臂物理约 `-90°`。

当前主要运动接口保留：

- 关节运动。
- 轴心笛卡尔运动。
- 夹爪中心笛卡尔运动。
- 直达、直线和实时目标。
- HOME、取消、急停、故障复位、温度和反馈超时保护。
- 命令邮箱、command ID 去重和主机状态。

### 6.2 ID1 和夹爪中心

ID1 控制末端相对小臂的俯仰。当前实机方向：

```c
#define ARM_TOOL_PITCH_DIRECTION (-1.0f)
```

ID1 参数：

- 中位 500：夹爪方向与小臂方向一致。
- 软件范围 125..875。
- 相对角范围 -90°..+90°。
- 控制值增大时，夹爪相对小臂向下。

夹爪中心到 ID1 输出轴中心的实测长度为 117 mm，沿夹爪绝对俯仰方向参与坐标换算：

```c
#define ARM_TOOL_PITCH_AXIS_TO_CENTER_MM 117.0f
```

因此工程同时保留两套末端：

- `WRIST_CENTER`：ID1 输出轴中心，用于主臂杆长、零位和机械重装标定。
- `TOOL_CENTER`：真实夹爪中心，用于业务目标和实际精度测量。

公共接口包括：

- `ArmForwardKinematicsToolCenter()`。
- `ArmInverseKinematicsToolCenter()`。
- `ArmInverseKinematicsToolCenterAll()`。
- `ArmSubmitToolCenterCommand()`。
- `ArmToolGetCenterFromWrist()` / `ArmToolGetWristFromCenter()`。

工具中心 IK 同时检查目标方位和反向方位，支持负径向分支；轨迹预检会保留每个采样点的全部合法候选，并用动态规划选择整条连续关节路径。

### 6.3 ID2 夹爪和堵转

当前 ID2 配置：

- 软件范围 550..660。
- 默认、等待、打开和释放位置均为 550。
- 抓取探测闭合目标为 660。
- 正常到达 660 且稳定后进入 `CLOSED_EMPTY`，仍允许业务进入下一步。
- 闭合途中位置长时间不变化会判定接触，随后每次向张开方向回退 10，最多回退 5 次。
- 任一次回退到位并稳定后进入 `HELD_CONTACT`，同样允许业务进入下一步。
- 五次回退仍不能跟随才进入 `JAMMED/FAULT`。
- 电压只用于供电健康观察，不参与堵转判断；当前没有电流、温度或真实夹持力反馈。

`HELD_CONTACT` 只表示位置运动受到阻挡，不能区分真实水果、机构摩擦或卡滞；固件不能声称检测到夹持力或 `GRIP_LOST`。

### 6.4 工作区和防干涉

安全判断统一以夹爪中心为准：

- `X<0` 的目标和路径要求工具中心 `Z>=160 mm`。
- 跨越 `X=0` 时规划高度为 210 mm。
- 实际高度达到 205 mm 才允许进入跨区水平段。
- 跨区运行中低于 200 mm 会中止。
- 工具中心仍在前方 `X>2 mm` 且底座仍朝前时，大臂 `q2` 不得超过 120°，防止向栏框方向后倾。
- 进入后方区域后，该前方栏框附加限制不再应用，但关节自身软限位仍有效。

路径预检失败时，完整机械臂模式优先看：

```text
g_app_arm_pick_place_test_debug.preflight_failed_segment
g_app_arm_pick_place_test_debug.preflight_failed_sample
g_app_arm_pick_place_test_debug.preflight_failed_check_mask
g_app_arm_pick_place_test_debug.preflight_failed_center_mm
g_app_arm_pick_place_test_debug.preflight_failed_q_deg
```

`preflight_failed_check_mask` 位 0..6 分别对应：IK、FK误差、关节限位、自动区、ID1俯仰、工作区、相邻关节跳变。

## 7. 完整抓放测试源码状态

完整抓放状态机仍保留，但当前模式不会运行。切换为 `APP_MODE_ARM` 后，`APP_ARM_TOOL_CENTER_TEST_ENABLE=1` 会启用它。

当前测试大意：

1. 完成三台达妙、双舵机和 HOME 初始化。
2. 抓取姿态要求夹爪世界水平。
3. 以夹爪中心移动到 `(250,0,200) mm`。
4. 等待 1 s，ID2 执行抓取到 660；堵转回退成功或正常到位都继续。
5. 再等待 1 s，进入固定安全转移姿态。
6. 底座通过 `0 -> +90 -> +180°` 连续转到后方。
7. 大臂到 `q2=150°`，小臂到 `q3=-45°`，释放前 ID1 相对小臂向下 45°。
8. ID2 打开到 550。
9. 恢复小臂、大臂、底座和抓取俯仰，返回抓取点继续循环。

该流程经历过多轮实机调整，方向和阶段不要凭直觉重写。修改前先查看 `App_Arm_Pick_Place_Test_State_e`、`AppArmPickPlaceTestTask()` 和 `g_app_arm_pick_place_test_debug`。

## 8. 底盘架构和测试状态

底盘为双 M3508 差速结构：左 CAN2 ID1，右 CAN2 ID2。当前配置：

- 轮半径 0.0475 m，即直径 95 mm。
- 减速比 19.2032。
- 轮距暂定 0.320 m，最终需要实测。
- 左命令/反馈符号为 `+1/+1`。
- 右命令/反馈符号为 `-1/-1`。
- IMU Yaw 符号当前为 `+1`，逆时针逻辑 Yaw 应增加。
- 底盘应用 5 ms 更新，电机闭环由高优先级任务 1 kHz 服务。
- 平移来自双轮编码器；航向以连续 IMU Yaw 为主，轮差 Yaw 用于诊断。

切换 `APP_MODE_CHASSIS_ONE_METER` 后自动循环：

```text
直行1m -> 停稳等待3s -> 右转90° -> 停稳等待3s
-> 直行1m -> 停稳等待3s -> 重新循环
```

直行和转弯均为 PID。参数位于 `Engineer/APPLICATION/chassis/chassis_config.h`。底盘实机效果已被用户评价为“不错”，但后续仍可能继续增大 IMU 航向补偿。切换模式前应再次确认机械问题已经处理，且机械臂/夹爪不会动作。

## 9. 上位机协议

当前生成协议文件：

- `Engineer/MODULE/protocol/protocol.c`
- `Engineer/MODULE/protocol/protocol.h`
- `Engineer/MODULE/protocol/PROTOCOL_DOC.md`

STM32 专用适配：

- `protocol_runtime.c/.h`：握手、心跳、可靠消息和会话状态。
- `protocol_port.c/.h`：USB 发送适配。
- `fruit_usb_bridge.c/.h`：水果数据值域检查和 Watch。

当前协议：

| ID | 消息 | 说明 |
|---:|---|---|
| `0xFD` | Ack | 框架 ACK |
| `0xFE` | Heartbeat | 严格心跳，3 s 超时 |
| `0xFF` | Handshake | 哈希握手 |
| `0x10` | FruitDetection | `{fruit_id:u8,status:u8}`，非可靠 |

协议哈希为 `0x923FFDD9`，帧头 `0x5A 0xA5`，CRC8 多项式 `0x31`。只有匹配握手后水果数据才进入应用；心跳超时或 USB 断开后保留原值用于诊断，但 `valid=0`。

水果合法 ID 为 1..6，成熟度 0..1；`0,0` 表示无目标。当前水果结果只更新 `g_fruit_usb_debug`，不会触发机械臂、夹爪或底盘动作。

用户以后可能用上位机生成的新 `protocol.c/.h/PROTOCOL_DOC.md` 直接覆盖这三个生成文件。因此不要把 STM32 会话逻辑写回生成文件，新业务放在独立桥或 Runtime 中。

注意：协议文档写的 115200 是生成的串口协议元数据，但当前工程上位机实际走 USB CDC；它与 USART6 幻儿控制板的 9600 无关。

## 10. 关键 Watch 变量

### 10.1 当前无力打点模式

优先只看：

```c
g_app_arm_teach_debug
```

关键字段：

- `feedback_ready`：三台达妙、两台舵机反馈有效且舵机卸载完成。
- `dm_all_disabled`：三台达妙是否全部失能。
- `dm_online[3]` / `dm_enabled[3]`。
- `dm_joint_deg[3]` / `dm_feedback_age_ms[3]`。
- `servo_online[2]`。
- `servo_position[2]` / `servo_angle_deg[2]`。
- `servo_unload_requested`。
- `servo_unload_done`。
- `servo_unload_count`，正常应为 1。
- `servo_unload_fail_count`，正常应为 0。
- `tool_pitch_deg`。
- `wrist_center_valid` / `wrist_center_mm[3]`。
- `tool_center_valid` / `tool_center_mm[3]`。

### 10.2 完整机械臂模式

- `g_app_arm_pick_place_test_debug`：抓放阶段、命令、失败原因、坐标误差和路径预检。
- `g_arm_dm_debug`：三达妙 CAN、在线、使能、反馈年龄和启动状态。
- `g_arm_kinematics_debug`：ID1 轴心、夹爪中心、117 mm 工具长度和绝对俯仰。
- `g_arm_servo_angle_debug`：两台舵机通信、当前角和目标角。
- `g_arm_gripper_stall_debug`：ID2 堵转、回退次数、是否允许进入下一阶段。
- `g_arm_tool_debug`：完整工具层状态，只有深挖问题时再展开。
- `g_huaner_servo_driver_debug`：USART6 事务、实际收发帧、超时和恢复计数。

### 10.3 底盘和协议

- `g_chassis_debug`：状态、故障、双轮距离、IMU/轮差 Yaw、PID 分项、轮速目标和循环数。
- `g_protocol_runtime_debug`：握手、心跳、超时、发送和重试。
- `g_fruit_usb_debug`：水果 ID、成熟度、有效性、更新时间和非法计数。

## 11. 参数位置

### 11.1 整机模式和抓放流程

文件：`Engineer/APPLICATION/app_config.h`

- `APP_MODE`：当前运行模式。
- `APP_ARM_TOOL_CENTER_TEST_ENABLE`：完整机械臂模式下是否运行抓放测试。
- 抓取点 `(250,0,200)`、速度、俯仰、等待时间。
- 固定转移/释放关节角和底座旋转方向。

### 11.2 机械臂和夹爪

文件：`Engineer/APPLICATION/arm/arm_config.h`

- 达妙 ID、Master ID、命令 ID。
- 电机零位、逻辑零点、方向和同步带耦合。
- `ARM_BASE_HEIGHT_MM`、`ARM_LINK_1_MM`、`ARM_LINK_2_MM`。
- 关节软限位、自动运行限位和速度。
- 工具中心长度 117 mm。
- ID1 方向、限位、发送周期。
- ID2 默认 550、闭合 660、堵转窗口和最多五次回退。
- 负 X 高度、跨区高度和前方栏框 `q2=120°` 限制。

### 11.3 底盘

文件：`Engineer/APPLICATION/chassis/chassis_config.h`

- 左右命令和反馈方向。
- 轮径、减速比和轮距。
- IMU Yaw 方向。
- 直行/转弯 PID。
- 测试距离、速度、加速度、停稳和等待时间。
- M3508 速度/电流环及电流上限。

更完整的参数说明见 `docs/TUNING_GUIDE.md`，但其中“当前默认模式”等描述可能落后于本文件和实时 `app_config.h`。

## 12. 构建、验证和回退边界

Keil 工程：

```text
Engineer/MDK-ARM/Engineer.uvprojx
```

已有辅助脚本：

- `Engineer/MDK-ARM/build_cleanup.cmd`：全量 Keil 构建，并将 ARMCC 临时目录放在项目内。
- `Engineer/MDK-ARM/gcc_arm_check.cmd`：GNU 严格语法检查。

当前刚完成的舵机卸载修改只运行了：

```text
git diff --check -- <相关文件>
```

结果无空白错误，仅有 Git 的 LF/CRLF 未来转换提示。本次没有运行 Keil/GCC 构建，没有刷写，也没有硬件确认。

历史上工具中心路径、动态规划和机械臂相关修改曾完成 Keil ARMCC `0 Error / 0 Warning` 验证，但当前工作区在此后又有新增修改，所以不能把历史构建结果当作当前源码构建结果。

已确认可回退的机械臂初始化固件：

```text
release/Engineer_arm_init_ok_20260811.hex
SHA-256: 267947ddf27bd7f0a0bc9bf42a77ea8603ef2b70972615b4afba76c2173ae7ca
```

除非用户明确要求，不要自动恢复或刷写该固件。

## 13. 工作区注意事项

- 当前分支为 `fruit`，工作区有大量未提交修改。
- `Test.c/.h` 已整理/重命名为 `app_runtime.c/.h`。
- 旧 `catch`、`remote`、`relay`、`C_comm`、`nac`、`robot_def`、`arm_wrist`、`arm_usb_bridge` 和飞特兼容代码已删除或退出工程。
- 舵机驱动已从旧 `feite_motor/hsl_servo` 迁移为 `MODULE/servo/huaner_servo.*`。
- 大量 Keil 构建产物和 VS Code 数据库显示为删除，属于清理工作，不要恢复。
- 当前还有未跟踪的 `docs/`、`release/`、`tools/`、协议 Runtime/Port 等有效文件，不要误删。
- `git status` 还会显示工作区外观异常的路径项；不要做宽泛清理，除非用户明确指定目标。
- 旧 `docs/PROJECT_OVERVIEW.md`、`docs/TUNING_GUIDE.md` 和根 `README.md` 的默认模式描述可能仍写着完整机械臂，应以本文件和实时 `app_config.h` 为准。

## 14. 新对话建议开场

可以在新对话中直接发送：

```text
请先读取 C:\Users\11737\Desktop\vison\docs\HANDOFF.md，
再核对当前 app_config.h、git status 和相关源码。不要回退现有修改。
当前正在测试无力打点模式：三台达妙已经确认失能，下一步验证
幻儿控制板 0x14 是否让 ID1/ID2 无力但继续保留 0x15 位置反馈。
非计划模式不要编译，不要自动刷写。
```

## 15. 下一步优先级

1. 按第 3.2 节验证 ID1/ID2 卸载和反馈能否同时成立。
2. 如果成功，确认无力拖动时 `wrist_center_mm` 和 `tool_center_mm` 是否符合实际方向和尺寸。
3. 如果失败，先看 `servo_unload_*` 和 `g_huaner_servo_driver_debug.tx_frame`，再决定是否抓 USART6 波形。
4. 打点和标定完成后，再由用户决定切回完整机械臂抓放模式还是底盘测试模式。
5. 切换模式只改 `APP_MODE`，不要同时改其他已验证参数。
