# 当前工程功能

## 当前默认运行内容

当前 `APP_MODE` 为 `APP_MODE_ARM`。上电后运行完整机械臂初始化和抓放循环测试：

1. 初始化 DWT、USB 软件队列、协议、蜂鸣器、三台达妙和两台幻儿舵机。
2. 先完成舵机反馈确认、三台达妙使能和机械臂自动 HOME。
3. HOME 后主臂保持不动，先单独把 ID1 的世界绝对俯仰调到 `0°`。
4. 俯仰反馈稳定后，以夹爪中心为控制点直线移动到抓取点 `(250,0,200) mm`。
5. 到位等待 `5 s`，ID2闭合到 `640`；提前受阻或无阻挡正常到位都视为抓取动作完成，随后再等待 `3 s`。
6. 移动到放置点 `(-22,0,230) mm`，ID2打开后返回抓取点并循环。
7. 任一步预检、通信或动作失败后原位保持，不自动重试。

默认模式不会初始化 BMI088、两台底盘 M3508，也不会运行底盘测试。

## 硬件分配

| 功能 | 接口 | 配置 |
|---|---|---|
| 左主动轮 | CAN2 | M3508 ID1，反馈 `0x201` |
| 右主动轮 | CAN2 | M3508 ID2，反馈 `0x202` |
| 机械臂底座 | CAN1 | DM4310 ID3 |
| 机械臂大臂 | CAN1 | DM4340 ID2 |
| 机械臂小臂 | CAN1 | DM4310 ID1 |
| 末端俯仰 | USART6 控制板 | 幻儿舵机 ID1 |
| 夹爪开合 | USART6 控制板 | 幻儿舵机 ID2 |
| 姿态传感器 | SPI1 | BMI088 |
| 上位机 | USB FS | CDC 虚拟串口 |

## FreeRTOS 任务

| 任务 | 作用 |
|---|---|
| `ImuTask` | 1 kHz IMU读取和姿态解算 |
| `ChassisTask` | 1 kHz DJI电机服务，内部5 ms底盘控制和里程计 |
| `UsbTask` | USB收发、协议、蜂鸣器和Daemon |
| `ArmControlTask` | 1 kHz运行机械臂和达妙控制，并推进抓放循环测试 |

`Catch_Task` 已删除。

## 保留的机械臂功能

- 三台达妙的使能、反馈、HOME、限位和故障处理。
- 三自由度 FK/IK 和关节/笛卡尔轨迹。
- 大臂与小臂同步带耦合换算。
- ID1 绝对俯仰闭环和 ID2 夹爪闭环/位置堵转判断。
- 当前 HOME 为 `q1=0°、q2=90°、q3=-60°`。
- 主臂三自由度 FK/IK 继续输出 ID1 俯仰舵机输出轴中心，作为机械重装、杆长和电机零位标定基准。
- 夹爪中心由 ID1 输出轴中心沿夹爪绝对俯仰方向偏移 `117 mm` 得到；正式业务名称为 `ARM_CONTROL_POINT_TOOL_CENTER`，旧 `TOOL_TIP` 只是兼容别名。
- 工具中心 IK 同时检查目标方位和反向方位，因此负 X 可以使用不超出底座限位的负径向解。
- 负 X 目标至少高 `160 mm`；跨越 `X=0` 时自动先抬到 `210 mm`，实测高度达到 `205 mm` 才允许水平跨越，运行中低于 `200 mm` 会中止。
- 夹爪中心处于前方 `X>=0` 时，大臂内部 `q2` 不得超过 `120°`，防止大臂后倾碰到前方栏框；进入 `X<0` 后方区域后取消该附加限制，恢复原 `180°` 软件上限。
- 当前抓放测试的抓取点和放置点都使用 `TOOL_CENTER`，测量点应放在真实夹爪中心。

## 上位机通信

生成文件为 `protocol.c`、`protocol.h` 和 `PROTOCOL_DOC.md`，可以由上位机新版本直接覆盖。STM32 自己的握手、心跳和 USB 适配位于 `protocol_runtime.*`、`protocol_port.*`。

当前业务消息只有水果识别。识别结果只更新 `g_fruit_usb_debug`，不会控制机械臂、夹爪或底盘。

## 目录说明

| 目录 | 内容 |
|---|---|
| `Engineer/APPLICATION` | 整机模式、底盘、机械臂和水果业务 |
| `Engineer/MODULE/motor` | DJI和达妙电机驱动 |
| `Engineer/MODULE/servo` | 幻儿舵机控制板驱动 |
| `Engineer/MODULE/imu` | BMI088和INS/EKF |
| `Engineer/MODULE/protocol` | 生成协议、会话Runtime和USB端口 |
| `Engineer/BSP` | CAN、USART和DWT底层封装 |
| `release` | 已确认固件归档和SHA-256 |

## 常用 Watch

- `g_chassis_debug`：循环状态、故障、分段距离、连续Yaw、直行/转弯PID和轮速目标。
- `g_app_arm_tool_center_test_debug`：水平俯仰、工具中心目标/反馈、反算轴心/关节角和工作区安全结果。
- `g_arm_kinematics_debug`：同时查看 ID1 输出轴中心、夹爪中心、工具长度和夹爪绝对俯仰。
- `g_huaner_servo_driver_debug`：幻儿底层通信事务和电压反馈。
- `g_arm_servo_angle_debug`：机械臂模式下两个舵机在线、当前和目标角度。
- `g_arm_dm_debug`：三台达妙反馈、使能和发送状态。
- `g_fruit_usb_debug`：握手、心跳和水果识别快照。
- `g_protocol_runtime_debug`：协议会话、心跳和可靠重试。
