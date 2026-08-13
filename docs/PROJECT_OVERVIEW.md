# 当前工程功能

更新时间：2026-08-13

## 当前默认运行内容

`Engineer/APPLICATION/app_config.h` 当前为：

```c
#define APP_MODE APP_MODE_ARM
#define APP_ARM_TOOL_CENTER_TEST_ENABLE 1u
```

上电初始化 USB、协议、蜂鸣器、BMI088、双 M3508、三台达妙和两台幻儿
舵机；机械臂完成 HOME、底盘完成 IMU/电机就绪后，自动执行：

```text
585 mm -> 第1次抓放(A左姿态) -> 500 mm -> 第2次抓放(A右姿态)
-> 500 mm -> 第3次抓放(A左姿态) -> DONE
```

正底盘距离始终表示物理车头向前。每段必须完成减速和停车稳定确认后才
开始机械臂动作。任何底盘或机械臂明确故障都会锁存任务失败，不重试、
不跳过当前点。完整说明见 `docs/FRUIT_TASK_FLOW.md`。

## 软件分层

| 模块 | 职责 |
|---|---|
| `app_runtime.c` | 模式初始化和FreeRTOS任务包装，不保存水果路线 |
| `app_fruit_task.c/.h` | A区任务表、点位语义、流程调度和任务Watch |
| `app_fruit_task_config.h` | A区距离、抓取点和左右放置参数 |
| `app_arm_flow.c/.h` | 单次教导抓取和显式profile放置 |
| `arm/` | 主臂、轨迹、运动学、工具和安全控制 |
| `chassis/` | 通用相对运动命令、里程计、IMU闭环和停稳 |
| `fruit_usb_bridge.c/.h` | 水果识别结果校验和观察，不直接驱动任务 |

底盘公共接口为 `ChassisInit`、`ChassisSubmitCommand`、
`ChassisGetStatus`、`ChassisCancelMotion` 和 `ChassisEmergencyStop`。
命令/状态边界已为未来上位机预留，本轮没有修改 USB 协议包或协议哈希。

## 硬件分配

| 功能 | 接口 | 配置 |
|---|---|---|
| 左主动轮 | CAN2 | M3508 ID1，反馈 `0x201` |
| 右主动轮 | CAN2 | M3508 ID2，反馈 `0x202` |
| 机械臂底座 | CAN1 | DM4310 ID3 |
| 机械臂大臂 | CAN1 | DM4340 ID2 |
| 机械臂小臂 | CAN1 | DM4310 ID1 |
| 末端俯仰 | USART6控制板 | 幻儿 ID1 |
| 夹爪开合 | USART6控制板 | 幻儿 ID2 |
| 姿态传感器 | SPI1 | BMI088 + INS/EKF |
| 上位机 | USB FS | CDC虚拟串口 |

## 坐标和当前点位

机械臂坐标为 `+X` 车头、`+Y` 车体左侧、`+Z` 向上。当前 HOME 为
`q=[0,90,-60] deg`。ID1 输出轴中心到夹爪中心长度为 `117 mm`。

- A左点1：`q=[90,31.64,-111.21] deg`，ID1相对小臂 `-41.5 deg`。
- A右点2：`q=[-90,33.88,-105.85] deg`，ID1相对小臂 `-41.5 deg`。

放置流程由显式 `App_Arm_Place_Profile_s` 决定。A左从左侧逆时针转到
后方，A右从右侧顺时针转到后方；底层不再根据抓取关节角猜测侧别。

## 夹爪容错

ID2 默认/张开位置为 `550`，探测闭合目标为 `660`。接触后每次向张开
方向回退 `10`，最多 4 次。任一次到位稳定后完成抓取；4 次均不能跟随
时记录 `ARM_GRIPPER_FORCED_HELD`，仍按抓取成功继续，避免任务卡死。
持续离线、通信错误和初始化故障仍按真实故障处理。

## 运行模式

| 模式 | 作用 |
|---|---|
| `APP_MODE_ARM` | 当前默认，执行A区三次自动抓放测试 |
| `APP_MODE_CHASSIS_ONE_METER` | 应用层通过公共底盘接口复现1m/右转90/1m循环 |
| `APP_MODE_HUANER_FEEDBACK` | 幻儿舵机反馈专项模式 |
| `APP_MODE_ARM_TEACH_POINT` | 三达妙和双舵机无力、保留反馈和坐标观察 |

## 常用 Watch

- `g_app_fruit_task_debug`：当前区域、侧别、点位、任务索引、距离和唯一失败来源。
- `g_chassis_debug`：公共状态、命令ID、目标/实测距离、故障、PID和轮速。
- `g_app_arm_pick_place_test_debug`：抓取/放置步骤、profile、命令和预检结果。
- `g_arm_tool_debug`：ID1/ID2通信、堵转、回退和强制抓取记录。
- `g_arm_dm_debug`：三台达妙反馈、使能和发送状态。
- `g_fruit_usb_debug`：水果识别快照；当前不直接控制任务。

## 验证边界

本次整理只执行源码静态检查和 `git diff --check`，没有运行 Keil/GCC
编译，没有烧录，也没有替代实机验收。实时源码始终优先于文档。
