# A 区水果采摘任务流程

更新时间：2026-08-14

## 1. 规则和物理尺寸映射

《水果采摘规则》给出的场地尺寸为 `5200 mm x 3000 mm`。A 区共有
8 个水果位置，沿机器人行进方向分成 4 组，每组左右各 1 个；启动区
边界到第一组、以及相邻各组之间的纵向距离均为 `500 mm`。比赛要求
机器人全程自主运行，抓取成熟蔬菜和放入机器人存放装置分别计分。

当前实机测试只使用 A 区左右两个已教导抓取姿态。启动时机械臂中心
位于物理车头后方 `85 mm`，不可能与启动区边界重合，因此只有首段需要
在规则标称距离上叠加安装偏移：

```text
A区首个标称位置 500 mm + 机械臂中心落后车头 85 mm = 585 mm
```

到达第一组后，机械臂中心已经完成一次性对齐，后续相邻水果组之间直接
按规则前进 `500 mm`，不能再次使用或叠加 `85 mm` 安装偏移。

## 2. 当前自动测试

默认 `APP_MODE_ARM` 上电完成 HOME 后执行：

```text
底盘沿物理车头方向前进 585 mm并停稳
-> 第1次抓取并放置（当前使用A左教导姿态）
-> 底盘沿物理车头方向前进 500 mm并停稳
-> 第2次抓取并放置（当前使用A右教导姿态）
-> 底盘沿物理车头方向前进 500 mm并停稳
-> 第3次抓取并放置（当前使用A左教导姿态）
-> DONE
```

静态任务表位于 `Engineer/APPLICATION/app_fruit_task.c`。场地和机构尺寸
位于 `app_fruit_task_config.h`，使用以下独立名称：

- `APP_FRUIT_AREA_A_FIRST_POSITION_MM = 500`
- `APP_FRUIT_ARM_CENTER_BEHIND_NOSE_MM = 85`
- `APP_FRUIT_AREA_A_FIRST_MOVE_MM = 500 + 85`
- `APP_FRUIT_AREA_A_GROUP_SPACING_MM = 500`

底盘或机械臂出现明确故障后，任务锁存 `APP_FRUIT_TASK_FAILED`，原位停止，
不自动重试、不跳过点位。只有底盘完成减速和停车稳定确认并发布
`CHASSIS_STATE_COMPLETED` 后，任务层才允许开始机械臂抓取。

## 3. A 区点位

坐标约定：`+X` 为物理车头方向，`+Y` 为车体左侧，`+Z` 向上。
观察坐标只用于 Watch，不参与当前教导关节抓取规划。现有观察坐标的 Y
符号与左右业务名不一致，接入坐标规划前必须重新实机确认；当前不得根据
观察坐标改动已验证的教导关节角和左右放置路线。

| 点位 | 语义 | 教导关节 `q1/q2/q3` | ID1相对小臂 | 观察坐标 `x/y/z` |
|---|---|---|---|---|
| `APP_FRUIT_POINT_A_LEFT` | A左点1 | `[90, 31.64, -111.21] deg` | `-41.5 deg` | `[0, -451.6, -73.3] mm` |
| `APP_FRUIT_POINT_A_RIGHT` | A右点2 | `[-90, 33.88, -105.85] deg` | `-41.5 deg` | `[0, 431.0, -76.9] mm` |

抓取流程仍由 `AppArmFlowStartPick()` 执行：低位底座对准限制为
`+/-89.5 deg`，随后联合三轴和 ID1 命令到完整教导位姿，等待俯仰反馈稳定
后闭合 ID2。夹爪接触后最多分级回退 4 次；4 次均耗尽时记录
`ARM_GRIPPER_FORCED_HELD`，仍按抓取成功继续，避免任务永久卡住。

## 4. A 区放置 profile

放置流程只读取传入的 `App_Arm_Place_Profile_s`。它不读取上一次抓取目标，
也不根据抓取 `q1` 正负推断左右侧。安全、释放和恢复都是完整三轴位姿。

### A 左

```text
安全位姿       [90, 90, -100] deg
到后方路线     +90 -> +135 -> +180 deg，逆时针
释放位姿       [180, 120, -70] deg
ID1相对小臂    -45 deg
ID2释放后净空  [180, 120, -80] deg，小臂再上抬10deg
返回前方路线   +180 -> +90 -> 0 deg
前方恢复位姿   [0, 90, -100] deg
```

### A 右

```text
安全位姿       [-90, 90, -100] deg
到后方路线     -90 -> -135 -> -180 deg，顺时针
释放位姿       [-180, 120, -70] deg
ID1相对小臂    -45 deg
ID2释放后净空  [-180, 120, -80] deg，小臂再上抬10deg
返回前方路线   -180 -> -90 -> 0 deg
前方恢复位姿   [0, 90, -100] deg
```

`AppArmFlowStartPlace(profile, now_ms)` 在提交第一条动作前校验：

- `configured` 和非零 `profile_id`；
- 安全、释放、释放后净空、恢复四组关节角均有限且位于软件限位；
- 到放置点和返回前方的引导角、终点角均有效；
- ID1 相对俯仰位于工具限位；
- 释放俯仰等待超时非零。

未配置返回 `APP_ARM_FLOW_START_NOT_CONFIGURED`，字段非法返回
`APP_ARM_FLOW_START_INVALID`，均不会开始机械动作。

## 5. 模块边界

| 层次 | 文件 | 职责 |
|---|---|---|
| 场地任务 | `app_fruit_task.c/.h` | 区域、侧别、点位、任务表、失败锁存和流程串联 |
| A区配置 | `app_fruit_task_config.h` | 物理尺寸、教导点和A左/A右放置参数 |
| 机械臂子流程 | `app_arm_flow.c/.h` | 单次教导抓取和显式profile放置 |
| 底盘执行器 | `chassis/chassis.c/.h` | 相对直线/转角、里程计、IMU PID、减速和停稳 |
| 运行入口 | `app_runtime.c` | 初始化模块并在FreeRTOS包装任务中周期调用 |

当前生成协议哈希为 `0x0EBAB184`，`FruitDetection` 只有
`fruit_id/status`，只更新 `g_fruit_usb_debug`，不驱动本页静态任务表。
未来增加上位机底盘桥时，应把协议命令转换为 `Chassis_Command_s` 并读取
`Chassis_Status_s`，不能绕过底盘执行器直接设置电机目标。

## 6. 新增区域流程

B/C/D 当前全部视为未配置，不允许回退使用 A 区安全点或释放角度。新增
区域时必须依次完成：

1. 按明确坐标系重新教导抓取点并记录区域、侧别和点位 ID。
2. 单独教导安全位姿、到放置点路线、释放位姿、恢复位姿和返回路线。
3. 用关节软限位和工作区安全规则完成静态预检。
4. 新建独立 `App_Arm_Place_Profile_s`，不得复制后只改名称。
5. 先架空验证方向，再进行低速实机验证。
6. 验证通过后才允许把点位/profile加入任务表。

## 7. Watch 和实机验收

任务总览看 `g_app_fruit_task_debug`：

- `state/area/side/point_id/task_index/completed_count`
- `place_profile_id`
- `chassis_command_id/chassis_target_distance_mm/chassis_actual_distance_mm`
- `arm_active_flow/arm_pick_step/arm_place_step`
- `failure_source/failure_code`

底盘细节看 `g_chassis_debug`，机械臂子流程看
`g_app_arm_pick_place_test_debug`，夹爪强制抓取看
`g_arm_tool_debug.gripper_forced_held_count` 和
`g_arm_tool_debug.gripper_relief_attempt_count`。

实机依次确认 `585 mm`、第1次抓放、`500 mm`、第2次抓放、`500 mm`、
第3次抓放以及最终保持 `DONE`。当前教导姿态顺序仍为A左/A右/A左。
本次源码整理未编译、未烧录、未进行硬件验证。
