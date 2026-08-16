# A 区水果采摘任务流程

更新时间：2026-08-14

## 1. 规则和物理尺寸映射

《水果采摘规则》给出的场地尺寸为 `5200 mm x 3000 mm`。A 区共有
8 个水果位置，沿机器人行进方向分成 4 组，每组左右各 1 个；启动区
边界到第一组、以及相邻各组之间的纵向距离均为 `500 mm`。比赛要求
机器人全程自主运行，抓取成熟蔬菜和放入机器人存放装置分别计分。

当前实机测试只使用 A 区左右两个工具中心抓取点。启动时机械臂中心
位于物理车头后方 `85 mm`，不可能与启动区边界重合，因此只有首段需要
在规则标称距离上叠加安装偏移：

```text
A区首个标称位置 500 mm + 机械臂中心落后车头 85 mm = 585 mm
```

到达第一组后，机械臂中心已经完成一次性对齐，后续相邻水果组之间直接
按规则前进 `500 mm`，不能再次使用或叠加 `85 mm` 安装偏移。

## 2. 保留的原地交替测试

当前固件临时选择 `APP_MODE_MG995_TEST` 测试双MG995到90deg，因此本页
机械臂任务表、抓放流程和底盘调度均不运行。切回
`APP_MODE_ARM_POSTURE_TEST` 后，专项测试在原地先执行左侧
`[0,340,-150] -> [0,480,-150] mm`，再执行只对Y取负的右侧
`[0,-340,-150] -> [0,-480,-150] mm`，分别复用A左/A右放置profile，
返回前方后持续交替。本节任务表仍完整保留，恢复 `APP_MODE_ARM` 后生效。

专项测试和正式抓取共用 `AppArmFlowBuildPickStaging()`。每侧抓取开始时，
一条联合命令同时提交 `q1=+/-89.5 deg`、`q2=80 deg`、`q3=-90 deg` 和
ID1相对俯仰`-80 deg`；放置回到前方后的下一侧同样执行完整准备动作，
不得沿用释放后的 `q2/q3` 而只旋转底座。

选择 `APP_MODE_ARM` 后，上电完成 HOME，且底盘与 IMU 报告就绪时执行：

```text
底盘保持原地
-> 业务点1完整抓取、安全转运、释放并返回前方
-> 业务点2完整抓取、安全转运、释放并返回前方
-> 回到点1并无限交替
```

当前 `APP_FRUIT_TEST_IN_PLACE_ALTERNATING_ENABLE=1`，任务层不提交任何
`ChassisSubmitCommand()`，Watch 中底盘命令 ID、目标距离和本任务实测
距离均保持 0。每侧放置完成并返回前方后，才切换到另一侧。

将该开关改为 `0` 后，恢复保留的场地路线：

```text
585 mm/点1 -> 500 mm/点2 -> 500 mm/点1 -> DONE
```

静态任务表位于 `Engineer/APPLICATION/app_fruit_task.c`。测试开关、场地和
机构尺寸位于 `app_fruit_task_config.h`，使用以下独立名称：

- `APP_FRUIT_TEST_IN_PLACE_ALTERNATING_ENABLE = 1`
- `APP_FRUIT_AREA_A_FIRST_POSITION_MM = 500`
- `APP_FRUIT_ARM_CENTER_BEHIND_NOSE_MM = 85`
- `APP_FRUIT_AREA_A_FIRST_MOVE_MM = 500 + 85`
- `APP_FRUIT_AREA_A_GROUP_SPACING_MM = 500`

底盘或机械臂出现明确故障后，任务锁存 `APP_FRUIT_TASK_FAILED`，原位停止，
不自动重试、不跳过点位。原地模式仍要求底盘和机械臂报告就绪，但不会
提交零距离伪命令；场地路线中仍必须等待底盘发布 `COMPLETED` 才能抓取。

## 3. A 区点位

坐标约定：`+X` 为物理车头方向，`+Y` 为物理左侧、`-Y` 为物理右侧，
`+Z` 向上。坐标和
世界绝对俯仰都直接参与工具中心IK；旧教导关节角不再作为抓取目标。

| 点位 | 语义 | 工具中心 `x/y/z` | 世界绝对俯仰 | 解析关节参考 |
|---|---|---|---|---|
| `APP_FRUIT_POINT_A_LEFT` | 业务点1，物理左侧 | `[0,400,-100] mm` | `-90 deg` | `[90,32.86,-101.44] deg` |
| `APP_FRUIT_POINT_A_RIGHT` | 业务点2，物理右侧 | `[0,-400,-100] mm` | `-90 deg` | `[-90,32.86,-101.44] deg` |

点1/点2按当前配置坐标确定：点1对应 `q1=+90 deg`，点2对应
`q1=-90 deg`。底座电机正方向与新的世界XY正方位一致，因此电机目标由
`atan2(y,x)` 得到；FK、IK和117 mm工具偏移均使用同一约定。抓取流程由
`AppArmFlowStartPick()` 执行：预对准限制为 `+/-89.5 deg`，同一条联合命令
让ID1和三台达妙一起进入 `[q2,q3]=[80,-90] deg`、ID1相对俯仰
`-80 deg` 的准备姿态。此时小臂绝对俯仰为 `-10 deg`，夹爪绝对俯仰为
`-90 deg`，并为关节到位误差保留限位余量；随后以 `200 mm/s` 请求速度
提交工具中心轨迹，等待反馈稳定后闭合 ID2。
夹爪接触后最多分级回退 4 次；4 次均耗尽时记录
`ARM_GRIPPER_FORCED_HELD`，仍按抓取成功继续，避免任务永久卡住。

## 4. A 区放置 profile

放置流程只读取传入的 `App_Arm_Place_Profile_s`。它不读取上一次抓取目标，
也不根据抓取 `q1` 正负推断左右侧。安全点显式提交完整三轴；定向转到
后方后，释放与释放后净空只更新 `q2/q3`，保留实际 `q1` 反馈，
避免在后方极限位置重复提交 `+180/-180 deg` 引起底座回拽。

### 点1（物理左侧，正角路线）

```text
安全位姿       [90, 90, -100] deg
到后方路线     +90 -> +135 -> +180 deg
释放位姿       q1保持实际反馈，q2=120、q3=-70 deg
ID1相对小臂    -45 deg
ID2释放后净空  q1保持实际反馈，q2=120、q3=-80 deg，小臂再上抬10deg
返回前方路线   +180 -> +90 -> 0 deg；到0deg即放置完成
```

### 点2（物理右侧，负角路线）

```text
安全位姿       [-90, 90, -100] deg
到后方路线     -90 -> -135 -> -180 deg
释放位姿       q1保持实际反馈，q2=120、q3=-70 deg
ID1相对小臂    -45 deg
ID2释放后净空  q1保持实际反馈，q2=120、q3=-80 deg，小臂再上抬10deg
返回前方路线   -180 -> -90 -> 0 deg；到0deg即放置完成
```

`AppArmFlowStartPlace(profile, now_ms)` 在提交第一条动作前校验：

- `configured` 和非零 `profile_id`；
- 安全、释放、释放后净空三组关节角均有限且位于软件限位；
- 到放置点和返回前方的引导角、终点角均有效；
- ID1 相对俯仰位于工具限位；
- 释放俯仰等待超时非零。

未配置返回 `APP_ARM_FLOW_START_NOT_CONFIGURED`，字段非法返回
`APP_ARM_FLOW_START_INVALID`，均不会开始机械动作。

## 5. 模块边界

| 层次 | 文件 | 职责 |
|---|---|---|
| 场地任务 | `app_fruit_task.c/.h` | 区域、侧别、点位、任务表、失败锁存和流程串联 |
| A区配置 | `app_fruit_task_config.h` | 物理尺寸、工具中心抓取点和点1/点2放置参数 |
| 机械臂子流程 | `app_arm_flow.c/.h` | 单次工具中心坐标抓取和显式profile放置 |
| 单侧抓放命令 | `app_arm_side_pick_place.c/.h` | 按LEFT/RIGHT执行准备、侧向推进、抓取、对应侧放置和回正 |
| 视觉目标换算 | `camera_target_transform.c/.h` | 相机外参、拍照姿态和相机点到机械臂基座点的刚体变换 |
| 底盘执行器 | `chassis/chassis.c/.h` | 相对直线/转角、连续vx/wz、里程计、IMU PID、超时减速和停稳 |
| 运行入口 | `app_runtime.c` | 初始化模块并在FreeRTOS包装任务中周期调用 |

当前生成协议哈希为 `0x2588BA9A`。`FruitDetection`仍只更新
`g_fruit_usb_debug`，不驱动本页静态任务表。新版`VelocityCommand`已经由
`upper_controller_bridge.c`转换为`Chassis_Velocity_Command_s`，桥内生成递增
命令ID；只给前后速度时`wz=0`，下位机锁定当前IMU航向保持直行。300ms未刷新
仍由底盘执行器平滑停车，协议桥不直接设置电机目标。

新版`ArmTarget`给出相机坐标系米制XYZ和`z_type`。固件已有
`P_B=T_B_E*T_E_C*P_C`完整点变换、外参矩阵校验和拍照姿态快照接口，但
当前外参保持未标定，协议也没有图像帧ID和抓取俯仰，因此仍不调用抓取流程；
可靠ACK只表示收到消息。动态视觉目标最终还必须通过现有工具中心IK、软件
限位和路径预检，不能直接覆盖本页静态点位。标定见
`docs/CAMERA_TARGET_TRANSFORM.md`。

未来增加上位机机械臂任务消息时，协议桥只把侧别映射为一次
`AppArmSidePickPlaceStart(APP_FRUIT_SIDE_LEFT/RIGHT, now_ms)` 调用；1ms应用
任务继续调用 `AppArmSidePickPlacePoll()`。LEFT命令只在完成左抓取、A左释放
和回正后返回 `DONE`，RIGHT同理。运行中重复命令返回 `BUSY`，不能重置当前
侧别、命令ID或profile。本轮没有修改 `protocol.c/.h`、USB消息或协议哈希。

## 6. 新增区域流程

B/C/D 当前全部视为未配置，不允许回退使用 A 区安全点或释放角度。新增
区域时必须依次完成：

1. 按明确坐标系重新教导抓取点并记录区域、侧别和点位 ID。
2. 单独教导安全位姿、到放置点路线、释放位姿和返回路线。
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
单侧公共命令看 `g_app_arm_posture_test_debug.operation_status`、
`last_start_result`、`active_side`、`start_count` 和左右完成计数；保留这个
原Watch全局名是为了兼容现有Keil联调布局。

机械臂模式实机依次确认底盘不动、点1完整抓放、点2完整抓放，并持续交替；
`task_index` 应在 `0/1` 间回绕，`completed_count` 持续递增。恢复场地路线
后，再按 `585/点1 -> 500/点2 -> 500/点1 -> DONE` 验收。
当前协议、桥接和坐标变换已通过ARM GCC严格检查；坐标变换29项离线检查
全部通过。当前MG995模式和临时上位机控制模式均通过Keil ArmCC 5全量构建，
0错误0警告，最终产物已恢复为MG995模式。未烧录、未标定相机外参，也未
进行上位机协议或机械动作实机验证。
