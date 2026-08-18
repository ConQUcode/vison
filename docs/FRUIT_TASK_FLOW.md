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

## 2. AC 区原地交替抓放测试

`APP_MODE_ARM_POSTURE_TEST` 保留为 AC 区（A/C 两区）专项流程：原地
先执行左侧 `[0,380,-140] -> [0,440,-140] mm`，再执行只对Y取负的
右侧 `[0,-380,-140] -> [0,-440,-140] mm`。到接近点的请求速度为
`600 mm/s`，最后 `60 mm` 推进段降为 `150 mm/s`；两侧分别取得当前
已验证A左/A右公共放置profile的副本，再注入AC专用收拢waypoint和Y/Z约束，
返回前方后持续交替。公共profile本身保持直接进入原安全姿态。

BD 区不接入该 AC 入口。当前已在独立模式中保存左右树上观察位；
完整抓取坐标、分段速度和放置profile仍需独立新增，不得覆盖
AC 区参数或直接假定两区几何相同。

AC区专项测试和正式抓取共用 `AppArmFlowBuildPickStaging()`。每侧抓取开始时，
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

### 上位机AC抓取方向命令

保留的`APP_MODE_HOST_CONTROL`通过`StateMachineCommand`的`task_id=2`
触发AC闭环观察姿态，不再直接复用AC开环完整抓放状态机：

```text
task_status=0：进入左侧观察姿态，保存拍照姿态快照
task_status=1：进入右侧观察姿态，保存拍照姿态快照
task_status=2：当前闭环链路无效，不接受双侧连续观察
```

观察任务受理后发送`ExecutionCallback(callback_id=2, status=1)`；到达对应
观察点并成功调用`UpperControllerCaptureCameraPose()`后，发送
`callback_id=2, status=0`并保持在观察状态。随后上位机发送`ArmTarget`，下位机
用最近一次观察快照进行`P_B=T_B_E*T_E_C*P_C`换算；抓取终点`X/Y`取换算后的
基座坐标，`Z`固定为`-100 mm`，夹爪世界绝对俯仰固定为`-90 deg`，
再调用`AppArmFlowStartPick()`完成到点闭合夹爪。
闭环抓取使用`ExecutionCallback(callback_id=3)`报告执行中1和完成0。一次
`ArmTarget`成功启动后会消费当前观察状态，必须重新观察才能再次抓取。阶段35的
AC完整左/右抓放仍作为开环备选保留在`app_arm_side_pick_place.c/.h`，不由当前
HOST `task_id=2`直接调用。

## 3. BD 区左右树上观察位测试

保留入口 `APP_MODE_ARM_BD_OBSERVATION_TEST` 可手动切回用于单次观察点测试；
当前默认已切回 `APP_MODE_HOST_CONTROL`。该测试入口上电HOME后自动执行一次
左侧观察路径并保持。`APP_ARM_BD_OBSERVATION_ACTIVE_SIDE`选择LEFT，右侧点仍作为
严格镜像保存并参与离线回放：

```text
LEFT： q1=+90 deg，夹爪中心=[0,+150,300] mm
RIGHT：q1=-90 deg，夹爪中心=[0,-150,300] mm
```

该测试模式先同步移动底座、主臂和ID1进入左侧紧凑过渡姿态，再提交
一次夹爪中心直线命令：

```text
HOME关节=[0,90,-60] deg
同步过渡关节=[+90,90,-80] deg，ID1相对俯仰=-48 deg
BD完整路径最低工具中心Z=77.557 mm
整段工具中心最大|Y|=318.051 mm，BD专用限制=405 mm
最终夹爪中心坐标=[0,+150,300] mm
最终世界绝对俯仰=-58 deg
请求速度=150 mm/s
预期终点IK约=[+90.000,123.291,-84.168] deg
```

终点的ID1相对小臂俯仰约 `-85.459 deg`，距离 `-90 deg` 下限约
`4.541 deg`。命令完成后保持位置，不闭合ID2、不运行底盘、不进入AC循环。
整条路径仍由轨迹层在执行前逐点检查IK、软限位、工作区和ID1可达性；
BD的405mm回放上限不用于AC；AC仍独立使用440mm抓取终点和445mm抓后上限。
任一检查失败都拒绝命令，不会反复重试。

左右摄像头选边、机械臂摄像头二次定位、实际抓取与放置路线尚未接入。
上述LEFT/RIGHT观察姿态已经确认，是后续通信协议接入的固定运动基准；协议消息
只选择侧别，不直接携带或覆盖观察点坐标、世界俯仰和staging关节参数。

## 4. A 区点位

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

## 5. A 区放置 profile

放置流程只读取传入的 `App_Arm_Place_Profile_s`。它不读取上一次抓取目标，
也不根据抓取 `q1` 正负推断左右侧。安全点显式提交完整三轴；定向转到
后方后，释放与释放后净空继续显式保持profile中的 `q1=+178/-178 deg`，
不再复制瞬时反馈，避免越过 `+/-180 deg` 时被周期角归一化到另一端。

### 点1（物理左侧，正角路线）

```text
收拢过渡       [90, 27.3, -62.7] deg；工具中心抬高约20mm
安全位姿       [90, 90, -100] deg
到后方路线     +90 -> +135 -> +178 deg
释放位姿       q1显式保持+178，q2=120、q3=-70 deg
ID1相对小臂    -45 deg
ID2释放后净空  q1显式保持+178，q2=120、q3=-80 deg，小臂再上抬10deg
返回前方路线   +178 -> +90 -> 0 deg；到0deg即放置完成
```

### 点2（物理右侧，负角路线）

```text
收拢过渡       [-90, 27.3, -62.7] deg；工具中心抬高约20mm
安全位姿       [-90, 90, -100] deg
到后方路线     -90 -> -135 -> -178 deg
释放位姿       q1显式保持-178，q2=120、q3=-70 deg
ID1相对小臂    -45 deg
ID2释放后净空  q1显式保持-178，q2=120、q3=-80 deg，小臂再上抬10deg
返回前方路线   -178 -> -90 -> 0 deg；到0deg即放置完成
```

`AppArmFlowStartPlace(profile, now_ms)` 在提交第一条动作前校验：

- `configured` 和非零 `profile_id`；
- 抓取后收拢路径Y上限、Z抬高目标和容差均为有效值；
- 收拢过渡、安全、释放、释放后净空四组关节角均有限且位于软件限位；
- 到放置点和返回前方的引导角、终点角均有效；
- ID1 相对俯仰位于工具限位；
- 释放俯仰等待超时非零。

未配置返回 `APP_ARM_FLOW_START_NOT_CONFIGURED`，字段非法返回
`APP_ARM_FLOW_START_INVALID`，均不会开始机械动作。

当前AC抓取终点到安全位姿必须经过上述收拢过渡点。轨迹层分别预检并
分段线性执行`终点 -> waypoint -> safe_q`，ID1保持动作开始时的相对小臂角。
`tools/arm_path_replay`同时验证左右镜像路径：第一段Z不下探并抬高
`19.993 mm`，完整两段名义最大`|Y|=440 mm`，运行时Y上限为445mm。每次
实际提交前还会使用当前三轴反馈和ID1反馈重算；Y检查与Z检查均通过才会发送
关节命令，超限则锁存`ARM_COMMAND_PREFLIGHT_FAILED`。

该约束只作用于`AppArmSidePickPlacePrepare()`生成的AC单侧profile副本。
`app_fruit_task.c`中的正式A左/A右公共profile令`transfer_waypoint_valid=0`、
`transfer_path_constraints_enabled=0`，继续按原逻辑直接进入安全姿态。

## 6. 模块边界

| 层次 | 文件 | 职责 |
|---|---|---|
| 场地任务 | `app_fruit_task.c/.h` | 区域、侧别、点位、任务表、失败锁存和流程串联 |
| A区配置 | `app_fruit_task_config.h` | 物理尺寸、工具中心抓取点和点1/点2放置参数 |
| 机械臂子流程 | `app_arm_flow.c/.h` | 单次工具中心坐标抓取和显式profile放置 |
| 单侧抓放命令 | `app_arm_side_pick_place.c/.h` | 按LEFT/RIGHT执行准备、侧向推进、抓取、对应侧放置和回正 |
| 视觉目标换算 | `camera_target_transform.c/.h` | 相机外参、拍照姿态和相机点到机械臂基座点的刚体变换 |
| 底盘执行器 | `chassis/chassis.c/.h` | 相对直线/转角、连续vx/wz、里程计、IMU PID、超时减速和停稳 |
| 运行入口 | `app_runtime.c` | 初始化模块并在FreeRTOS包装任务中周期调用 |

当前生成协议哈希为 `0x740E426B`，新协议已删除`FruitDetection`。
`VelocityCommand`已经由
`upper_controller_bridge.c`转换为`Chassis_Velocity_Command_s`，桥内生成递增
命令ID；只给前后速度时`wz=0`，下位机锁定当前IMU航向保持直行。300ms未刷新
仍由底盘执行器平滑停车，协议桥不直接设置电机目标。

新版`ArmTarget`给出相机坐标系米制XYZ和`z_type`。固件已有
`P_B=T_B_E*T_E_C*P_C`完整点变换、外参矩阵校验和拍照姿态快照接口。当前AC闭环
阶段已启用D435i外参，但协议仍没有图像帧ID和抓取俯仰，因此只允许使用最近
一次AC观察完成时保存的快照，最大年龄5000ms；换算成功后固定抓取`Z=-100mm`，
夹爪世界绝对俯仰固定`-90deg`，`X/Y`来自基座坐标，随后进入工具中心IK、
软件限位和路径预检。可靠ACK只表示
收到消息，`callback_id=3`才表示闭环抓取动作执行中或完成。标定见
`docs/CAMERA_TARGET_TRANSFORM.md`。

`StateMachineCommand task_id=5`已用于同步当前区域：status 0/1/2/3分别为
A/B/C/D，下位机保存后发送`callback_id=5`的执行中和完成；该状态不会选择
BD观察左/右侧，也不会自动启动机械臂。`task_id=4`二维码识别姿态尚未定义
实际点位，只记录明确的unsupported Watch，不发送callback 4完成。

未来增加上位机机械臂任务消息时，协议桥只把侧别映射为一次
`AppArmSidePickPlaceStart(APP_FRUIT_SIDE_LEFT/RIGHT, now_ms)` 调用；1ms应用
任务继续调用 `AppArmSidePickPlacePoll()`。LEFT命令只在完成左抓取、A左释放
和回正后返回 `DONE`，RIGHT同理。运行中重复命令返回 `BUSY`，不能重置当前
侧别、命令ID或profile。本轮没有修改 `protocol.c/.h`、USB消息或协议哈希。

## 7. 新增区域流程

B/D 当前除左右树上观察位外仍视为未配置，不允许回退使用 A 区安全点或释放角度。新增
区域时必须依次完成：

1. 按明确坐标系重新教导抓取点并记录区域、侧别和点位 ID。
2. 单独教导安全位姿、到放置点路线、释放位姿和返回路线。
3. 用关节软限位和工作区安全规则完成静态预检。
4. 新建独立 `App_Arm_Place_Profile_s`，不得复制后只改名称。
5. 先架空验证方向，再进行低速实机验证。
6. 验证通过后才允许把点位/profile加入任务表。

## 8. Watch 和实机验收

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
当前 BD 观察位测试看 `g_app_arm_bd_observation_debug`：`state=4`
表示已到位保持，`state=5` 表示提交或路径预检失败；同时确认
`actual_center_mm`、`actual_tool_pitch_deg`、`actual_q_deg`、`center_error_mm`、
`pitch_error_deg` 和 `path_preflight_passed`。

机械臂模式实机依次确认底盘不动、点1完整抓放、点2完整抓放，并持续交替；
`task_index` 应在 `0/1` 间回绕，`completed_count` 持续递增。恢复场地路线
后，再按 `585/点1 -> 500/点2 -> 500/点1 -> DONE` 验收。
当前协议、桥接和坐标变换已通过ARM GCC严格检查；坐标变换29项离线检查
全部通过。MG995模式和临时上位机控制模式的历史构建均通过Keil ArmCC 5
全量构建，0错误0警告；最新有效产物为 BD 右观察位专项模式，同样
0错误0警告。
未烧录、未标定相机外参，也未
进行上位机协议或机械动作实机验证。
