# vison 工程交接文档

更新时间：2026-08-17
工程目录：`C:\Users\11737\Desktop\vison`
Keil 工程：`Engineer\MDK-ARM\Engineer.uvprojx`
当前分支：`fruit`

实时源码始终优先于本文。继续工作前先查看 `git status`、`app_config.h`、
当前任务调用链和相关 Watch；不得回退用户已有修改。

## 0. 新会话快速接续

新会话首先读取本文和 `docs/CAMERA_TARGET_TRANSFORM.md`，然后核对：

```text
git status --short --branch
Engineer/APPLICATION/app_config.h
Engineer/APPLICATION/camera_target_transform_config.h
Engineer/APPLICATION/upper_controller_bridge.c
```

当前阶段20源码和文档尚未提交，工作区中的相机变换模块、上位机桥、Keil
工程、严格检查脚本及维护文档属于本轮有效改动，不得回退。当前另外存在
`Engineer/MDK-ARM/Engineer.uvoptx` 用户改动，本轮没有编辑它，新会话也不得
为了清理工作区覆盖或还原。

当前立即状态：

```text
默认APP_MODE              APP_MODE_HOST_CONTROL
默认实际运行              上位机协议控制底盘/夹爪/摄像头/AC闭环观察抓取
BD活动侧                  APP_ARM_BD_OBSERVATION_SIDE_LEFT
相机外参启用              CAMERA_TARGET_DEFAULT_CALIBRATED=1
ArmTarget坐标算法          已完成并接入观察链
ArmTarget机械臂动作        观察完成后允许AC闭环抓取，Z固定-100mm，俯仰-90deg
最近Keil AXF/HEX（HOST）   2026-08-17 02:58，早于本轮闭环接入
```

HOST模式保留的上位机底盘控制包为协议ID `0x14`、8字节payload：`linear_x`为
float/m/s，`angular_z`为float/rad/s。当前执行范围分别为
`|linear_x|<=1.0 m/s`、`|angular_z|<=1.5 rad/s`；有限超限值逐字段饱和，
NaN/Inf仍拒绝。建议至少10Hz持续发送，300ms未刷新时下位机平滑停车到
`CANCELLED`。切回HOST模式后仍会执行机械臂正常初始化和HOME，并保持ID2夹爪、
双MG995上位机命令可用；收到`StateMachineCommand task_id=2`时只运行AC
左/右观察姿态，BD观察状态机不运行；后续`ArmTarget`才触发闭环抓取。

闭环方案的AC/BD左右观察点已分别保存在`app_config.h`：LEFT为
`q1=+90deg/[0,+150,300]mm`，RIGHT为严格镜像
`q1=-90deg/[0,-150,300]mm`，绝对俯仰均为`-58deg`，斜向对应侧观察。当前
`APP_ARM_BD_OBSERVATION_ACTIVE_SIDE=LEFT`，staging为
`[+90,90,-80]deg`、ID1相对俯仰`-48deg`；两侧完整回放最低
`Z=77.557mm`、峰值`|Y|=318.051mm < 405mm`。405mm为BD专用回放边界；AC继续独立使用
`+/-440mm`抓取终点和445mm抓后边界。左右观察位已由用户确认，后续协议层
只能传递/选择LEFT或RIGHT，不能在协议桥、上位机消息处理或任务状态机中复制
这些坐标、俯仰和staging参数；几何参数继续只由`app_config.h`维护。
BD闭环通信、目标执行、抓取分段速度和放置profile仍未配置。阶段35的AC左右抓放
永久保留为开环备选；HOST协议中的AC闭环现在复用这些观察点，只做“观察->坐标
换算->Z=-100mm/俯仰=-90deg抓取”，不自动放置。

下一步是同步用户即将更新的上下位机通信协议。收到新协议文件后，以生成的
`protocol.h`、`protocol.c`和`PROTOCOL_DOC.md`为线格式事实来源，先核对协议哈希、
消息ID、payload和回调语义，再适配`protocol_runtime`与
`upper_controller_bridge`；不得用旧协议字段猜测新包，也不得借协议同步修改
本节已经确认的BD观察姿态。

末端摄像头外参已经保存：D435i固定在ID1之后并随夹爪俯仰，Fusion测得
ID1轴心到螺丝偏移，螺丝到RGB光心使用官方数据，XYZ轴方向已确认。结合
117mm工具长度换算后，`TOOL_CENTER`参考系下`t_E_C`为
`[-80.055106,69.184360,48.192321]mm`，`R_E_C`见
`camera_target_transform_config.h`。当前已置`calibrated=1`供AC闭环联调使用；
若实机已知点验证发现轴向或偏移错误，立即改回0。观察点到位时必须调用
`UpperControllerCaptureCameraPose(capture_id, now_ms)`；不能在推理结果到达时
读取当前姿态代替拍照姿态。

当前协议`ArmTarget`没有帧ID，也没有抓取绝对俯仰。本轮采用阶段性方案：只允许
使用最近一次AC观察完成时保存的pose snapshot，最大年龄5000ms；换算成功后
`X/Y`取基座系结果，`Z`固定`-100mm`，夹爪世界绝对俯仰固定`-90deg`并调用
`AppArmFlowStartPick()`。后续协议最好补真实帧ID；实机验证前不能把ACK、换算成功
或callback发送成功当成机械臂已完成。一次`ArmTarget`成功启动后会消费观察保持
状态，下一次闭环抓取必须重新发`task_id=2`观察。

## 1. 当前默认行为

当前默认为`APP_MODE_HOST_CONTROL`；USB协议在线后，上位机发送`VelocityCommand`，
`upper_controller_bridge.c`将`linear_x*1000`转换为
`vx_mm_s`，并保持`angular_z`为`wz_rad_s`提交到底盘连续速度接口：

```text
linear_x执行范围：[-1.0,+1.0] m/s
angular_z执行范围：[-1.5,+1.5] rad/s
推荐刷新：>=10 Hz
超时停车：300 ms未刷新后平滑停车到CANCELLED
纯平移：捕获当前IMU航向并自动直行保持
非零角速度：以上位机wz为主
```

有限超限速度包会逐字段钳位后提交；空包、NaN/Inf、链路未在线或bridge未
初始化时不会提交底盘动作。每个有效速度包使用递增命令ID刷新运行中的连续
速度命令。机械臂仍执行正常上电HOME，ID2
夹爪和双MG995的离散上位机命令仍可用；`ArmTarget`只有在AC观察完成保持后才会
驱动闭环抓取。`APP_MODE_ARM_BD_OBSERVATION_TEST` 仍保留为手动切回的单次左侧
观察路径测试模式，该模式不初始化USB业务或底盘。

MG995台架模式仍保留，切到 `APP_MODE_MG995_TEST` 后由
`mg995_servo.c/.h` 控制；`g_mg995_servo_debug.state` 应为
`MG995_SERVO_STATE_READY`，`initialized=1`；左右摄像头角度均为`0 deg`，
左侧为`pulse_us=1500/逻辑angle_deg=90`；右侧逻辑angle_deg仍为90，但因机械安装方向相反且零位偏高15deg，采用`+15deg`补偿，实际约为`pulse_us=1583/物理105deg`。摄像头坐标换算仍以逻辑90deg为水平中心。
MG995必须使用独立5~6V大电流供电并与STM32共地。

保留的 `APP_MODE_ARM_POSTURE_TEST` 固定先抓AC左侧，再抓右侧并持续循环：

```text
左侧：夹爪中心=[0,380,-140] -> [0,440,-140] mm
右侧：夹爪中心=[0,-380,-140] -> [0,-440,-140] mm
两侧夹爪世界绝对俯仰=-5 deg
接近点请求速度=600 mm/s，最后60 mm推进速度=150 mm/s
左抓取/A左放置 -> 返回前方 -> 右抓取/A右放置 -> 返回前方 -> 重复
```

AC抓取后进入放置准备时，首条关节命令通过profile waypoint执行：
`抓取终点 -> [q1=+/-90,q2=27.3,q3=-62.7] -> [+/-90,90,-100] deg`。
普通关节轨迹锁存抓取结束时的ID1相对小臂角；离线逐1deg回放得到第一段
`Z=-140 -> -120.007 mm`，抬高`19.993 mm`且不下探，完整两段
名义`max |Y|=440 mm`，运行时允许上限为445mm。固件提交首段前还会按实际
反馈重算；Watch看
`transfer_path_y_limit_mm`、`transfer_path_peak_abs_y_mm`和
`transfer_path_y_check_passed`，以及对应的`transfer_path_z_*`字段；Y超限、
Z下探、抬高量超出`20+/-2 mm`或ID1反馈无效都会锁存预检失败且不运动。
这组waypoint和Y/Z约束不是公共A区profile的固有参数：
`app_fruit_task.c`中的A左/A右公共profile保持两个开关为0并直接进入安全姿态；
只有`AppArmSidePickPlacePrepare()`复制profile后，才为AC单侧任务开启并注入。

每轮抓取预对准均通过 `AppArmFlowBuildPickStaging()` 生成同一组准备语义：
`q1=+/-89.5 deg`、`q2=80 deg`、`q3=-90 deg`、ID1相对俯仰`-80 deg`。
左/右放置回到 `q1=0` 后，下一侧必须重新提交这条四轴联合命令；只复制
反馈姿态并覆盖 `q1` 会保留释放后的 `q2/q3=[120,-80] deg`，并在下一侧
工具中心直线的首个采样点触发前方栏框保护。

Watch 主要使用 `g_app_arm_posture_test_debug`。该模式现定义为 AC 区专项，
不运行底盘任务、不调用 `AppFruitTask()`，但会闭合ID2并按当前侧显式复用
已验证的A左/A右放置profile。
`active_side=1/2`分别表示左/右；另有总计和左右完成次数。正式点1/点2及
带底盘任务表未修改，恢复 `APP_MODE_ARM` 后继续使用。

AC区完整单侧动作已经迁入 `app_arm_side_pick_place.c/.h`：调用
`AppArmSidePickPlaceStart(APP_FRUIT_SIDE_LEFT, now_ms)` 会完成左侧准备、
接近、推进、抓取、A左释放和回正；RIGHT同理使用镜像路径和A右profile。
接口为非阻塞命令，1ms机械臂应用任务必须持续调用
`AppArmSidePickPlacePoll(now_ms)`。单侧成功后停在 `DONE`，不会由模块自行
切换另一侧；当前持续交替仅由 `app_runtime.c` 在收到 `DONE` 后提交下一侧。
运行中重复提交返回 `BUSY`，非法侧别返回 `INVALID_SIDE`，明确故障锁存
`FAILED`。新版协议尚未定义左右侧完整抓放命令，因此该接口当前仍只供专项
测试调用；后续增加AC区侧别消息时应直接调用它，不复制内部状态机。BD区
目前已保存左右两个独立观察位，完整抓取坐标、分段速度和放置profile仍未配置，
不能覆盖AC参数。

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
| `app_runtime.c` | 初始化、FreeRTOS包装入口和BD左右观察位单次状态机 |
| `MODULE/servo/mg995_servo.c/.h` | TIM8双路MG995角度/PWM控制和Watch |
| `app_fruit_task.c/.h` | A区静态任务表、区域/侧别/点位和调度失败锁存 |
| `app_fruit_task_config.h` | A区距离、工具中心抓取点和左右放置参数 |
| `app_arm_flow.c/.h` | 单次工具中心坐标抓取和显式profile放置 |
| `app_arm_side_pick_place.c/.h` | AC区LEFT/RIGHT单侧完整抓放命令；BD不复用该入口 |
| `app_arm_command_id.c/.h` | 固件内部机械臂命令ID的唯一分配器 |
| `arm/` | 主臂、工具、轨迹、运动学和安全保护 |
| `chassis/chassis.c/.h` | 通用相对运动命令执行器 |
| `chassis/chassis_config.h` | 已验证方向、机械参数、PID和停车边界 |
| `camera_target_transform.c/.h` | 外参校验、拍照姿态快照和相机点到机械臂基座点的刚体变换 |
| `upper_controller_bridge.c/.h` | 新版速度、夹爪、摄像头和ArmTarget协议适配及Watch |

完整 A 区语义和扩展步骤见 `docs/FRUIT_TASK_FLOW.md`。

`ArmTarget`已经进入`P_B=T_B_E*T_E_C*P_C`算法，已启用随ID1转动的D435i
工具中心外参。AC观察完成时调用`UpperControllerCaptureCameraPose(capture_id,
now_ms)`保存拍照时反馈，不能在推理结果到达后读取当前机械臂姿态代替。
当前协议没有帧ID和抓取俯仰；阶段性实现使用最新AC观察快照，换算后固定
`Z=-100mm`并启动`AppArmFlowStartPick()`。详见`docs/CAMERA_TARGET_TRANSFORM.md`。

## 3. A 区机械臂数据

坐标为 `+X` 车头、`+Y` 物理左侧、`-Y` 物理右侧、`+Z` 向上。

- 业务点1：工具中心 `[0,400,-100] mm`，世界绝对俯仰 `-90 deg`。
- 业务点2：工具中心 `[0,-400,-100] mm`，世界绝对俯仰 `-90 deg`。

两组坐标直接参与工具中心IK；点1对应 `q1` 约 `+90 deg`，点2对应约
`-90 deg`，当前解析关节参考分别为 `[90,32.86,-101.44] deg` 和
`[-90,32.86,-101.44] deg`。

点1安全位姿 `[90,90,-100]`，经 `+135` 到 `+178`；点2安全位姿
`[-90,90,-100]`，经 `-135` 到 `-178`。两侧释放均使用
`q2=120`、`q3=-70`、ID1相对小臂 `-45 deg` 释放；ID2张开完成后保持
`q2=120`，小臂再上抬10deg到 `q3=-80`，随后分别经 `+90/-90` 返回
`q1=0` 并结束放置。下一抓预对准时，ID1与三台达妙同步进入
`[q2,q3]=[80,-90]`、ID1相对俯仰 `-80 deg`；随后工具中心轨迹以
`200 mm/s` 请求速度运行。该准备姿态避免关节到位误差把相对角推过
`-90 deg` 软件下限。定向转到后方以后，左右后方目标分别保持
`q1=+178/-178 deg`；释放与抬臂命令显式提交profile中的同一q1，不再
使用底座瞬时反馈，避免越过 `+/-180 deg` 时发生周期角翻边。

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
`upper_controller_bridge.c` 转换为该公共速度接口；当前默认
`APP_MODE_HOST_CONTROL`会初始化USB、底盘和IMU。手动切回BD观察位专项模式时
不会初始化底盘。

速度接口限制为`|vx|<=1000 mm/s`、`|wz|<=1.5 rad/s`。有限超限值分别
饱和到正负上限并返回ACCEPTED，NaN/Inf仍返回INVALID。左右轮按
`vl=vx-wz*L/2`、`vr=vx+wz*L/2`换算；任一轮超过1.3m/s时两侧按相同
比例缩小，保持上位机给定的转弯曲率。当前最大组合命令的单轮理论峰值为
`1.24m/s`，正常不会触发二次比例限幅。

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
- `g_protocol_runtime_debug`：握手、心跳、会话和可靠发送队列。
- `g_camera_target_transform_debug`：外参、旋转矩阵质量、拍照姿态、相机/E/基座三层坐标和失败状态。
- `g_upper_controller_debug`：新版离散命令、底盘速度、执行回调、ArmTarget原始点和换算结果。
  其中`current_area/current_area_valid/current_area_callback_pending/current_area_update_count`观察task 5，
  `qr_pose_request_count/qr_pose_unsupported_count`确认未配置的task 4请求。

机械臂邮箱只接受比上一条更新的命令ID。当前专项姿态测试和
`AppArmFlow` 已统一调用 `AppArmCommandIdNext()`；后续新增任何固件内部
机械臂命令生产者也必须复用该分配器，禁止新增模块私有基址或在流程初始化时
重置序列。底盘命令使用另一套邮箱，继续维护独立ID域。

## 9. 协议和定位边界

当前生成协议哈希为 `0x740E426B`，生成文件是
`protocol.c/.h/PROTOCOL_DOC.md`。`protocol_runtime.*`、
`protocol_port.*`、`upper_controller_bridge.*` 是工程维护
文件，不能随生成文件一起覆盖。新版无需强制握手且心跳非严格，生成FSM自动
回心跳和入站可靠消息ACK；运行层只观察状态，不再重复回心跳。

`VelocityCommand` 已接到底盘连续速度接口；`StateMachineCommand` 已接到ID2
夹爪、双MG995和AC闭环观察方向。`task_id=2`时status 0进入左观察、1进入右观察、
2在当前闭环链路中判为无效，使用`ExecutionCallback(callback_id=2)`报告观察
执行中1和到位完成0；完成时保存pose snapshot。`ArmTarget`会执行经过强校验的
相机点到基座点换算；满足AC观察保持、快照新鲜、机械臂空闲后，调用
`AppArmFlowStartPick()`，目标`X/Y`取换算值、`Z=-100mm`，夹爪世界绝对俯仰
固定`-90deg`。新协议已删除`FruitDetection`；`ArmTarget`使用`callback_id=3`
报告闭环抓取执行中1和完成0。
成功启动后观察状态被消费，不能用同一快照连续触发多次抓取。

`task_id=5`已经同步：status 0/1/2/3分别保存当前区域A/B/C/D，不占用正在
执行的夹爪/摄像头/AC离散状态；生成FSM自动ACK后，下一次桥任务发送
`callback_id=5`的status 1和0。
维护代码通过`UpperControllerGetCurrentArea()`读取，不能直接把Watch结构体当
业务接口。区域只表示车辆所在区，本协议版本没有BD观察LEFT/RIGHT选择字段，
因此task 5不会自动移动机械臂。

`task_id=4/status=0`虽在生成文档中命名为二维码识别姿态，但当前工程和协议均
没有给出坐标、关节角或既有执行入口。下位机只累计
`qr_pose_request_count/qr_pose_unsupported_count`，不移动机械臂，也不发送虚假的
`callback_id=4`执行中或完成。后续必须先由用户确认姿态，再接非阻塞动作和真实
终态回调；可靠ACK仍只代表收包。

底盘直线距离取左右主动轮相对里程平均值，IMU 只闭环航向。
`g_chassis_debug.y_m` 是积分观察值，不参与横向闭环。后续上位机需要提供
带符号且有新鲜度判断的横向误差，通过低带宽外环修正航向；不能把累计
`y_m` 当成外部绝对位置。

## 10. 构建和验证边界

最近一次HOST模式已通过ARM GCC严格检查和Keil ArmCC 5.06u7全量重建，
`Code=130740`、`RO-data=3600`、`RW-data=1436`、`ZI-data=136984`，
2026-08-17 02:58生成新AXF/HEX/MAP，构建为0错误0警告。MAP确认
`on_receive_VelocityCommand -> ChassisSubmitVelocityCommand`，以及
`UpperControllerBridgeTask -> AC Start/Poll -> AppArmSidePickPlaceStart`；
`AppArmTask`也真实调用单侧任务Poll。该AXF/HEX早于当前BD左观察模式切换，
不能当作本轮可上传固件；BD双侧及AC路径回放仍通过。
未烧录，也未进行真实上位机USB发送、1m/s底盘动作、AC方向命令、夹爪、
摄像头、相机标定或ArmTarget实机验收。

当前BD观察测试不执行水果任务。切到`APP_MODE_ARM`后，才验收底盘
保持不动、点1 `[0,400,-100]` 与点2
`[0,-400,-100]` 完整抓放并持续交替；抓取时ID1绝对俯仰应为`-90 deg`。
关闭原地交替开关并恢复场地路线后，再按
`585 mm -> 抓1 -> 500 mm -> 抓2 -> 500 mm -> 抓3 -> DONE`验收。

## 11. 工作区注意事项

- 当前分支为 `fruit`，保留所有用户已有未提交修改。
- 不使用 `git reset --hard`、`git checkout --` 或宽泛清理。
- PowerShell 可能以 `8009001d` 失败，出现后改用原生 `cmd.exe`。
- 实时 `app_config.h` 和源码优先于任何旧构建产物或历史说明。
