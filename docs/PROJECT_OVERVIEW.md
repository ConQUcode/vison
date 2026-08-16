# 当前工程功能

更新时间：2026-08-16

## 当前默认运行内容

`Engineer/APPLICATION/app_config.h` 当前为：

```c
#define APP_MODE APP_MODE_MG995_TEST
#define APP_ARM_TOOL_CENTER_TEST_ENABLE 1u
```

当前只启动 TIM8 的两路MG995 PWM：右侧为 `PI6/TIM8_CH2`，左侧为
`PI7/TIM8_CH3`。舵机 `90 deg` 定义为摄像头水平 `0 deg`；当前两侧摄像头
均恢复到水平 `0 deg`，左右舵机均为 `90 deg/1500 us`。机械臂、
底盘和IMU应用模块不初始化，DM/DJI周期控制入口为空操作。

切回 `APP_MODE_ARM_POSTURE_TEST` 后，机械臂仍按先左后右、只对Y取负的
已验证路径运行：

```text
左侧=[0,340,-150] -> [0,480,-150] mm
右侧=[0,-340,-150] -> [0,-480,-150] mm
两侧世界绝对俯仰=-5 deg
请求速度=100 mm/s
左抓取/A左放置 -> 返回前方 -> 右抓取/A右放置 -> 返回前方 -> 重复
```

左右两侧每次都先调用 `AppArmFlowBuildPickStaging()`，用一条联合命令同时
到达 `q1=+/-89.5 deg`、`q2=80 deg`、`q3=-90 deg` 和ID1相对俯仰
`-80 deg`，再开始工具中心轨迹；这也适用于放置回正后的下一侧。

该模式不运行底盘和 `AppFruitTask()`，但按当前侧复用 A 左/A右放置
profile。正式点1 `[0,400,-100]`、点2 `[0,-400,-100]` 和带底盘任务表
仍保留；恢复 `APP_MODE_ARM` 后继续使用。完整流程见
`docs/FRUIT_TASK_FLOW.md`。

当前左右抓放不是两套复制流程。`AppArmSidePickPlaceStart(side, now_ms)`
受理一次LEFT或RIGHT完整任务，`AppArmSidePickPlacePoll(now_ms)`非阻塞推进到
对应侧释放并回正后的 `DONE`。`app_runtime.c` 只利用这个接口实现当前
LEFT/RIGHT循环；未来USB协议桥可提交相同命令，本轮尚未增加协议消息。

完整上电初始化顺序固定为 `q2大臂+q3小臂同步 -> q1底座 -> ID1/ID2`。
三台达妙先全部使能并保持当前位置；q2/q3由同一联合位姿命令驱动，
同步带耦合补偿在联合目标换算中继续生效。

## 软件分层

| 模块 | 职责 |
|---|---|
| `app_runtime.c` | 模式初始化和FreeRTOS任务包装，不保存水果路线 |
| `MODULE/servo/mg995_servo.c/.h` | TIM8_CH2/CH3双路MG995控制、角度换算和Watch |
| `app_fruit_task.c/.h` | A区任务表、点位语义、流程调度和任务Watch |
| `app_fruit_task_config.h` | A区距离、抓取点和左右放置参数 |
| `app_arm_flow.c/.h` | 单次工具中心坐标抓取和显式profile放置 |
| `app_arm_side_pick_place.c/.h` | 单次LEFT/RIGHT完整抓取、对应侧释放和回正 |
| `app_arm_command_id.c/.h` | 固件内部机械臂命令ID的唯一分配器 |
| `arm/` | 主臂、轨迹、运动学、工具和安全控制 |
| `chassis/` | 相对运动和连续vx/wz命令、里程计、IMU闭环、超时停车和停稳 |
| `fruit_usb_bridge.c/.h` | 水果识别结果校验和观察，不直接驱动任务 |
| `camera_target_transform.c/.h` | 相机外参、拍照姿态快照及摄像头系到机械臂基座系的刚体点变换 |
| `upper_controller_bridge.c/.h` | 新版上位机速度、离散执行和ArmTarget适配；当前目标变换后仍禁止运动 |

底盘公共接口为 `ChassisInit`、`ChassisSubmitCommand`、
`ChassisSubmitVelocityCommand`、`ChassisGetStatus`、`ChassisCancelMotion`
和 `ChassisEmergencyStop`。连续速度接口接收带递增ID的`vx_mm_s/wz_rad_s`；
零`wz`平移时自动锁定当前IMU航向，非零`wz`由上位机直接指定，300ms未刷新
则平滑停车到`CANCELLED`。接口已为未来上位机预留，但当前USB协议尚未包含
底盘速度消息。
机械臂左右抓放的固件内部命令边界已经提供，但同样尚未映射到USB消息ID。

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
| 右侧MG995 | TIM8_CH2 | PI6，舵机90deg/1500us，摄像头水平0deg |
| 左侧MG995 | TIM8_CH3 | PI7，舵机90deg/1500us，摄像头水平0deg |
| 姿态传感器 | SPI1 | BMI088 + INS/EKF |
| 上位机 | USB FS | CDC虚拟串口 |

## 坐标和当前点位

机械臂坐标为 `+X` 车头、`+Y` 物理左侧、`-Y` 物理右侧、`+Z` 向上。当前 HOME 为
`q=[0,90,-60] deg`。ID1 输出轴中心到夹爪中心长度为 `117 mm`。

- 点1工具中心：`[0,400,-100] mm`，世界绝对俯仰 `-90 deg`。
- 点2工具中心：`[0,-400,-100] mm`，世界绝对俯仰 `-90 deg`。

上述坐标直接参与工具中心IK，不再保存旧教导关节角。点1解算到
`q=[90,32.86,-101.44] deg`，点2解算到 `[-90,32.86,-101.44] deg`；
`-90 deg` 绝对俯仰由ID1随主臂轨迹同步保持。

放置流程由显式 `App_Arm_Place_Profile_s` 决定。点1从 `+90` 经 `+135`
到 `+180 deg`，点2从 `-90` 经 `-135` 到 `-180 deg`；ID2张开后小臂从
`q3=-70` 上抬到 `q3=-80`。定向转到后方后，释放和抬臂命令保留底座
实时反馈角，不再次提交 `q1=+180/-180`；底座回到 `q1=0` 即结束放置，
下一抓的底座预对准会让ID1与三台达妙同步进入 `[q2,q3]=[80,-90]`、
ID1相对俯仰 `-80 deg` 的准备姿态；随后工具中心轨迹以 `200 mm/s`
请求速度运行，使夹爪绝对俯仰保持 `-90 deg` 并保留 `10 deg` 名义余量。
底层不再根据抓取关节角猜测侧别。

## 底盘里程计边界

直线目标距离由左右主动轮相对里程的平均值判断，BMI088 只闭环航向。
`g_chassis_debug.x_m/y_m` 是根据轮速距离和 IMU 航向积分得到的估计坐标；
当前控制器不会根据 `y_m` 主动横向回线，因此轮径差、打滑和 IMU 偏差会
累积为横向估计误差。后续上位机闭环应提供有符号横向误差，再在公共底盘
外环中换算为`wz`并刷新`ChassisSubmitVelocityCommand()`，不能把里程计
`y_m`当成外部绝对位置。上位机只给`vx`并令`wz=0`时，下位机仍用IMU
锁定开始直行时的航向。

## 上位机协议边界

当前生成协议哈希为 `0x2588BA9A`，生成文件是
`protocol.c/.h/PROTOCOL_DOC.md`；项目维护的 `protocol_runtime.*`、
`protocol_port.*`、`fruit_usb_bridge.*` 和 `upper_controller_bridge.*` 不能被
生成文件替换。新版协议无需强制握手且心跳非严格，但下位机仍观察连接状态；
生成FSM负责心跳回包和入站可靠消息ACK，运行层不得重复回包。

业务消息新增 `StateMachineCommand`、`ExecutionCallback`、`ArmTarget` 和
`VelocityCommand`。在 `APP_MODE_HOST_CONTROL` 中，速度包转换为
`vx_mm_s/wz_rad_s`后提交底盘；离散包控制ID2夹爪或双MG995。
`ArmTarget`的米制相机坐标会先转换为毫米，再尝试执行
`P_B=T_B_E*T_E_C*P_C`。算法、旋转矩阵校验、腕部/夹爪两种安装参考系、
姿态快照和离线测试均已实现；当前外参配置保持`calibrated=0`，且协议没有
图像帧ID，因此桥只记录明确失败原因，不产生机械臂动作。拍照触发侧必须在
拍照瞬间调用`UpperControllerCaptureCameraPose(capture_id, now_ms)`，不能在
目标包到达时读取当前姿态代替。`z_type`只保留任务分类语义，不参与坐标计算。
具体标定参数和矩阵方向见`docs/CAMERA_TARGET_TRANSFORM.md`。

## 夹爪容错

ID2 默认/张开位置为 `450`，探测闭合目标为 `660`。接触后每次向张开
方向回退 `10`，最多 4 次。任一次到位稳定后完成抓取；4 次均不能跟随
时记录 `ARM_GRIPPER_FORCED_HELD`，仍按抓取成功继续，避免任务卡死。
持续离线、通信错误和初始化故障仍按真实故障处理。

## 运行模式

| 模式 | 作用 |
|---|---|
| `APP_MODE_MG995_TEST` | 当前默认，只输出PI6/PI7双路PWM并保持两侧摄像头水平0deg |
| `APP_MODE_HOST_CONTROL` | USB上位机控制底盘速度、ID2夹爪和双MG995；ArmTarget暂只观察 |
| `APP_MODE_ARM_POSTURE_TEST` | HOME后先执行左侧 `[0,340,-150] -> [0,480,-150]`，再执行右侧镜像路径并持续交替抓放 |
| `APP_MODE_ARM` | 底盘原地执行点1/点2无限交替抓放 |
| `APP_MODE_CHASSIS_ONE_METER` | 应用层通过公共底盘接口复现1m/右转90/1m循环 |
| `APP_MODE_HUANER_FEEDBACK` | 幻儿舵机反馈专项模式 |
| `APP_MODE_ARM_TEACH_POINT` | 三达妙和双舵机无力、保留反馈和坐标观察 |

## 常用 Watch

- `g_app_arm_posture_test_debug`：点1测试状态、坐标/关节目标与反馈、绝对俯仰误差和命令结果。
- `g_app_arm_command_id_debug`：公共机械臂命令ID种子、最后发放值、发放次数和回绕次数。
- `g_app_fruit_task_debug`：当前区域、侧别、点位、任务索引、距离和唯一失败来源。
- `g_chassis_debug`：公共状态、命令ID、目标/实测距离、故障、PID和轮速。
- `g_app_arm_pick_place_test_debug`：抓取/放置步骤、profile、命令和预检结果。
- `g_arm_tool_debug`：ID1/ID2通信、堵转、回退和强制抓取记录。
- `g_arm_dm_debug`：三台达妙反馈、使能和发送状态。
- `g_arm_dm_debug.auto_init`：`step=1, axis=4` 表示大臂/小臂同步HOME，`step=2, axis=1` 表示底座HOME；完成后才启动ID1/ID2。
- `g_fruit_usb_debug`：水果识别快照；当前不直接控制任务。
- `g_protocol_runtime_debug`：握手、心跳、会话和可靠发送队列状态。
- `g_camera_target_transform_debug`：相机外参、拍照姿态、旋转矩阵质量、三层坐标和唯一变换状态。
- `g_upper_controller_debug`：新版原始相机点、变换后基座点、目标/姿态计数以及底盘和离散命令状态。

所有固件内部机械臂命令必须通过 `AppArmCommandIdNext()` 领取ID。专项测试、
抓放流程和未来新增流程不得各自建立命令基址或重置私有序列，否则切换流程时
会被底层单调邮箱判定为 `ARM_COMMAND_DUPLICATE`。底盘使用独立邮箱和ID域，
不接入此分配器。

## 验证边界

当前协议、桥接和坐标变换已通过 ARM GCC 严格语法检查，坐标变换29项离线
检查全部通过；当前MG995模式和临时`APP_MODE_HOST_CONTROL`模式均通过
Keil ArmCC 5全量构建，均为0错误0警告，host模式MAP确认ArmTarget调用变换
模块。最终AXF/HEX已恢复为MG995默认模式。未烧录、未标定相机外参，也没有
替代USB、底盘、夹爪、摄像头或机械臂的实机验收。
