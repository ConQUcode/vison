# 参数位置和调参说明

更新时间：2026-08-16

修改参数后需要重新编译。底盘和机械臂首次动作应架空测试；当前已实测
方向符号不得凭直觉改动，每次只调整一个参数组。

## 运行模式

文件：`Engineer/APPLICATION/app_config.h`

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `APP_MODE` | `APP_MODE_ARM_BD_OBSERVATION_TEST` | 当前HOME后单次到BD右观察位并保持 |
| `APP_ARM_BD_OBSERVATION_ACTIVE_SIDE` | `RIGHT` | 当前上传固件执行右观察点 |
| BD左观察点 | `q1=+90 deg / [0,+57,210] mm` | 已保存并参与每次双侧回放 |
| BD右观察点 | `q1=-90 deg / [0,-57,210] mm` | 已保存并参与每次双侧回放 |
| BD当前同步过渡关节 | `[-90,120,-48] deg` | HOME后底座和主臂同步动作，最大 `|Y|=231.470 mm` |
| BD路径Y限制 | `260 mm` | 两段离线回放均逐点检查工具中心 `|Y|` |
| BD观察俯仰 | `-30 deg` | 最终夹爪中心线世界绝对俯仰 |
| BD观察速度 | `150 mm/s` | 首次单点确认速度 |
| 右侧MG995 | `PI6 / TIM8_CH2 / 1500 us` | 舵机90deg，对应摄像头0deg |
| 左侧MG995 | `PI7 / TIM8_CH3 / 1500 us` | 舵机90deg，对应摄像头0deg |
| AC左侧定姿路径 | `[0,380,-150] -> [0,480,-150] mm` | 使用A左放置profile |
| AC右侧定姿路径 | `[0,-380,-150] -> [0,-480,-150] mm` | 只对左侧Y取负，使用A右放置profile |
| 两侧绝对俯仰 | `-5 deg` | X/Z、速度、等待和夹爪动作完全一致 |
| AC定姿速度 | `600/150 mm/s` | 接近点/最后100 mm推进分段速度 |
| `APP_ARM_TOOL_CENTER_TEST_ENABLE` | `1` | 为正式水果流程和当前A左/A右放置提供共享子流程 |

当前BD观察位模式不运行底盘或夹爪，也不进入AC循环。观察
`g_app_arm_bd_observation_debug`：`state=4` 为到位保持，`state=5`
为失败；重点查看 `actual_center_mm`、`actual_tool_pitch_deg`、`actual_q_deg`、
`center_error_mm`、`pitch_error_deg` 和 `path_preflight_passed`。

切到 `APP_MODE_MG995_TEST` 后只调用 `Mg995ServoInit()`，机械臂、底盘、
IMU、USB业务和DM/DJI电机周期控制均不运行。观察
`g_mg995_servo_debug`：正常应为 `state=READY`、`initialized=1`。

切回 `APP_MODE_ARM_POSTURE_TEST` 后，机械臂才会先完成左抓放并返回前方，
再完成右抓放并持续交替。只有放置子流程返回 `DONE` 才切换侧别；任何
机械臂、舵机或profile故障都锁存 `FAILED`，不会跳过当前侧。

左右完整动作统一由 `AppArmSidePickPlaceStart(side, now_ms)` 提交并由
`AppArmSidePickPlacePoll(now_ms)` 周期推进。调参时不要在上位机桥或
`app_runtime.c` 复制坐标和放置角；它们只选择LEFT/RIGHT并观察
`g_app_arm_posture_test_debug.operation_status`。运行中重复提交返回 `BUSY`，
成功完成一侧后状态保持 `DONE`，由调用者决定是否以及何时提交下一侧。

当前绝对俯仰为 `-5 deg`、Z为 `-150 mm`。按实时运动学对
`|Y|=380..480 mm` 逐毫米镜像预检，两侧q2/q3和ID1结果一致；接近点ID1
相对俯仰约 `+82.80 deg`、控制值约 `155.02`，距当前
`+92.4 deg/115` 软件边界约 `9.60 deg/40.02`。
终点q2约 `7.88 deg`，距3deg软件下限约 `4.88 deg`。

`APP_ARM_COMMAND_ID_SEED=0xA1100000u` 是固件内部机械臂命令序列的启动
种子，通常不作为调参项修改。专项测试、抓放流程和未来新增内部流程必须统一
调用 `AppArmCommandIdNext()`；不得新增私有命令基址，也不得在子流程初始化时
重置序列。可通过 `g_app_arm_command_id_debug` 检查最后发放ID和计数。底盘
命令属于独立命令域，不使用该种子。

## A 区任务参数

文件：`Engineer/APPLICATION/app_fruit_task_config.h`

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `APP_FRUIT_TEST_IN_PLACE_ALTERNATING_ENABLE` | 1 | 原地左右点无限交替；不提交底盘命令 |
| `APP_FRUIT_AREA_A_FIRST_POSITION_MM` | 500 mm | 启动区边界到A区第一组水果的纵向距离 |
| `APP_FRUIT_ARM_CENTER_BEHIND_NOSE_MM` | 85 mm | 启动时机械臂中心落后物理车头的安装偏移 |
| `APP_FRUIT_AREA_A_FIRST_MOVE_MM` | 585 mm | 首段 `500+85`，只补偿一次安装偏移 |
| `APP_FRUIT_AREA_A_GROUP_SPACING_MM` | 500 mm | A区相邻两组水果的纵向距离 |
| `APP_FRUIT_CHASSIS_TOLERANCE_MM` | 3 mm | 水果任务停车距离容差 |

A区共4组、每组左右各1个水果；当前测试开关为 `1`，底盘原地不动，
点1/点2完整抓放并无限交替。将开关改为 `0` 后恢复连续3组场地测试，
距离为 `585/500/500 mm`。两点抓取坐标和放置角也集中在该文件；
`A_LEFT/A_RIGHT` 分别固定表示物理左侧 `Y>0` 和物理右侧 `Y<0`。
BD目前只配置左右树上观察位，完整抓放仍不能套用A/AC区参数。

## 底盘通用接口和方向

接口：`Engineer/APPLICATION/chassis/chassis.h`

`Chassis_Command_s.distance_mm` 使用毫米，正值固定表示物理车头向前，负值
表示后退。`CHASSIS_COMMAND_RELATIVE_TURN` 的正角表示逻辑 Yaw 增加。

公共状态：

```text
WAIT_READY -> IDLE -> RUNNING -> STOPPING -> COMPLETED
                                      +----> CANCELLED
任意明确故障 ------------------------------> FAULT
```

重复/旧命令 ID、未就绪、忙、非法参数和故障锁存分别返回独立结果。
`COMPLETED` 只会在速度低于阈值并连续稳定后发布。

文件：`Engineer/APPLICATION/chassis/chassis_config.h`

| 参数 | 当前值 | 调整说明 |
|---|---:|---|
| `CHASSIS_LEFT_COMMAND_SIGN` | `-1.0` | 已验证左轮物理方向，不要联动猜测 |
| `CHASSIS_RIGHT_COMMAND_SIGN` | `1.0` | 已验证右轮物理方向 |
| `CHASSIS_LEFT_FEEDBACK_SIGN` | `-1.0` | 物理向前时逻辑里程应增加 |
| `CHASSIS_RIGHT_FEEDBACK_SIGN` | `1.0` | 物理向前时逻辑里程应增加 |
| `CHASSIS_IMU_YAW_SIGN` | `-1.0` | 逆时针车体旋转时逻辑Yaw增加 |
| `CHASSIS_WHEEL_RADIUS_M` | 0.0475 m | 距离整体偏小时增大，偏大时减小 |
| `CHASSIS_REDUCTION_RATIO` | 19.2032 | M3508减速比，通常不改 |
| `CHASSIS_TRACK_WIDTH_M` | 0.320 m | 轮中心距，影响轮差Yaw和转动目标 |
| `CHASSIS_TEST_MAX_SPEED_M_S` | 0.200 m/s | 当前直线最大速度 |
| `CHASSIS_TEST_MIN_SPEED_M_S` | 0.060 m/s | 终点附近最低速度 |
| `CHASSIS_TEST_DECEL_DISTANCE_M` | 0.250 m | 增大可更早减速 |
| `CHASSIS_MAX_LINEAR_ACCEL_M_S2` | 0.35 m/s2 | 增大响应和电流冲击都会增加 |
| `CHASSIS_STOP_SPEED_M_S` | 0.020 m/s | 停稳速度阈值 |
| `CHASSIS_STOP_STABLE_MS` | 300 ms | 连续停稳窗口 |

直行 PID 看 `CHASSIS_HEADING_*`，转角 PID 看 `CHASSIS_TURN_*`。D 项使用
BMI088 Z 轴角速度，不是离散 Yaw 误差差分。推荐顺序：反馈方向、命令
方向、IMU方向、有效轮径、轮距、速度环、直行P/D/I、转角P/D/I。

连续速度接口为`ChassisSubmitVelocityCommand()`：

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `CHASSIS_VELOCITY_MAX_LINEAR_MM_S` | 200 mm/s | 上位机vx绝对值上限 |
| `CHASSIS_VELOCITY_MAX_ANGULAR_RAD_S` | 0.8 rad/s | 上位机wz绝对值上限 |
| `CHASSIS_VELOCITY_COMMAND_TIMEOUT_MS` | 300 ms | 未刷新后平滑停车到CANCELLED |
| `CHASSIS_VELOCITY_LINEAR_ZERO_MM_S` | 0.5 mm/s | vx零值归一化阈值 |
| `CHASSIS_VELOCITY_ANGULAR_ZERO_RAD_S` | 0.005 rad/s | wz零值及IMU直行保持判定阈值 |

每次刷新必须使用更大的`command_id`。`wz=0`且`vx!=0`时，首次命令捕获
当前Yaw并持续复用同一目标；非零`wz`不叠加航向保持，转回零`wz`平移时
重新捕获当前Yaw。左右轮联合超限时通过`velocity_wheel_scale`同比例缩小，
不能分别削顶改变曲率。新版USB `VelocityCommand` 已在
`APP_MODE_HOST_CONTROL`中通过`upper_controller_bridge.c`调用该接口；协议输入
`linear_x`为m/s，桥内乘1000转换为`vx_mm_s`，`angular_z`保持rad/s。

主要 Watch：

- `state/command_id/command_type/last_submit_result`
- `target_distance_mm/actual_distance_mm/tolerance_mm`
- `heading_target_deg/heading_error_deg/heading_pid_*`
- `target_angle_deg/actual_angle_deg/turn_pid_*`
- `left_target_m_s/right_target_m_s/left_speed_m_s/right_speed_m_s`
- `velocity_target_vx_mm_s/velocity_actual_vx_mm_s`
- `velocity_target_wz_rad_s/velocity_actual_wz_rad_s`
- `velocity_heading_hold_active/velocity_command_tick`
- `velocity_refresh_count/velocity_timeout_count/velocity_heading_capture_count`
- `velocity_wheel_scale`
- `fault/motor_offline_count/imu_fault_count/direction_fault_count`

### 里程计 Y 误差

直线完成条件使用左右主动轮相对里程的平均值，IMU PID 只保持起步航向。
`g_chassis_debug.x_m/y_m` 是轮里程与 IMU 航向的积分估计，不是绝对定位；
当前 `y_m` 不参与横向闭环。两轮有效直径不一致、地面打滑、从动轮扰动
和航向零偏都会使 Y 误差累计。

后续上位机闭环时，应由相机、场地线或其他外部定位输出带符号的
`lateral_error_mm`，先滤波和限幅，再将其转换为小幅航向修正；底盘内部
仍负责轮速、航向和停车。外环丢失或数据过期时必须降级为当前航向保持，
不要直接用累计的 `y_m` 作为绝对横向误差。

## 末端摄像头外参

文件：`Engineer/APPLICATION/camera_target_transform_config.h`

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `CAMERA_TARGET_DEFAULT_CALIBRATED` | `0` | 当前未实测；必须保持0，避免占位参数驱动机械臂 |
| `CAMERA_TARGET_DEFAULT_REFERENCE_FRAME` | `TOOL_CENTER` | 摄像头随ID1运动时使用；若固定在小臂/ID1前则改为`WRIST_PITCH_AXIS` |
| `CAMERA_TARGET_DEFAULT_T_E_C_X/Y/Z_MM` | `[0,0,0]` | 摄像头光心在所选末端E系中的位置，单位mm |
| `CAMERA_TARGET_DEFAULT_R_E_C_00..22` | 单位矩阵占位 | 相机三根轴在末端E系中的方向 |
| `CAMERA_TARGET_DEFAULT_MAX_POSE_AGE_MS` | `5000 ms` | ArmTarget允许使用的最新拍照姿态年龄上限 |

不能只测相机到夹爪的三个距离后就把`CALIBRATED`改为1。还必须确认上位机
相机坐标的X/Y/Z方向，并填写`R_E_C`；它满足
`P_E=R_E_C*P_C+t_E_C`，矩阵三列依次为相机C-X/C-Y/C-Z轴在E系中的
单位方向。固件会拒绝非正交矩阵和行列式不是`+1`的镜像矩阵。

拍照触发处调用`UpperControllerCaptureCameraPose(capture_id, now_ms)`保存
当时姿态。当前协议`ArmTarget`没有帧ID，不能证明目标和快照一一对应，且
没有抓取绝对俯仰字段，所以即使变换成功也继续禁止动作。详细坐标系、明日
测量清单、Watch和离线测试见`docs/CAMERA_TARGET_TRANSFORM.md`。

## 机械臂点位和放置 profile

抓取流程通用参数在 `app_config.h`，A区实际点位/profile 在
`app_fruit_task_config.h`。当前点位：

| 点位 | 工具中心 `x/y/z` | 世界绝对俯仰 | 解析关节参考 |
|---|---|---|---|
| 业务点1，物理左侧 | `[0,400,-100] mm` | `-90 deg` | `[90,32.86,-101.44] deg` |
| 业务点2，物理右侧 | `[0,-400,-100] mm` | `-90 deg` | `[-90,32.86,-101.44] deg` |

坐标与绝对俯仰直接参与工具中心IK。物理左侧固定为 `Y>0`：点1对应
`q1=+90 deg`；物理右侧固定为 `Y<0`：点2对应 `q1=-90 deg`。

底座预对准由 `APP_ARM_PICK_BASE_AIM_MAX_ABS_Q1_DEG=89.5` 限制。同一条联合
命令让三台达妙同步到对应底座角和 `[q2,q3]=[80,-90]`，并让ID1同步到
相对俯仰 `-80 deg`；此时夹爪绝对俯仰为 `-90 deg`，不会贴住相对角
`-90 deg` 软件下限。工具中心命令随后以 `200 mm/s` 请求速度到
`+/-90 deg` 侧的抓取坐标；底层关节速度和加速度安全上限保持不变。
不要直接把预对准改为 `+/-90`，否则可能跨过 `X=0` 并触发跨区保护。

点1放置：`[90,90,-100] -> +135 -> +180 -> [q1反馈,120,-70]`；
点2放置：`[-90,90,-100] -> -135 -> -180 -> [q1反馈,120,-70]`。
释放 ID1 相对小臂均为 `-45 deg`。ID2张开后保持 `q2=120 deg`，小臂
从 `q3=-70 deg` 再上抬10deg到 `q3=-80 deg`，随后分别经 `+90/-90`
返回 `q1=0` 并立即结束放置，不再额外恢复 `[0,90,-100]`。下一次抓取的
底座预对准会同时进入 `[q2,q3]=[80,-90]` 准备姿态。释放与释放后抬臂
不重复提交后方 `q1=+180/-180`，而是保留定向旋转完成后的底座实际反馈角。

正式抓取与当前左右专项测试都必须调用 `AppArmFlowBuildPickStaging()`；
不要在某个测试状态机中复制反馈 `q2/q3` 后只覆盖 `q1`。实机已经证明，
从释放回正姿态 `[0,120,-80]` 只转到底座 `-89.5 deg`，下一条右侧直线会
在首采样点因 `X>2 mm` 且 `q2>120 deg` 被前方栏框保护拒绝。

放置参数以完整 `App_Arm_Place_Profile_s` 提交。更改某侧时应同时核对：

- 安全三轴位姿；
- 到放置点的引导角和终点角；
- 释放三轴位姿和ID1相对俯仰；
- 释放反馈等待超时；
- ID2释放后的小臂净空位姿；
- 返回前方的引导角和终点角。

## 主臂几何和安全

文件：`Engineer/APPLICATION/arm/arm_config.h`

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `ARM_BASE_HEIGHT_MM` | 62 mm | 安装基准到肩轴高度 |
| `ARM_LINK_1_MM` | 260 mm | 肩轴到肘轴中心距 |
| `ARM_LINK_2_MM` | 260 mm | 肘轴到ID1轴中心距 |
| `ARM_TOOL_PITCH_AXIS_TO_CENTER_MM` | 117 mm | ID1轴到夹爪中心 |
| `ARM_SAFE_Q1/Q2/Q3_DEG` | `[0,90,-60]` | HOME关节目标 |
| `ARM_Q1_SOFT_MIN/MAX_DEG` | `[-180,180]` | 关节命令软限位 |
| `ARM_Q2_SOFT_MIN/MAX_DEG` | `[3,180]` | 大臂普通运动软限位；`0..3 deg`仅允许单向脱困 |
| `ARM_Q3_SOFT_MIN/MAX_DEG` | `[-190,-35]` | 小臂软限位 |

连杆长度必须测转轴中心到转轴中心，不要用 HOME 坐标补偿杆长误差。
负 X、跨区高度和前方栏框限制仍由 `ARM_REAR_*`、
`ARM_FRONT_BARRIER_*` 参数控制，不要为了让单个点通过而放宽软限位。

完整上电顺序固定为 `q2大臂+q3小臂同步 -> q1底座 -> ID1/ID2`。Watch 中
`g_arm_dm_debug.auto_init.step=1, axis=4` 表示联合臂阶段，`step=2, axis=1`
表示底座阶段；两阶段完成后才进入 `ARM_BOOT_WAIT_TOOL` 初始化双舵机。

## ID1 和 ID2

文件：`Engineer/APPLICATION/arm/arm_config.h`

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `ARM_TOOL_PITCH_NEUTRAL_POS` | 500 | ID1与小臂同向 |
| `ARM_TOOL_PITCH_DIRECTION` | `-1.0` | 目标和反馈换算共用方向 |
| `ARM_TOOL_PITCH_SERVO_MIN/MAX_POS` | `[115,875]` | ID1允许控制值范围 |
| `ARM_TOOL_PITCH_RELATIVE_MIN/MAX_DEG` | `[-90,92.4]` | ID1相对小臂软件角范围 |
| `ARM_GRIPPER_DEFAULT_POS` | 450 | 上电、等待和张开位置 |
| `ARM_GRIPPER_CLOSE_POS` | 660 | 探测闭合目标 |
| `ARM_GRIPPER_FEEDBACK_RECOVERY_TIMEOUT_MS` | 500 ms | ID2张开/回等待位的短时反馈恢复窗口 |
| `ARM_GRIPPER_RELIEF_STEP_POS` | 10 | 每次向张开方向回退量 |
| `ARM_GRIPPER_RELIEF_MAX_ATTEMPTS` | 4 | 最多回退4次，累计最多40 |
| `ARM_GRIPPER_RELIEF_ATTEMPT_TIMEOUT_MS` | 400 ms | 单次回退等待上限 |

4 次均不能跟随时进入 `ARM_GRIPPER_FORCED_HELD` 并继续业务，不再进入会
卡死采摘任务的夹爪故障。这个状态只表示“容错按抓住处理”，不证明一定
抓到水果。ID2在张开或回等待位期间若反馈短时失效，会保持原目标并等待最多
`500 ms`；只有新鲜反馈恢复且确认到位后才继续放置，持续失联或原动作总截止
时间到期仍会使机械臂流程失败，不会把“无反馈”误判为释放成功。

主要 Watch：

- `g_arm_tool_debug.gripper_state`
- `g_arm_tool_debug.gripper_relief_attempt_count`
- `g_arm_tool_debug.gripper_forced_held_count`
- `g_arm_tool_debug.gripper_timeout_count`
- `g_arm_tool_debug.gripper_feedback_lost_tick`
- `g_arm_tool_debug.gripper_feedback_dropout_count`
- `g_arm_tool_debug.gripper_feedback_recovery_count`
- `g_arm_tool_debug.servo_feedback_valid[1]`
- `g_arm_tool_debug.servo_online[1]`

## 协议边界

当前生成协议哈希为 `0x2588BA9A`。生成文件为 `protocol.c/.h` 和
`PROTOCOL_DOC.md`；`protocol_runtime.*`、`protocol_port.*`、
`fruit_usb_bridge.*`、`upper_controller_bridge.*` 是工程维护层，更新生成文件
时不能覆盖。协议无需强制握手且心跳非严格；生成FSM自动回心跳和可靠入站ACK。

`StateMachineCommand`控制夹爪和双摄像头舵机，摄像头逻辑目标为向下`-45deg`
或向上`+45deg`，PWM提交后等待500ms再回完成。`VelocityCommand`已接连续底盘
接口。`ArmTarget`当前只记录相机坐标和`z_type`，未标定外参前不执行机械臂。
`FruitDetection`仍只更新`g_fruit_usb_debug`，不改变A区静态测试表。
