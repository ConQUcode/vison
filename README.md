# vison 水果采摘控制工程

STM32F407、FreeRTOS 和 Keil MDK 工程。有效工程根目录为 `Engineer/`，
Keil 工程文件为 `Engineer/MDK-ARM/Engineer.uvprojx`。

当前默认固件已切回 `APP_MODE_HOST_CONTROL`。上电完成机械臂HOME并初始化USB、
底盘、IMU、ID2夹爪和双MG995后，上位机协议ID `0x14` 的 `VelocityCommand`
会提交给现有连续底盘速度接口：

```text
linear_x：float，单位m/s，执行范围[-1.0,+1.0]
angular_z：float，单位rad/s，执行范围[-1.5,+1.5]
建议发送频率：>=10 Hz
300 ms未收到新速度包：底盘平滑停车并进入CANCELLED
```

`linear_x` 在协议桥内乘1000转换为 `vx_mm_s`；`angular_z` 保持
`wz_rad_s`。有限的超范围速度不会被拒绝，而是分别按正负上限饱和；
NaN/Inf仍会被拒绝。纯平移时固件使用IMU保持提交瞬间的当前航向，非零
`angular_z` 时以上位机角速度为主。USB链路必须在线。

AC 区（A/C 两区）左右镜像抓放流程作为开环备选，仍独立保留在
`APP_MODE_ARM_POSTURE_TEST`：

```text
左侧：夹爪中心=[0,380,-140] -> [0,440,-140] mm
右侧：夹爪中心=[0,-380,-140] -> [0,-440,-140] mm
两侧夹爪世界绝对俯仰=-5 deg
接近点请求速度=600 mm/s，最后60 mm推进速度=150 mm/s
左抓取/左放置 -> 返回前方 -> 右抓取/右放置 -> 返回前方 -> 重复
```

抓取完成后的放置准备不再从终点直接插值到安全姿态；左右先经过
`[q1=+/-90,q2=27.3,q3=-62.7] deg`，再到`[+/-90,90,-100] deg`。
第一段工具中心从`Z=-140`单调抬到约`-120 mm`，实际回放抬高
`19.993 mm`；完整两段最大`|Y|=440 mm`。固件每次提交前会按实际关节
和ID1反馈重算Y/Z约束，超限或Z下探时拒绝运动，不继续转向后方。
该waypoint和Y/Z约束只在`app_arm_side_pick_place`取得公共A左/A右profile
副本后注入；`app_fruit_task`中的正式A区公共profile保持开关关闭，仍直接
进入原安全姿态，不受这组AC专用参数影响。

闭环方案的AC/BD左右观察点已保存：LEFT为
`q1=+90 deg/[0,+150,300] mm`，RIGHT为严格镜像
`q1=-90 deg/[0,-150,300] mm`，绝对俯仰均为`-58 deg`，斜向对应侧观察。
当前活动侧为LEFT；同步 staging 为`q=[+90,90,-80] deg`、ID1相对俯仰
`-48 deg`，完整回放最低`Z=77.557 mm`、
`max|Y|=318.051 mm < 405 mm`。405mm只属于BD观察回放；AC继续独立使用
440mm抓取终点和445mm抓后约束。本轮只确认观察姿态，相机通信和视觉目标
执行尚未接入，且不得覆盖AC开环备选参数。
BD左右观察姿态已经确认，后续上下位机协议只负责选择LEFT/RIGHT；观察点坐标、
俯仰和staging继续只在`app_config.h`维护，协议桥不得保存第二份参数。
`APP_MODE_MG995_TEST` 仍保留用于双摄像头舵机方向确认。

一侧完整动作已封装在 `app_arm_side_pick_place.c/.h`，作为AC区开环备选保留。
新版上位机协议已同步到哈希 `0x740E426B`；协议已移除旧 `FruitDetection`，
保留底盘速度、夹爪/摄像头离散命令、相机坐标机械臂目标和执行回调。当前
`APP_MODE_HOST_CONTROL` 中，`VelocityCommand` 会进入现有 `vx/wz` 底盘接口；
`StateMachineCommand`除控制ID2夹爪及双MG995外，还支持`task_id=2`触发AC闭环
观察姿态：status 0进入左观察，status 1进入右观察，status 2在闭环链路中无效。
该观察任务使用`ExecutionCallback(callback_id=2)`报告执行中1和到位完成0；
完成时下位机保存拍照姿态快照，供下一帧`ArmTarget`使用。
`task_id=5`已接入当前区域：status 0/1/2/3对应A/B/C/D，下位机幂等保存后
发送`callback_id=5`的执行中1和完成0；可通过`UpperControllerGetCurrentArea()`
读取。`task_id=4`虽然在线协议中命名为二维码识别姿态，但当前没有任何已确认
点位，因此只记录`qr_pose_unsupported_count`，不动作也不伪报callback 4完成。
本版协议仍没有BD观察LEFT/RIGHT选择字段，区域消息不会自动驱动观察点。

`ArmTarget` 已接入完整的 `T_B_E * T_E_C * P_C` 点变换算法。当前D435i外参
已按工具中心参考系启用：相机固定在ID1之后并随夹爪俯仰运动；观察姿态到位后
由 `UpperControllerCaptureCameraPose()` 保存当时的机械臂姿态。只有在
`task_id=2`观察完成并处于保持状态、快照未超过5000ms、外参/矩阵/目标点均校验
通过且机械臂空闲时，`ArmTarget` 才会启动AC闭环抓取：抓取终点`X/Y`取换算后的
基座坐标，`Z`固定为`-100 mm`，夹爪世界绝对俯仰固定为竖直向下`-90 deg`。
`ExecutionCallback(callback_id=3)`报告闭环抓取执行中1和完成0；协议可靠ACK
仍只表示下位机收到该包，不代表机械臂动作完成。一次`ArmTarget`成功启动后会消费
当前观察状态，下一次闭环抓取必须重新观察。坐标系和Watch见坐标转换文档。

每次开始当前侧抓取时都先执行同一条联合准备命令：底座到对应
`q1=+/-89.5 deg`，同时大臂/小臂到 `[q2,q3]=[80,-90] deg`，ID1到
相对小臂 `-80 deg`；放置回正后的下一侧也不例外，不能只旋转底座。

完整上电初始化固定为大臂q2和小臂q3通过同一联合位姿命令同步到HOME，
随后底座q1到HOME，最后才初始化ID1/ID2；同步带耦合补偿仍在联合命令中生效。

正式A区点1/点2、AC开环抓放、BD左右观察点和带底盘任务表均仍保留；
`APP_MODE_ARM_BD_OBSERVATION_TEST` 仍可手动切回用于单次左侧BD观察姿态测试。

- [当前工程功能](docs/PROJECT_OVERVIEW.md)
- [A区任务和机械臂流程](docs/FRUIT_TASK_FLOW.md)
- [末端摄像头目标坐标转换](docs/CAMERA_TARGET_TRANSFORM.md)
- [参数位置和调参说明](docs/TUNING_GUIDE.md)
- [下一版本交接](docs/HANDOFF.md)
- [生成协议定义](Engineer/MODULE/protocol/PROTOCOL_DOC.md)

运行模式在 `Engineer/APPLICATION/app_config.h` 中选择。文档用于交接和调参，
发生差异时以当前分支的实时源码和实机验证结果为准。
