# vison 水果采摘控制工程

STM32F407、FreeRTOS 和 Keil MDK 工程。有效工程根目录为 `Engineer/`，
Keil 工程文件为 `Engineer/MDK-ARM/Engineer.uvprojx`。

当前默认固件是双MG995方向确认专项测试：`TIM8_CH2/PI6`控制右侧舵机，
`TIM8_CH3/PI7`控制左侧舵机。以舵机 `90 deg` 为摄像头水平 `0 deg`，当前
两只摄像头都恢复到水平 `0 deg`，左右舵机均为 `90 deg/1500 us`。
`APP_MODE=APP_MODE_MG995_TEST` 时不初始化机械臂、底盘或IMU，并禁止达妙和
DJI电机周期控制。通过 `g_mg995_servo_debug` 确认初始化和两路目标。

侧向夹爪左右镜像交替抓放代码完整保留；切回
`APP_MODE_ARM_POSTURE_TEST` 后仍按以下已验证流程运行：

```text
左侧：夹爪中心=[0,340,-150] -> [0,480,-150] mm
右侧：夹爪中心=[0,-340,-150] -> [0,-480,-150] mm
两侧夹爪世界绝对俯仰=-5 deg
请求速度=100 mm/s
左抓取/左放置 -> 返回前方 -> 右抓取/右放置 -> 返回前方 -> 重复
```

一侧完整动作已封装在 `app_arm_side_pick_place.c/.h`。新版上位机协议已同步到
哈希 `0x2588BA9A`，并增加底盘速度、夹爪/摄像头离散命令、相机坐标机械臂
目标和执行回调。切到 `APP_MODE_HOST_CONTROL` 后，`VelocityCommand` 会进入
现有 `vx/wz` 底盘接口，`StateMachineCommand` 可控制ID2夹爪及双MG995；当前
默认仍为MG995台架模式，不会启动USB、底盘或机械臂。

`ArmTarget` 已接入完整的 `T_B_E * T_E_C * P_C` 点变换算法，可分别使用
ID1轴心/小臂俯仰或夹爪中心/ID1绝对俯仰作为摄像头安装参考系。当前外参配置
明确保持 `calibrated=0`，协议也没有图像帧ID，因此只记录原始相机点和明确的
变换失败原因，不会提交机械臂；协议可靠ACK只表示下位机收到该包。明日测量
摄像头光心偏移和三根相机轴方向后的填写方式见坐标转换文档。

每次开始当前侧抓取时都先执行同一条联合准备命令：底座到对应
`q1=+/-89.5 deg`，同时大臂/小臂到 `[q2,q3]=[80,-90] deg`，ID1到
相对小臂 `-80 deg`；放置回正后的下一侧也不例外，不能只旋转底座。

完整上电初始化固定为大臂q2和小臂q3通过同一联合位姿命令同步到HOME，
随后底座q1到HOME，最后才初始化ID1/ID2；同步带耦合补偿仍在联合命令中生效。

正式A区点1/点2、左右专项抓放和带底盘任务表均仍保留；本次只临时切换
应用模式，不修改任何机械臂点位或流程参数。

- [当前工程功能](docs/PROJECT_OVERVIEW.md)
- [A区任务和机械臂流程](docs/FRUIT_TASK_FLOW.md)
- [末端摄像头目标坐标转换](docs/CAMERA_TARGET_TRANSFORM.md)
- [参数位置和调参说明](docs/TUNING_GUIDE.md)
- [下一版本交接](docs/HANDOFF.md)
- [生成协议定义](Engineer/MODULE/protocol/PROTOCOL_DOC.md)

运行模式在 `Engineer/APPLICATION/app_config.h` 中选择。文档用于交接和调参，
发生差异时以当前分支的实时源码和实机验证结果为准。
