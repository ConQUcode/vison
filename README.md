# vison 水果采摘控制工程

STM32F407、FreeRTOS 和 Keil MDK 工程。有效工程根目录为 `Engineer/`，
Keil 工程文件为 `Engineer/MDK-ARM/Engineer.uvprojx`。

当前默认固件为 `APP_MODE_ARM_BD_OBSERVATION_TEST`。正常 HOME 后，
机械臂先让底座和主臂同步进入紧凑右侧过渡姿态，再以 `150 mm/s`
直线移动到 BD 区右侧树上水果观察位并保持：

```text
已保存LEFT： q1=+90 deg，夹爪中心=[0,+57,210] mm
已保存RIGHT：q1=-90 deg，夹爪中心=[0,-57,210] mm
当前ACTIVE_SIDE=RIGHT
```

```text
HOME -> 过渡关节=[-90,120,-48] deg，第一段工具中心最大|Y|=231.470 mm
最终夹爪中心=[0,-57,210] mm，夹爪中心线世界绝对俯仰=-30 deg
两段离线回放约束：工具中心|Y|<=260 mm
不运行底盘，不闭合夹爪，不进入AC左右循环
```

AC 区（A/C 两区）左右镜像抓放流程仍独立保留在
`APP_MODE_ARM_POSTURE_TEST`：

```text
左侧：夹爪中心=[0,380,-150] -> [0,480,-150] mm
右侧：夹爪中心=[0,-380,-150] -> [0,-480,-150] mm
两侧夹爪世界绝对俯仰=-5 deg
接近点请求速度=600 mm/s，最后100 mm推进速度=150 mm/s
左抓取/左放置 -> 返回前方 -> 右抓取/右放置 -> 返回前方 -> 重复
```

BD 区目前只配置了左侧观察位第一阶段。左右摄像头选边、机械臂
摄像头二次定位、抓取点和放置profile仍未接入，后续不得覆盖
AC 参数。`APP_MODE_MG995_TEST` 仍保留用于双摄像头舵机方向确认。

一侧完整动作已封装在 `app_arm_side_pick_place.c/.h`。新版上位机协议已同步到
哈希 `0x2588BA9A`，并增加底盘速度、夹爪/摄像头离散命令、相机坐标机械臂
目标和执行回调。切到 `APP_MODE_HOST_CONTROL` 后，`VelocityCommand` 会进入
现有 `vx/wz` 底盘接口，`StateMachineCommand` 可控制ID2夹爪及双MG995；
只有切到 `APP_MODE_HOST_CONTROL` 后才运行该上位机控制链。

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

正式A区点1/点2、AC抓放和带底盘任务表均仍保留；当前 BD
观察位专项模式不运行底盘。

- [当前工程功能](docs/PROJECT_OVERVIEW.md)
- [A区任务和机械臂流程](docs/FRUIT_TASK_FLOW.md)
- [末端摄像头目标坐标转换](docs/CAMERA_TARGET_TRANSFORM.md)
- [参数位置和调参说明](docs/TUNING_GUIDE.md)
- [下一版本交接](docs/HANDOFF.md)
- [生成协议定义](Engineer/MODULE/protocol/PROTOCOL_DOC.md)

运行模式在 `Engineer/APPLICATION/app_config.h` 中选择。文档用于交接和调参，
发生差异时以当前分支的实时源码和实机验证结果为准。
