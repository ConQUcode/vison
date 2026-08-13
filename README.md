# vison 控制工程

STM32F407 + FreeRTOS + Keil 工程。当前默认固件运行完整机械臂，并在 HOME 后先将 ID1 调到水平，再把夹爪中心直线移动到 `(250,0,200) mm` 后保持；底盘和 IMU 当前不初始化。

- [当前工程功能](docs/PROJECT_OVERVIEW.md)
- [参数位置和调参说明](docs/TUNING_GUIDE.md)
- [下一会话交接](docs/HANDOFF.md)
- [上位机生成协议](Engineer/MODULE/protocol/PROTOCOL_DOC.md)

运行模式在 `Engineer/APPLICATION/app_config.h` 中选择。Keil 工程为 `Engineer/MDK-ARM/Engineer.uvprojx`。
