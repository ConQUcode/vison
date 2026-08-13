# vison 水果采摘控制工程

STM32F407、FreeRTOS 和 Keil MDK 工程。有效工程根目录为 `Engineer/`，
Keil 工程文件为 `Engineer/MDK-ARM/Engineer.uvprojx`。

当前默认固件在机械臂 HOME、底盘电机和 IMU 就绪后自动执行 A 区三次
抓放测试：

```text
物理向前585 mm -> A左抓放
-> 物理向前500 mm -> A右抓放
-> 物理向前500 mm -> A左抓放
-> DONE
```

`585 mm = 启动区到第一组500 mm + 车头到机械臂中心的一次性85 mm补偿`；
后续按 A 区相邻组间距各前进 `500 mm`。当前协议哈希为 `0x0EBAB184`，
上位机水果消息目前只更新观察数据，不直接改变上述静态任务表。

- [当前工程功能](docs/PROJECT_OVERVIEW.md)
- [A区任务和机械臂流程](docs/FRUIT_TASK_FLOW.md)
- [参数位置和调参说明](docs/TUNING_GUIDE.md)
- [下一版本交接](docs/HANDOFF.md)
- [生成协议定义](Engineer/MODULE/protocol/PROTOCOL_DOC.md)

运行模式在 `Engineer/APPLICATION/app_config.h` 中选择。文档用于交接和调参，
发生差异时以当前分支的实时源码和实机验证结果为准。
