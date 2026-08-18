# MCU ↔ ROS 通信协议文档

> **Auto-generated** — 由 `scripts/codegen.py` 根据 `config/protocol.yaml` 生成，请勿手动修改。

---

## 全局参数

| 参数                       | 值                                     |
| :------------------------- | :------------------------------------- |
| 波特率                     | `115200`                               |
| 帧头字节 1                 | `0x5a`                                 |
| 帧头字节 2                 | `0xa5`                                 |
| 校验算法                   | `CRC8`                                 |
| 强制握手                   | `否`                                   |
| 协议哈希（握手用）         | `0x740E426B`                           |
| 严格心跳模式               | `否`                                   |
| 心跳发送间隔（ROS 端发起） | `1000 ms`                              |
| 心跳超时时间               | `3000 ms`                              |
| 可靠传输重试间隔           | `100 ms`                               |
| 可靠传输重试告警阈值       | 每 `3 次`告警一次，收到 ACK 前持续重试 |

---

## 帧格式

每帧结构如下（小端序）：

|  字节位置   | 字段     | 说明                                      |
| :---------: | :------- | :---------------------------------------- |
|      0      | Header1  | 固定 `0x5a`                               |
|      1      | Header2  | 固定 `0xa5`                               |
|      2      | ID       | 消息 ID，见下表                           |
|      3      | Len      | 数据段字节数                              |
| 4 … 4+Len-1 | Data     | 各字段按结构体内存布局排列                |
|    4+Len    | Checksum | CRC8，覆盖 ID + Len + Data，多项式 `0x31` |

---

## MCU 集成注意事项

- 系统消息无需编写代码：心跳回包、握手回应、可靠消息 ACK 均由生成的协议状态机自动完成。
- 协议层会在 `protocol_fsm_feed()` 的调用上下文中发送回包。若 `protocol_fsm_feed()` 在串口中断中调用，而业务代码在主循环中调用 `send_xxx()`，`serial_write()` 必须自行保证可重入/并发安全（如关中断保护或发送队列）。
- 心跳由 ROS 端周期发起并在超时窗口内重发同一 `count`，MCU 侧对重复心跳原样回包即可（自动完成）。

---

## 电控 → ROS（电控主动发送）

### `ExecutionCallback` — ID `0x12`

- **ROS 话题**：`fruit_hardware_bridge/execution/callback`
- **ROS 消息类型**：`fruit_hardware_interfaces/msg/ExecutionCallback`
- **数据段字节数（Len）**：`2`
- **注意事项**：Lower-controller execution status for discrete hardware and observation-position actions.

| callback_id | Device or command         |
| :---------: | :------------------------ |
|      0      | gripper                   |
|      1      | camera gimbal             |
|      2      | side observation position |
|      3      | arm target                |
|      4      | QR recognition pose       |
|      5      | current area              |
|      6      | return initial pose       |

| callback_status | State     |
| :-------------: | :-------- |
|        0        | completed |
|        1        | executing |


| 字节偏移 | 字段名            | C 类型    | 字节数 |
| :------: | :---------------- | :-------- | :----: |
|    0     | `callback_id`     | `uint8_t` |   1    |
|    1     | `callback_status` | `uint8_t` |   1    |
|  **2**   | *(CRC8)*          | `uint8_t` |   1    |

### `Ack` — ID `0xfd`

- **数据段字节数（Len）**：`2`
- **注意事项**：框架内置：可靠消息的确认包，由协议层自动收发。

| 字节偏移 | 字段名     | C 类型    | 字节数 |
| :------: | :--------- | :-------- | :----: |
|    0     | `acked_id` | `uint8_t` |   1    |
|    1     | `ack_seq`  | `uint8_t` |   1    |
|  **2**   | *(CRC8)*   | `uint8_t` |   1    |

### `Heartbeat` — ID `0xfe`

- **数据段字节数（Len）**：`4`
- **注意事项**：框架内置：心跳包，对端原样回传 count。

| 字节偏移 | 字段名   | C 类型     | 字节数 |
| :------: | :------- | :--------- | :----: |
|    0     | `count`  | `uint32_t` |   4    |
|  **4**   | *(CRC8)* | `uint8_t`  |   1    |

### `Handshake` — ID `0xff`

- **数据段字节数（Len）**：`4`
- **注意事项**：框架内置：握手包，携带协议哈希。

| 字节偏移 | 字段名          | C 类型     | 字节数 |
| :------: | :-------------- | :--------- | :----: |
|    0     | `protocol_hash` | `uint32_t` |   4    |
|  **4**   | *(CRC8)*        | `uint8_t`  |   1    |

---

## ROS → 电控（电控被动接收）

### `StateMachineCommand` — ID `0x11`

- **ROS 话题**：`fruit_hardware_bridge/state_machine/command`
- **ROS 消息类型**：`fruit_hardware_interfaces/msg/StateMachineCommand`
- **数据段字节数（Len）**：`3`
- **注意事项**：Discrete hardware or observation-position command. Publish once; reliable transport handles delivery acknowledgement.

| task_id | Device or command         | task_status = 0     | task_status = 1 | task_status = 2 | task_status = 3 |
| :-----: | :------------------------ | :------------------ | :-------------- | :-------------- | :-------------- |
|    0    | gripper                   | grip                | open            | -               | -               |
|    1    | camera gimbal             | look down           | look up         | -               | -               |
|    2    | side observation position | left                | right           | -               | -               |
|    4    | QR recognition pose       | enter pose          | -               | -               | -               |
|    5    | current area              | A                   | B               | C               | D               |
|    6    | return initial pose       | return initial pose | -               | -               | -               |

- **可靠投递**：该消息启用 ACK/重传。ROS 端会在业务结构体 payload 后透明追加 1 字节 reliable seq；该字节参与 `Len` 和校验，但不属于 `Packet_xxx` 业务结构体字段。生成的分发逻辑会先调用 `on_receive_xxx()`，回调返回后再自动 `send_Ack()`。
- **投递语义**：ROS 节点进程存活期间的链路级 at-least-once。同 ID 消息按 FIFO 发送，ACK 丢失或串口断连重连时 ROS 端会持续重传到收到匹配 ACK；节点进程崩溃后的恢复不在保证范围内。`on_receive_xxx()` 可能被重复调用，回调实现需保证幂等。

| 字节偏移 | 字段名                                       | C 类型    | 字节数 |
| :------: | :------------------------------------------- | :-------- | :----: |
|    0     | `task_id`                                    | `uint8_t` |   1    |
|    1     | `task_status`                                | `uint8_t` |   1    |
|    2     | *(reliable seq，框架附加，非业务结构体字段)* | `uint8_t` |   1    |
|  **3**   | *(CRC8)*                                     | `uint8_t` |   1    |

### `ArmTarget` — ID `0x13`

- **ROS 话题**：`fruit_hardware_bridge/arm/target`
- **ROS 消息类型**：`fruit_hardware_interfaces/msg/ArmTarget`
- **数据段字节数（Len）**：`14`
- **注意事项**：One-shot arm target in the camera coordinate frame. target_x, target_y, and target_z use meters.
The lower controller reports execution with ExecutionCallback callback_id = 3.

| z_type |  Area  | Hook  | Layer |
| :----: | :----: | :---: | :---: |
|   0    | ground |   -   |   -   |
|   1    |   B    |   1   |   1   |
|   2    |   B    |   1   |   2   |
|   3    |   B    |   2   |   1   |
|   4    |   B    |   2   |   2   |
|   5    |   B    |   3   |   1   |
|   6    |   B    |   3   |   2   |
|   7    |   D    |   1   |   1   |
|   8    |   D    |   1   |   2   |
|   9    |   D    |   1   |   3   |
|   10   |   D    |   2   |   1   |
|   11   |   D    |   2   |   2   |
|   12   |   D    |   2   |   3   |
|   13   |   D    |   3   |   1   |
|   14   |   D    |   3   |   2   |
|   15   |   D    |   3   |   3   |

- **可靠投递**：该消息启用 ACK/重传。ROS 端会在业务结构体 payload 后透明追加 1 字节 reliable seq；该字节参与 `Len` 和校验，但不属于 `Packet_xxx` 业务结构体字段。生成的分发逻辑会先调用 `on_receive_xxx()`，回调返回后再自动 `send_Ack()`。
- **投递语义**：ROS 节点进程存活期间的链路级 at-least-once。同 ID 消息按 FIFO 发送，ACK 丢失或串口断连重连时 ROS 端会持续重传到收到匹配 ACK；节点进程崩溃后的恢复不在保证范围内。`on_receive_xxx()` 可能被重复调用，回调实现需保证幂等。

| 字节偏移 | 字段名                                       | C 类型    | 字节数 |
| :------: | :------------------------------------------- | :-------- | :----: |
|    0     | `target_x`                                   | `float`   |   4    |
|    4     | `target_y`                                   | `float`   |   4    |
|    8     | `target_z`                                   | `float`   |   4    |
|    12    | `z_type`                                     | `uint8_t` |   1    |
|    13    | *(reliable seq，框架附加，非业务结构体字段)* | `uint8_t` |   1    |
|  **14**  | *(CRC8)*                                     | `uint8_t` |   1    |

### `VelocityCommand` — ID `0x14`

- **ROS 话题**：`cmd_vel`
- **ROS 消息类型**：`geometry_msgs/msg/Twist`
- **数据段字节数（Len）**：`8`
- **注意事项**：Differential-drive command: linear_x in m/s; angular_z is negated before transmission to match the lower controller yaw direction.

| 字节偏移 | 字段名      | C 类型    | 字节数 |
| :------: | :---------- | :-------- | :----: |
|    0     | `linear_x`  | `float`   |   4    |
|    4     | `angular_z` | `float`   |   4    |
|  **8**   | *(CRC8)*    | `uint8_t` |   1    |

### `Ack` — ID `0xfd`

- **数据段字节数（Len）**：`2`
- **注意事项**：框架内置：可靠消息的确认包，由协议层自动收发。

| 字节偏移 | 字段名     | C 类型    | 字节数 |
| :------: | :--------- | :-------- | :----: |
|    0     | `acked_id` | `uint8_t` |   1    |
|    1     | `ack_seq`  | `uint8_t` |   1    |
|  **2**   | *(CRC8)*   | `uint8_t` |   1    |

### `Heartbeat` — ID `0xfe`

- **数据段字节数（Len）**：`4`
- **注意事项**：框架内置：心跳包，对端原样回传 count。
- **内置行为**：ROS 端每 `1000 ms` 发起一次心跳，协议层自动按原样回传同一个 `count` 作为 ACK，MCU 侧无需编写代码。

| 字节偏移 | 字段名   | C 类型     | 字节数 |
| :------: | :------- | :--------- | :----: |
|    0     | `count`  | `uint32_t` |   4    |
|  **4**   | *(CRC8)* | `uint8_t`  |   1    |

### `Handshake` — ID `0xff`

- **数据段字节数（Len）**：`4`
- **注意事项**：框架内置：握手包，携带协议哈希。

| 字节偏移 | 字段名          | C 类型     | 字节数 |
| :------: | :-------------- | :--------- | :----: |
|    0     | `protocol_hash` | `uint32_t` |   4    |
|  **4**   | *(CRC8)*        | `uint8_t`  |   1    |

---

*文档由构建系统自动生成，版本以协议哈希为准。*
