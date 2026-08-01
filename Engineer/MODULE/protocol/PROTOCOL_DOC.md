# 拼图装置 USB CDC 通信协议

当前固件按上位机最新协议实现，帧格式为：

```text
5A A5 | ID | Len | Payload | CRC8
```

- 多字节字段小端序。
- `float` 为 IEEE-754 `float32`。
- CRC8 多项式 `0x31`，初值 `0x00`。
- CRC 覆盖 `ID + Len + Payload`，不包含 `5A A5`。
- 可靠 payload 末尾附加透明 `ack_seq:u8`。
- `PROTOCOL_HASH = 0x8845D84A`。

## 消息

| ID | 名称 | 方向 | 可靠 | Payload |
|---:|---|---|:---:|---|
| `0x01` | `TaskStatus` | 上位机→下位机 | 是 | `task_id:u8`, `task_status:u8` |
| `0x02` | `CallbackStatus` | 下位机→上位机 | 是 | `callback_id:u8`, `callback_status:u8` |
| `0x04` | `TargetControl` | 上位机→下位机 | 是 | `x_mm:f32`, `y_mm:f32`, `yaw_deg:f32` |
| `0x05` | `MotionStatus` | 下位机→上位机 | 是 | `state:u8`, `fault:u8`, `x_mm:f32`, `y_mm:f32`, `yaw_deg:f32` |
| `0xFD` | `Ack` | 双向 | 否 | `acked_id:u8`, `ack_seq:u8` |
| `0xFE` | `Heartbeat` | 双向 | 否 | `count:u32` |
| `0xFF` | `Handshake` | 双向 | 否 | `protocol_hash:u32` |

`Ack` 只表示可靠帧已收到，不表示机械动作完成。动作结果由
`MotionStatus` 或 `CallbackStatus` 表达。

## 结构体尺寸

```c
sizeof(Packet_TaskStatus)     == 2
sizeof(Packet_CallbackStatus) == 2
sizeof(Packet_TargetControl)  == 12
sizeof(Packet_MotionStatus)   == 14
sizeof(Packet_Ack)            == 2
sizeof(Packet_Heartbeat)      == 4
sizeof(Packet_Handshake)      == 4
```

可靠 `TargetControl` 的 `Len=13`，整帧长度为 `18` 字节。  
可靠 `MotionStatus` 的 `Len=15`，整帧长度为 `20` 字节。

## TargetControl

上位机发送：

```c
typedef struct {
    float x_mm;
    float y_mm;
    float yaw_deg;
} Packet_TargetControl;
```

固件解释为电磁铁吸取点目标。Z 不在线协议中发送，而是按目标 X
使用固件标定的线性默认高度：

```text
X = x_mm
Y = y_mm
X <= 240mm: Z = 32mm
X >= 450mm: Z = 35mm
240mm < X < 450mm: Z在32mm到35mm之间线性插值
yaw = yaw_deg
```

下位机执行 `TOOL_TIP + LINEAR` 运动，ID1 保持竖直补偿，ID2 执行
`yaw_deg`。执行状态用可靠 `MotionStatus` 上报：

```text
ACCEPTED -> RUNNING -> COMPLETED
```

失败时上报：

```text
FAILED + fault
```

## TaskStatus

```c
typedef struct {
    uint8_t task_id;
    uint8_t task_status;
} Packet_TaskStatus;
```

`task_status`：

```text
0 = END
1 = START
2 = FAULT_RETRY
3 = STOP
```

固件业务语义：

| TaskStatus | 下位机行为 | 完成回传 |
|---|---|---|
| `{0, START}` | HOME 到固件等待点 | `CallbackStatus{0, END}` |
| `{5, START}` | 当前 XY 下降到按X线性标定的抓取高度，开启电磁铁，等待配置时间，再回按X标定的默认高度 | `CallbackStatus{5, END}` |
| `{6, START}` | 当前 XY 下降到按X线性标定的释放高度，关闭电磁铁，等待配置时间，再回按X标定的默认高度 | `CallbackStatus{6, END}` |
| `{1..4, START}` | 只记录当前任务 ID，不自动运动 | 无 |
| `{1..4, END}` | 任务结束提示，蜂鸣器响 3s | 无 |
| `{*, FAULT_RETRY}` | 清除桥接层上一条失败标志，不自动重放动作 | 无 |
| `{当前任务, STOP}` | 取消当前动作，保持电磁铁状态回 HOME，到位后关闭电磁铁并清空任务状态 | `CallbackStatus{当前任务, STOP}` |

`STOP` 是幂等安全动作。可靠包使用相同 `ack_seq` 重传时只重新 ACK；若
上位机使用新序号重复发送 STOP，已经处于安全 HOME 状态的下位机会保持
电磁铁关闭并重新发送 `CallbackStatus{task_id, STOP}`，不会重复运动。

STOP 回 HOME 失败时不会提前关闭电磁铁，也不会回 STOP 完成，而是回
`CallbackStatus{task_id, FAULT_RETRY}`。

## MotionStatus

```c
typedef struct {
    uint8_t state;
    uint8_t fault;
    float x_mm;
    float y_mm;
    float yaw_deg;
} Packet_MotionStatus;
```

`state`：

```text
1 = ACCEPTED
2 = RUNNING
3 = COMPLETED
4 = FAILED
```

`fault`：

```text
0 = NONE
1 = IK_UNREACHABLE
2 = JOINT_LIMIT
3 = TIMEOUT
4 = COLLISION
5 = GRIP_LOST
6 = LINK_LOSS
7 = ESTOP
```

当前固件没有吸取检测传感器，不主动产生 `GRIP_LOST`。

## Heartbeat

心跳由上位机发送，下位机收到正确 `Heartbeat{count}` 后原样回显：

```text
Host -> Board: Heartbeat{count}
Board -> Host: Heartbeat{same count}
```

固件本轮不再主动周期发送心跳，也不再用严格心跳超时自动取消机械臂动作。

## 可靠包规则

- 收到可靠 `TaskStatus` 或 `TargetControl` 后立即发送 `Ack`。
- 收到重复 `ack_seq` 时重新 ACK，但不重复交付业务。
- `MotionStatus` 和 `CallbackStatus` 等待上位机 ACK。
- 未收到 ACK 时每 `100ms` 重试一次，最多重试 `3` 次。
