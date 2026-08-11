# MCU <-> ROS USB CDC通信协议

本文件同步自上位机生成协议 `0x923FFDD9`。当前工程通过USB CDC传输，
生成配置中的`115200`仅适用于物理串口，不改变USB CDC速率。

## 全局参数

| 参数 | 值 |
|---|---|
| 帧头 | `5A A5` |
| 字节序 | 小端 |
| CRC | CRC8，多项式`0x31`，初值`0x00` |
| CRC范围 | `ID + Len + Payload` |
| 协议哈希 | `0x923FFDD9` |
| 强制握手 | 是 |
| 严格心跳 | 是，超时`3000 ms` |
| 可靠重试基础参数 | `100 ms`，最多`3`次 |

帧格式：

```text
5A A5 | ID:u8 | Len:u8 | Payload:Len bytes | CRC8:u8
```

当前活动消息只有：

| ID | 名称 | Payload | 方向 |
|---|---|---|---|
| `0x10` | FruitDetection | `fruit_id:u8, status:u8` | ROS -> MCU |
| `0xFD` | Ack | `acked_id:u8, ack_seq:u8` | 双向框架消息 |
| `0xFE` | Heartbeat | `count:u32` | ROS发送，MCU原样回显 |
| `0xFF` | Handshake | `protocol_hash:u32` | ROS发送，匹配时MCU原样回显 |

## 握手与心跳

- MCU上电或USB重连后处于未握手状态。
- 未握手时只有`Handshake`可以进入业务分发，其他消息均忽略。
- 哈希匹配时MCU回显相同Handshake并建立新会话。
- 哈希不匹配时不回显、不进入连接状态，并记录版本错误。
- 握手成功后，ROS必须在3000 ms内持续发送Heartbeat。
- MCU收到Heartbeat后原样回显同一个`count`并刷新在线时间。
- 连续3000 ms没有Heartbeat时会话失效，旧水果结果不可再使用。
- 心跳失联不会让本测试固件打开夹爪、回HOME或中断独立上电HOME。

## FruitDetection

`FruitDetection`固定`Len=2`，不带`ack_seq`，MCU不会为它发送ACK。
重复识别帧只刷新最新Watch快照，本阶段不会触发机械臂或夹爪动作。

| fruit_id | 含义 |
|---:|---|
| 0 | 无目标，要求`status=0` |
| 1 | 苹果 |
| 2 | 辣椒 |
| 3 | 南瓜 |
| 4 | 洋葱 |
| 5 | 梨 |
| 6 | 西红柿 |

| status | 含义 |
|---:|---|
| 0 | 未成熟 |
| 1 | 成熟 |

合法业务结果为`fruit_id=1..6`且`status=0..1`。`{0,0}`表示无目标并
使当前结果无效；`fruit_id=0,status!=0`、未知水果ID或未知状态均拒绝。

## 当前机械臂测试语义

- 新协议不再包含旧Task 0..6、Cartesian目标、ToolControl或动作状态包。
- 机械臂独立完成三台CAN1达妙、两台USART6舵机初始化和HOME。
- HOME控制点是ID1俯仰舵机轴心，目标约为`(225.1666,0,192.0) mm`。
- 物理+X方向由底座朝向目标侧后通过达妙上位机保存的零点定义。
- 固件不发送达妙清零命令，也不在FK/IK中增加X反号或180度偏置。
