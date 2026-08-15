/**
 * @file protocol_runtime.h
 * @brief STM32端协议会话、心跳观察和可靠重试运行层。
 */

#ifndef PROTOCOL_RUNTIME_H
#define PROTOCOL_RUNTIME_H

#include <stdint.h>

#include "protocol.h"

#define PROTOCOL_RUNTIME_MAX_PAYLOAD_LEN 32u
#define PROTOCOL_RUNTIME_FIFO_DEPTH       4u

typedef struct {
    uint8_t connection_ready;        /* 协议哈希匹配并完成握手。 */
    uint8_t link_online;             /* 最近 3000 ms 内收到过心跳。 */
    uint8_t reliable_queue_count;
    uint8_t last_rx_id;
    uint8_t last_ack_id;
    uint8_t last_ack_seq;
    uint32_t session_count;
    uint32_t now_ms;
    uint32_t last_heartbeat_count;
    uint32_t last_heartbeat_rx_ms;
    uint32_t heartbeat_age_ms;
    uint32_t handshake_ok_count;
    uint32_t handshake_mismatch_count;
    uint32_t heartbeat_rx_count;
    uint32_t heartbeat_timeout_count;
    uint32_t ack_rx_count;
    uint32_t reliable_tx_count;
    uint32_t reliable_retry_count;
    uint32_t reliable_warning_count;
    uint32_t reliable_drop_count;
    uint32_t prehandshake_drop_count;
    uint32_t connection_reset_count;
    uint32_t tx_fail_count;
} Protocol_Runtime_Debug_s;

extern Protocol_Runtime_Debug_s g_protocol_runtime_debug;

/** 初始化会话和可靠发送队列；不初始化 USB 外设。 */
void ProtocolRuntimeInit(void);
/** 由 USB 任务每 1 ms 调用，监督心跳并推进可靠重试。 */
void ProtocolRuntimeTask(uint32_t now_ms);
/** 将 USB 收到的单字节送入上位机生成的协议解析器。 */
void ProtocolRuntimeFeedByte(uint8_t byte);
/** USB 断开时撤销会话并清空可靠队列。 */
void ProtocolRuntimeResetConnection(void);
/** 业务回调收到一帧合法协议包时调用，用于非强制握手模式建立在线状态。 */
void ProtocolRuntimeNotifyApplicationRx(void);
uint8_t ProtocolRuntimeConnectionReady(void);
uint8_t ProtocolRuntimeLinkOnline(void);
uint32_t ProtocolRuntimeNowMs(void);
const Protocol_Runtime_Debug_s *ProtocolRuntimeGetDebug(void);

/** 为后续可靠业务保留；当前 FruitDetection 不调用该接口。 */
uint8_t ProtocolRuntimeSendReliable(uint8_t id,
                                    const void *payload,
                                    uint8_t payload_len);

#endif
