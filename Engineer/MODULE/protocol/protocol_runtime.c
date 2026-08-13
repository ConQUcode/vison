/**
 * @file protocol_runtime.c
 * @brief 管理协议握手、心跳会话和可靠消息，不依赖具体业务消息定义。
 */

#include "protocol_runtime.h"

#include "protocol_port.h"
#include "usb.h"

#include <string.h>

#define PROTOCOL_HEARTBEAT_TIMEOUT_MS 3000u
#define PROTOCOL_RETRY_INTERVAL_MS      100u
#define PROTOCOL_MAX_RETRIES              3u

typedef struct {
    uint8_t id;
    uint8_t payload[PROTOCOL_RUNTIME_MAX_PAYLOAD_LEN - 1u];
    uint8_t payload_len;
    uint8_t sequence;
    uint8_t retries;
    uint8_t sent;
    uint32_t last_tx_ms;
} Protocol_Reliable_Slot_s;

Protocol_Runtime_Debug_s g_protocol_runtime_debug;

static Protocol_Reliable_Slot_s reliable_fifo[PROTOCOL_RUNTIME_FIFO_DEPTH];
static uint8_t reliable_head;
static uint8_t reliable_count;
static uint8_t reliable_next_sequence;

static void ProtocolRuntimeClearReliable(void)
{
    memset(reliable_fifo, 0, sizeof(reliable_fifo));
    reliable_head = 0u;
    reliable_count = 0u;
    g_protocol_runtime_debug.reliable_queue_count = 0u;
}

static void ProtocolRuntimeExpireSession(void)
{
    g_protocol_runtime_debug.connection_ready = 0u;
    g_protocol_runtime_debug.link_online = 0u;
    ProtocolRuntimeClearReliable();
}

static uint8_t ProtocolRuntimeSendFrame(uint8_t id,
                                        const void *payload,
                                        uint8_t payload_len,
                                        int16_t sequence)
{
    uint8_t frame[4u + PROTOCOL_RUNTIME_MAX_PAYLOAD_LEN + 1u];
    uint8_t wire_len = payload_len;
    uint16_t index = 0u;

    if ((payload_len != 0u && payload == NULL) ||
        payload_len > PROTOCOL_RUNTIME_MAX_PAYLOAD_LEN -
            (sequence >= 0 ? 1u : 0u)) {
        return 0u;
    }
    if (sequence >= 0) {
        wire_len++;
    }
    frame[index++] = FRAME_HEADER1;
    frame[index++] = FRAME_HEADER2;
    frame[index++] = id;
    frame[index++] = wire_len;
    if (payload_len != 0u) {
        memcpy(&frame[index], payload, payload_len);
        index = (uint16_t)(index + payload_len);
    }
    if (sequence >= 0) {
        frame[index++] = (uint8_t)sequence;
    }
    frame[index] = calculate_checksum(&frame[2], index - 2u);
    index++;
    if (USB_ProtocolWrite(frame, index, 0u) == 0u) {
        g_protocol_runtime_debug.tx_fail_count++;
        return 0u;
    }
    return 1u;
}

/* 以下强回调覆盖生成文件中的弱实现，系统消息的业务规则集中在此处。 */
void on_receive_Handshake(const Packet_Handshake *packet)
{
    if (packet == NULL || packet->protocol_hash != PROTOCOL_HASH) {
        g_protocol_runtime_debug.handshake_mismatch_count++;
        ProtocolRuntimeExpireSession();
        return;
    }
    ProtocolRuntimeClearReliable();
    g_protocol_runtime_debug.connection_ready = 1u;
    g_protocol_runtime_debug.link_online = 1u;
    g_protocol_runtime_debug.last_heartbeat_rx_ms =
        g_protocol_runtime_debug.now_ms;
    g_protocol_runtime_debug.heartbeat_age_ms = 0u;
    g_protocol_runtime_debug.handshake_ok_count++;
    g_protocol_runtime_debug.session_count++;
    send_Handshake(packet);
    if (ProtocolPortLastWriteOk() == 0u) {
        g_protocol_runtime_debug.tx_fail_count++;
    }
}

void on_receive_Heartbeat(const Packet_Heartbeat *packet)
{
    if (packet == NULL ||
        g_protocol_runtime_debug.connection_ready == 0u) {
        g_protocol_runtime_debug.prehandshake_drop_count++;
        return;
    }
    g_protocol_runtime_debug.last_heartbeat_count = packet->count;
    g_protocol_runtime_debug.last_heartbeat_rx_ms =
        g_protocol_runtime_debug.now_ms;
    g_protocol_runtime_debug.heartbeat_age_ms = 0u;
    g_protocol_runtime_debug.link_online = 1u;
    g_protocol_runtime_debug.heartbeat_rx_count++;
    send_Heartbeat(packet);
    if (ProtocolPortLastWriteOk() == 0u) {
        g_protocol_runtime_debug.tx_fail_count++;
    }
}

void on_receive_Ack(const Packet_Ack *packet)
{
    Protocol_Reliable_Slot_s *slot;

    if (packet == NULL ||
        g_protocol_runtime_debug.connection_ready == 0u) {
        return;
    }
    g_protocol_runtime_debug.ack_rx_count++;
    g_protocol_runtime_debug.last_ack_id = packet->acked_id;
    g_protocol_runtime_debug.last_ack_seq = packet->ack_seq;
    if (reliable_count == 0u) {
        return;
    }
    slot = &reliable_fifo[reliable_head];
    if (slot->id == packet->acked_id &&
        slot->sequence == packet->ack_seq) {
        reliable_head = (uint8_t)((reliable_head + 1u) %
                                  PROTOCOL_RUNTIME_FIFO_DEPTH);
        reliable_count--;
        g_protocol_runtime_debug.reliable_queue_count = reliable_count;
    }
}

void ProtocolRuntimeInit(void)
{
    memset(&g_protocol_runtime_debug, 0,
           sizeof(g_protocol_runtime_debug));
    reliable_next_sequence = 0u;
    ProtocolRuntimeClearReliable();
}

void ProtocolRuntimeFeedByte(uint8_t byte)
{
    protocol_fsm_feed(byte);
}

void ProtocolRuntimeResetConnection(void)
{
    ProtocolRuntimeExpireSession();
    g_protocol_runtime_debug.connection_reset_count++;
}

uint8_t ProtocolRuntimeConnectionReady(void)
{
    return g_protocol_runtime_debug.connection_ready;
}

uint8_t ProtocolRuntimeLinkOnline(void)
{
    return g_protocol_runtime_debug.link_online;
}

uint32_t ProtocolRuntimeNowMs(void)
{
    return g_protocol_runtime_debug.now_ms;
}

const Protocol_Runtime_Debug_s *ProtocolRuntimeGetDebug(void)
{
    return &g_protocol_runtime_debug;
}

uint8_t ProtocolRuntimeSendReliable(uint8_t id,
                                    const void *payload,
                                    uint8_t payload_len)
{
    Protocol_Reliable_Slot_s *slot;
    uint8_t tail;

    if (g_protocol_runtime_debug.connection_ready == 0u ||
        payload == NULL || payload_len == 0u ||
        payload_len > PROTOCOL_RUNTIME_MAX_PAYLOAD_LEN - 1u ||
        id == PACKET_ID_FRUITDETECTION || id >= PACKET_ID_ACK ||
        reliable_count >= PROTOCOL_RUNTIME_FIFO_DEPTH) {
        return 0u;
    }
    tail = (uint8_t)((reliable_head + reliable_count) %
                     PROTOCOL_RUNTIME_FIFO_DEPTH);
    slot = &reliable_fifo[tail];
    slot->id = id;
    memcpy(slot->payload, payload, payload_len);
    slot->payload_len = payload_len;
    slot->sequence = reliable_next_sequence++;
    slot->retries = 0u;
    slot->sent = 0u;
    slot->last_tx_ms = 0u;
    reliable_count++;
    g_protocol_runtime_debug.reliable_queue_count = reliable_count;
    return 1u;
}

static void ProtocolRuntimeServiceReliable(uint32_t now_ms)
{
    Protocol_Reliable_Slot_s *slot;

    if (reliable_count == 0u) {
        return;
    }
    slot = &reliable_fifo[reliable_head];
    if (slot->sent == 0u) {
        if (ProtocolRuntimeSendFrame(slot->id, slot->payload,
                slot->payload_len, slot->sequence) != 0u) {
            slot->sent = 1u;
            slot->last_tx_ms = now_ms;
            g_protocol_runtime_debug.reliable_tx_count++;
        }
        return;
    }
    if ((uint32_t)(now_ms - slot->last_tx_ms) <
        PROTOCOL_RETRY_INTERVAL_MS) {
        return;
    }
    if (slot->retries >= PROTOCOL_MAX_RETRIES) {
        reliable_head = (uint8_t)((reliable_head + 1u) %
                                  PROTOCOL_RUNTIME_FIFO_DEPTH);
        reliable_count--;
        g_protocol_runtime_debug.reliable_queue_count = reliable_count;
        g_protocol_runtime_debug.reliable_drop_count++;
        return;
    }
    if (ProtocolRuntimeSendFrame(slot->id, slot->payload,
            slot->payload_len, slot->sequence) != 0u) {
        slot->retries++;
        slot->last_tx_ms = now_ms;
        g_protocol_runtime_debug.reliable_tx_count++;
        g_protocol_runtime_debug.reliable_retry_count++;
    }
}

void ProtocolRuntimeTask(uint32_t now_ms)
{
    g_protocol_runtime_debug.now_ms = now_ms;
    if (g_protocol_runtime_debug.connection_ready == 0u) {
        g_protocol_runtime_debug.heartbeat_age_ms = 0u;
        return;
    }
    g_protocol_runtime_debug.heartbeat_age_ms =
        (uint32_t)(now_ms -
                   g_protocol_runtime_debug.last_heartbeat_rx_ms);
    if (g_protocol_runtime_debug.heartbeat_age_ms >=
        PROTOCOL_HEARTBEAT_TIMEOUT_MS) {
        g_protocol_runtime_debug.heartbeat_timeout_count++;
        ProtocolRuntimeExpireSession();
        return;
    }
    ProtocolRuntimeServiceReliable(now_ms);
}
