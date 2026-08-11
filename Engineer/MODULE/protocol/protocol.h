#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

/* 新上位机线协议。USB CDC不使用生成文档中的物理串口波特率。 */
#define PROTOCOL_HASH UINT32_C(0x923FFDD9)
#define FRAME_HEADER1 90u
#define FRAME_HEADER2 165u
#define PROTOCOL_ENABLE_HEARTBEAT 1u
#define PROTOCOL_STRICT_HEARTBEAT 1u
#define PROTOCOL_REQUIRE_HANDSHAKE 1u
#define PROTOCOL_HEARTBEAT_TIMEOUT_MS 3000u
#define PROTOCOL_RETRY_INTERVAL_MS 100u
#define PROTOCOL_MAX_RETRIES 3u
#define PROTOCOL_RELIABLE_FIFO_DEPTH 4u
#define PROTOCOL_MAX_PAYLOAD_LEN 32u

typedef enum {
    PACKET_ID_FRUITDETECTION = 0x10,
    PACKET_ID_ACK = 0xFD,
    PACKET_ID_HEARTBEAT = 0xFE,
    PACKET_ID_HANDSHAKE = 0xFF,
} PacketID;

#pragma pack(push, 1)
#define PROTOCOL_STATIC_ASSERT(name, cond) \
    typedef char protocol_static_assert_##name[(cond) ? 1 : -1]

typedef struct {
    uint8_t acked_id;
    uint8_t ack_seq;
} Packet_Ack;

typedef struct {
    uint32_t count;
} Packet_Heartbeat;

typedef struct {
    uint32_t protocol_hash;
} Packet_Handshake;

/* D435i当前最高置信度识别结果；本阶段只观察，不触发机械臂动作。 */
typedef struct {
    uint8_t fruit_id; /* 0无目标，1..6对应协议文档中的六类水果。 */
    uint8_t status;   /* 0未成熟，1成熟。 */
} Packet_FruitDetection;
#pragma pack(pop)

typedef struct {
    uint8_t connection_ready;       /* 已完成匹配哈希的握手。 */
    uint8_t link_online;            /* 严格心跳仍在3000 ms有效期内。 */
    uint8_t parse_state;
    uint8_t current_rx_id;
    uint8_t current_rx_len;
    uint8_t current_rx_pos;
    uint8_t last_rx_id;
    uint8_t last_rx_len;
    uint8_t last_tx_id;
    uint8_t last_tx_len;
    uint8_t last_ack_id;
    uint8_t last_ack_seq;
    uint8_t reliable_queue_count;
    uint32_t session_count;         /* 每次匹配握手递增，供应用层失效旧数据。 */
    uint32_t last_heartbeat_count;
    uint32_t last_heartbeat_rx_ms;
    uint32_t heartbeat_age_ms;
    uint32_t rx_frame_count;
    uint32_t crc_fail_count;
    uint32_t length_fail_count;
    uint32_t unknown_id_count;
    uint32_t prehandshake_drop_count;
    uint32_t handshake_ok_count;
    uint32_t handshake_mismatch_count;
    uint32_t heartbeat_tx_count;
    uint32_t heartbeat_rx_count;
    uint32_t heartbeat_timeout_count;
    uint32_t ack_tx_count;
    uint32_t ack_rx_count;
    uint32_t reliable_tx_count;
    uint32_t reliable_retry_count;
    uint32_t reliable_drop_count;
    uint32_t tx_fail_count;
    uint32_t connection_reset_count;
} Protocol_Debug_s;

PROTOCOL_STATIC_ASSERT(ack_size, sizeof(Packet_Ack) == 2);
PROTOCOL_STATIC_ASSERT(heartbeat_size, sizeof(Packet_Heartbeat) == 4);
PROTOCOL_STATIC_ASSERT(handshake_size, sizeof(Packet_Handshake) == 4);
PROTOCOL_STATIC_ASSERT(fruit_detection_size,
                       sizeof(Packet_FruitDetection) == 2);

uint8_t protocol_crc8(const uint8_t *data, size_t len);
void protocol_init(void);
void protocol_reset_connection(void);
void protocol_fsm_feed(uint8_t byte);
void protocol_tick(uint32_t now_ms);
uint8_t protocol_link_is_online(void);
uint8_t protocol_connection_ready(void);
uint32_t protocol_get_time_ms(void);
const Protocol_Debug_s *protocol_get_debug(void);

/* 通用可靠发送入口保留给后续业务包；当前FruitDetection不使用它。 */
int protocol_send_reliable(uint8_t id, const void *payload, uint8_t size);
int protocol_send_ack(uint8_t acked_id, uint8_t ack_seq);
int protocol_send_heartbeat(const Packet_Heartbeat *packet);
int protocol_send_handshake(const Packet_Handshake *packet);
int protocol_send_fruit_detection(const Packet_FruitDetection *packet);

uint8_t serial_write(const uint8_t *data, uint16_t len);

/* 应用层覆盖该弱回调；协议层只完成帧校验和握手门控。 */
void on_receive_FruitDetection(const Packet_FruitDetection *packet);

#endif
