#include "protocol.h"

#include <string.h>

#if defined(__CC_ARM)
#define PROTOCOL_WEAK __weak
#else
#define PROTOCOL_WEAK __attribute__((weak))
#endif

typedef enum {
    WAIT_H1 = 0,
    WAIT_H2,
    WAIT_ID,
    WAIT_LEN,
    WAIT_DATA,
    WAIT_CRC
} ParseState;

typedef struct {
    uint8_t id;
    uint8_t payload[PROTOCOL_MAX_PAYLOAD_LEN - 1u];
    uint8_t payload_len;
    uint8_t seq;
    uint8_t retries;
    uint8_t sent;
    uint32_t last_tx_ms;
} ReliableSlot;

static ReliableSlot reliable_fifo[PROTOCOL_RELIABLE_FIFO_DEPTH];
static uint8_t reliable_head;
static uint8_t reliable_count;
static uint8_t reliable_next_seq;
static ParseState rx_state;
static uint8_t rx_id;
static uint8_t rx_len;
static uint8_t rx_pos;
static uint8_t rx_crc;
static uint8_t rx_data[PROTOCOL_MAX_PAYLOAD_LEN];
static uint8_t connection_ready;
static uint8_t link_online;
static uint32_t protocol_now_ms;
static uint32_t last_heartbeat_rx_ms;
static Protocol_Debug_s protocol_debug;

static uint8_t crc_update(uint8_t crc, uint8_t byte)
{
    uint8_t i;

    crc ^= byte;
    for (i = 0u; i < 8u; ++i) {
        crc = (uint8_t)((crc & 0x80u) != 0u ?
            (uint8_t)((uint8_t)(crc << 1) ^ 0x31u) :
            (uint8_t)(crc << 1));
    }
    return crc;
}

uint8_t protocol_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0u;

    if (data == NULL && len != 0u) {
        return 0u;
    }
    while (len-- != 0u) {
        crc = crc_update(crc, *data++);
    }
    return crc;
}

static uint8_t send_frame(uint8_t id, const void *payload, uint8_t size,
                          int seq)
{
    uint8_t frame[4u + PROTOCOL_MAX_PAYLOAD_LEN + 1u];
    uint8_t i = 0u;
    uint8_t payload_len = (uint8_t)(size + (seq >= 0 ? 1u : 0u));

    if (payload_len > PROTOCOL_MAX_PAYLOAD_LEN ||
        (size != 0u && payload == NULL)) {
        protocol_debug.tx_fail_count++;
        return 0u;
    }

    frame[i++] = FRAME_HEADER1;
    frame[i++] = FRAME_HEADER2;
    frame[i++] = id;
    frame[i++] = payload_len;
    if (size != 0u) {
        memcpy(frame + i, payload, size);
        i = (uint8_t)(i + size);
    }
    if (seq >= 0) {
        frame[i++] = (uint8_t)seq;
    }
    frame[i] = protocol_crc8(frame + 2u, (size_t)i - 2u);
    ++i;

    if (serial_write(frame, i) == 0u) {
        protocol_debug.tx_fail_count++;
        return 0u;
    }

    protocol_debug.last_tx_id = id;
    protocol_debug.last_tx_len = i;
    if (id == PACKET_ID_HEARTBEAT) {
        protocol_debug.heartbeat_tx_count++;
    }
    return 1u;
}

static void protocol_clear_reliable_queue(void)
{
    reliable_head = 0u;
    reliable_count = 0u;
    protocol_debug.reliable_queue_count = 0u;
}

static void protocol_clear_parser(void)
{
    rx_state = WAIT_H1;
    rx_id = 0u;
    rx_len = 0u;
    rx_pos = 0u;
    rx_crc = 0u;
}

static void protocol_expire_session(void)
{
    connection_ready = 0u;
    link_online = 0u;
    protocol_clear_reliable_queue();
    protocol_debug.connection_ready = 0u;
    protocol_debug.link_online = 0u;
}

int protocol_send_ack(uint8_t acked_id, uint8_t ack_seq)
{
    Packet_Ack packet;

    packet.acked_id = acked_id;
    packet.ack_seq = ack_seq;
    protocol_debug.last_ack_id = acked_id;
    protocol_debug.last_ack_seq = ack_seq;
    if (send_frame(PACKET_ID_ACK, &packet, sizeof(packet), -1) == 0u) {
        return 0;
    }
    protocol_debug.ack_tx_count++;
    return 1;
}

int protocol_send_heartbeat(const Packet_Heartbeat *packet)
{
    if (packet == NULL) {
        return 0;
    }
    return send_frame(PACKET_ID_HEARTBEAT, packet, sizeof(*packet), -1) != 0u;
}

int protocol_send_handshake(const Packet_Handshake *packet)
{
    if (packet == NULL) {
        return 0;
    }
    return send_frame(PACKET_ID_HANDSHAKE, packet, sizeof(*packet), -1) != 0u;
}

int protocol_send_fruit_detection(const Packet_FruitDetection *packet)
{
    if (packet == NULL) {
        return 0;
    }
    return send_frame(PACKET_ID_FRUITDETECTION, packet,
                      sizeof(*packet), -1) != 0u;
}

int protocol_send_reliable(uint8_t id, const void *payload, uint8_t size)
{
    ReliableSlot *slot;
    uint8_t tail;

    if (connection_ready == 0u || payload == NULL || size == 0u ||
        size > PROTOCOL_MAX_PAYLOAD_LEN - 1u ||
        id == PACKET_ID_FRUITDETECTION || id >= PACKET_ID_ACK ||
        reliable_count >= PROTOCOL_RELIABLE_FIFO_DEPTH) {
        return 0;
    }

    tail = (uint8_t)((reliable_head + reliable_count) %
                     PROTOCOL_RELIABLE_FIFO_DEPTH);
    slot = &reliable_fifo[tail];
    slot->id = id;
    memcpy(slot->payload, payload, size);
    slot->payload_len = size;
    slot->seq = reliable_next_seq++;
    slot->retries = 0u;
    slot->sent = 0u;
    slot->last_tx_ms = 0u;
    reliable_count++;
    protocol_debug.reliable_queue_count = reliable_count;
    return 1;
}

uint8_t protocol_link_is_online(void)
{
    return link_online;
}

uint8_t protocol_connection_ready(void)
{
    return connection_ready;
}

uint32_t protocol_get_time_ms(void)
{
    return protocol_now_ms;
}

const Protocol_Debug_s *protocol_get_debug(void)
{
    return &protocol_debug;
}

PROTOCOL_WEAK void on_receive_FruitDetection(
    const Packet_FruitDetection *packet)
{
    (void)packet;
}

static void protocol_accept_handshake(const Packet_Handshake *packet)
{
    protocol_clear_reliable_queue();
    connection_ready = 1u;
    link_online = 1u;
    last_heartbeat_rx_ms = protocol_now_ms;
    protocol_debug.last_heartbeat_rx_ms = last_heartbeat_rx_ms;
    protocol_debug.heartbeat_age_ms = 0u;
    protocol_debug.handshake_ok_count++;
    protocol_debug.session_count++;
    protocol_debug.connection_ready = 1u;
    protocol_debug.link_online = 1u;
    (void)protocol_send_handshake(packet);
}

static void protocol_handle_ack(const Packet_Ack *packet)
{
    ReliableSlot *slot;

    protocol_debug.ack_rx_count++;
    protocol_debug.last_ack_id = packet->acked_id;
    protocol_debug.last_ack_seq = packet->ack_seq;
    if (reliable_count == 0u) {
        return;
    }
    slot = &reliable_fifo[reliable_head];
    if (packet->acked_id == slot->id && packet->ack_seq == slot->seq) {
        reliable_head = (uint8_t)((reliable_head + 1u) %
                                  PROTOCOL_RELIABLE_FIFO_DEPTH);
        reliable_count--;
        protocol_debug.reliable_queue_count = reliable_count;
    }
}

static void dispatch(uint8_t id, uint8_t len)
{
    protocol_debug.rx_frame_count++;
    protocol_debug.last_rx_id = id;
    protocol_debug.last_rx_len = len;

    if (id == PACKET_ID_HANDSHAKE) {
        Packet_Handshake packet;

        if (len != sizeof(packet)) {
            protocol_debug.length_fail_count++;
            return;
        }
        memcpy(&packet, rx_data, sizeof(packet));
        if (packet.protocol_hash != PROTOCOL_HASH) {
            protocol_debug.handshake_mismatch_count++;
            protocol_expire_session();
            return;
        }
        protocol_accept_handshake(&packet);
        return;
    }

    if (connection_ready == 0u) {
        protocol_debug.prehandshake_drop_count++;
        return;
    }

    if (id == PACKET_ID_HEARTBEAT) {
        Packet_Heartbeat packet;

        if (len != sizeof(packet)) {
            protocol_debug.length_fail_count++;
            return;
        }
        memcpy(&packet, rx_data, sizeof(packet));
        protocol_debug.heartbeat_rx_count++;
        protocol_debug.last_heartbeat_count = packet.count;
        last_heartbeat_rx_ms = protocol_now_ms;
        protocol_debug.last_heartbeat_rx_ms = last_heartbeat_rx_ms;
        protocol_debug.heartbeat_age_ms = 0u;
        link_online = 1u;
        protocol_debug.link_online = 1u;
        (void)protocol_send_heartbeat(&packet);
        return;
    }

    if (id == PACKET_ID_ACK) {
        Packet_Ack packet;

        if (len != sizeof(packet)) {
            protocol_debug.length_fail_count++;
            return;
        }
        memcpy(&packet, rx_data, sizeof(packet));
        protocol_handle_ack(&packet);
        return;
    }

    if (id == PACKET_ID_FRUITDETECTION) {
        Packet_FruitDetection packet;

        if (len != sizeof(packet)) {
            protocol_debug.length_fail_count++;
            return;
        }
        memcpy(&packet, rx_data, sizeof(packet));
        on_receive_FruitDetection(&packet);
        return;
    }

    protocol_debug.unknown_id_count++;
}

void protocol_fsm_feed(uint8_t byte)
{
    protocol_debug.parse_state = (uint8_t)rx_state;
    protocol_debug.current_rx_id = rx_id;
    protocol_debug.current_rx_len = rx_len;
    protocol_debug.current_rx_pos = rx_pos;

    switch (rx_state) {
        case WAIT_H1:
            if (byte == FRAME_HEADER1) {
                rx_state = WAIT_H2;
            }
            break;

        case WAIT_H2:
            rx_state = byte == FRAME_HEADER2 ? WAIT_ID :
                (byte == FRAME_HEADER1 ? WAIT_H2 : WAIT_H1);
            break;

        case WAIT_ID:
            rx_id = byte;
            rx_crc = crc_update(0u, byte);
            rx_state = WAIT_LEN;
            break;

        case WAIT_LEN:
            rx_len = byte;
            rx_crc = crc_update(rx_crc, byte);
            rx_pos = 0u;
            if (rx_len > sizeof(rx_data)) {
                protocol_debug.length_fail_count++;
                rx_state = WAIT_H1;
            } else {
                rx_state = rx_len != 0u ? WAIT_DATA : WAIT_CRC;
            }
            break;

        case WAIT_DATA:
            rx_data[rx_pos++] = byte;
            rx_crc = crc_update(rx_crc, byte);
            if (rx_pos == rx_len) {
                rx_state = WAIT_CRC;
            }
            break;

        case WAIT_CRC:
            if (byte == rx_crc) {
                dispatch(rx_id, rx_len);
            } else {
                protocol_debug.crc_fail_count++;
            }
            rx_state = byte == FRAME_HEADER1 ? WAIT_H2 : WAIT_H1;
            break;

        default:
            rx_state = WAIT_H1;
            break;
    }

    protocol_debug.parse_state = (uint8_t)rx_state;
    protocol_debug.current_rx_id = rx_id;
    protocol_debug.current_rx_len = rx_len;
    protocol_debug.current_rx_pos = rx_pos;
}

static void protocol_service_reliable(uint32_t now_ms)
{
    ReliableSlot *slot;

    if (reliable_count == 0u) {
        return;
    }
    slot = &reliable_fifo[reliable_head];
    if (slot->sent == 0u) {
        if (send_frame(slot->id, slot->payload, slot->payload_len,
                       slot->seq) != 0u) {
            slot->sent = 1u;
            slot->last_tx_ms = now_ms;
            protocol_debug.reliable_tx_count++;
        }
        return;
    }
    if ((uint32_t)(now_ms - slot->last_tx_ms) <
        PROTOCOL_RETRY_INTERVAL_MS) {
        return;
    }
    if (slot->retries >= PROTOCOL_MAX_RETRIES) {
        reliable_head = (uint8_t)((reliable_head + 1u) %
                                  PROTOCOL_RELIABLE_FIFO_DEPTH);
        reliable_count--;
        protocol_debug.reliable_queue_count = reliable_count;
        protocol_debug.reliable_drop_count++;
        return;
    }
    if (send_frame(slot->id, slot->payload, slot->payload_len,
                   slot->seq) != 0u) {
        slot->retries++;
        slot->last_tx_ms = now_ms;
        protocol_debug.reliable_retry_count++;
        protocol_debug.reliable_tx_count++;
    }
}

void protocol_tick(uint32_t now_ms)
{
    protocol_now_ms = now_ms;
    protocol_debug.connection_ready = connection_ready;
    protocol_debug.link_online = link_online;
    protocol_debug.reliable_queue_count = reliable_count;

    if (connection_ready == 0u) {
        protocol_debug.heartbeat_age_ms = 0u;
        return;
    }

#if PROTOCOL_ENABLE_HEARTBEAT && PROTOCOL_STRICT_HEARTBEAT
    protocol_debug.heartbeat_age_ms =
        (uint32_t)(now_ms - last_heartbeat_rx_ms);
    if (protocol_debug.heartbeat_age_ms >=
        PROTOCOL_HEARTBEAT_TIMEOUT_MS) {
        protocol_debug.heartbeat_timeout_count++;
        protocol_expire_session();
        return;
    }
#endif

    protocol_service_reliable(now_ms);
}

void protocol_reset_connection(void)
{
    protocol_clear_parser();
    protocol_expire_session();
    protocol_debug.connection_reset_count++;
}

void protocol_init(void)
{
    memset(&protocol_debug, 0, sizeof(protocol_debug));
    memset(reliable_fifo, 0, sizeof(reliable_fifo));
    protocol_now_ms = 0u;
    last_heartbeat_rx_ms = 0u;
    reliable_next_seq = 0u;
    protocol_clear_parser();
    protocol_expire_session();
}
