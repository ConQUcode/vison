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
    Packet_CallbackStatus packet;
    uint8_t seq;
    uint8_t retries;
    uint8_t sent;
    uint32_t last_tx_ms;
} CallbackSlot;

typedef struct {
    Packet_MotionStatus packet;
    uint8_t seq;
    uint8_t retries;
    uint8_t sent;
    uint32_t last_tx_ms;
} MotionStatusSlot;

static CallbackSlot callback_fifo[PROTOCOL_CALLBACK_FIFO_DEPTH];
static MotionStatusSlot motion_status_fifo[PROTOCOL_MOTION_FIFO_DEPTH];
static uint8_t callback_head;
static uint8_t callback_count;
static uint8_t callback_next_seq;
static uint8_t motion_status_head;
static uint8_t motion_status_count;
static uint8_t motion_status_next_seq;
static ParseState rx_state;
static uint8_t rx_id;
static uint8_t rx_len;
static uint8_t rx_pos;
static uint8_t rx_crc;
static uint8_t rx_data[PROTOCOL_MAX_PAYLOAD_LEN];
static uint8_t connection_ready;
static uint8_t task_seen;
static uint8_t task_seq;
static uint8_t target_control_seen;
static uint8_t target_control_seq;
static uint8_t next_queue_selector;
static uint32_t protocol_now_ms;
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

    if (payload_len > PROTOCOL_MAX_PAYLOAD_LEN) {
        protocol_debug.tx_fail_count++;
        return 0u;
    }

    frame[i++] = FRAME_HEADER1;
    frame[i++] = FRAME_HEADER2;
    frame[i++] = id;
    frame[i++] = payload_len;
    if (size != 0u && payload != NULL) {
        memcpy(frame + i, payload, size);
        i = (uint8_t)(i + size);
    }
    if (seq >= 0) {
        frame[i++] = (uint8_t)seq;
    }
    frame[i] = protocol_crc8(frame + 2u, (size_t)i - 2u);
    ++i;

    if (!serial_write(frame, i)) {
        protocol_debug.tx_fail_count++;
        return 0u;
    }

    protocol_debug.last_tx_id = id;
    protocol_debug.last_tx_len = i;
    if (id == PACKET_ID_HEARTBEAT) {
        protocol_debug.heartbeat_tx_count++;
    } else if (id == PACKET_ID_CALLBACKSTATUS) {
        protocol_debug.callback_status_tx_count++;
    } else if (id == PACKET_ID_MOTIONSTATUS) {
        protocol_debug.motion_status_tx_count++;
    }
    return 1u;
}

static uint8_t send_ack(uint8_t id, uint8_t seq)
{
    Packet_Ack ack;
    uint8_t ok;

    ack.acked_id = id;
    ack.ack_seq = seq;
    protocol_debug.last_ack_id = id;
    protocol_debug.last_ack_seq = seq;
    ok = send_frame(PACKET_ID_ACK, &ack, sizeof(ack), -1);
    if (ok != 0u) {
        protocol_debug.ack_tx_count++;
    }
    return ok;
}

static uint8_t send_callback_head(uint32_t now_ms)
{
    CallbackSlot *slot = &callback_fifo[callback_head];

    if (!send_frame(PACKET_ID_CALLBACKSTATUS, &slot->packet,
                    sizeof(slot->packet), slot->seq)) {
        return 0u;
    }
    slot->sent = 1u;
    slot->last_tx_ms = now_ms;
    return 1u;
}

static uint8_t send_motion_status_head(uint32_t now_ms)
{
    MotionStatusSlot *slot = &motion_status_fifo[motion_status_head];

    if (!send_frame(PACKET_ID_MOTIONSTATUS, &slot->packet,
                    sizeof(slot->packet), slot->seq)) {
        return 0u;
    }
    slot->sent = 1u;
    slot->last_tx_ms = now_ms;
    return 1u;
}

int protocol_send_callback_status(const Packet_CallbackStatus *pkt)
{
    uint8_t tail;

    if (pkt == NULL || callback_count >= PROTOCOL_CALLBACK_FIFO_DEPTH) {
        return 0;
    }
    tail = (uint8_t)((callback_head + callback_count) %
                     PROTOCOL_CALLBACK_FIFO_DEPTH);
    callback_fifo[tail].packet = *pkt;
    callback_fifo[tail].seq = callback_next_seq++;
    callback_fifo[tail].retries = 0u;
    callback_fifo[tail].sent = 0u;
    callback_fifo[tail].last_tx_ms = 0u;
    ++callback_count;
    protocol_debug.callback_queue_count = callback_count;
    return 1;
}

int protocol_send_motion_status(const Packet_MotionStatus *pkt)
{
    uint8_t tail;

    if (pkt == NULL || motion_status_count >= PROTOCOL_MOTION_FIFO_DEPTH) {
        return 0;
    }
    tail = (uint8_t)((motion_status_head + motion_status_count) %
                     PROTOCOL_MOTION_FIFO_DEPTH);
    motion_status_fifo[tail].packet = *pkt;
    motion_status_fifo[tail].seq = motion_status_next_seq++;
    motion_status_fifo[tail].retries = 0u;
    motion_status_fifo[tail].sent = 0u;
    motion_status_fifo[tail].last_tx_ms = 0u;
    ++motion_status_count;
    protocol_debug.motion_queue_count = motion_status_count;
    return 1;
}

size_t protocol_callback_queue_size(void)
{
    return callback_count;
}

uint8_t protocol_link_is_online(void)
{
    return connection_ready;
}

uint8_t protocol_connection_ready(void)
{
    return connection_ready;
}

const Protocol_Debug_s *protocol_get_debug(void)
{
    return &protocol_debug;
}

PROTOCOL_WEAK void on_receive_TaskStatus(const Packet_TaskStatus *pkt)
{
    (void)pkt;
}

PROTOCOL_WEAK void on_receive_TargetControl(const Packet_TargetControl *pkt)
{
    (void)pkt;
}

static void dispatch(uint8_t id, uint8_t len)
{
    protocol_debug.rx_frame_count++;
    protocol_debug.last_rx_id = id;
    protocol_debug.last_rx_len = len;

    if (id == PACKET_ID_HANDSHAKE && len == sizeof(Packet_Handshake)) {
        Packet_Handshake packet;

        memcpy(&packet, rx_data, sizeof(packet));
        if (packet.protocol_hash == PROTOCOL_HASH) {
            connection_ready = 1u;
            task_seen = 0u;
            target_control_seen = 0u;
            (void)send_frame(id, &packet, sizeof(packet), -1);
        }
        protocol_debug.connection_ready = connection_ready;
        protocol_debug.link_online = connection_ready;
        return;
    }

    if (connection_ready == 0u) {
        return;
    }

    if (id == PACKET_ID_HEARTBEAT && len == sizeof(Packet_Heartbeat)) {
        protocol_debug.heartbeat_rx_count++;
        protocol_debug.link_online = 1u;
        (void)send_frame(PACKET_ID_HEARTBEAT, rx_data, len, -1);
        return;
    }

    if (id == PACKET_ID_ACK && len == sizeof(Packet_Ack)) {
        Packet_Ack ack;

        memcpy(&ack, rx_data, sizeof(ack));
        protocol_debug.ack_rx_count++;
        if (callback_count != 0u &&
            ack.acked_id == PACKET_ID_CALLBACKSTATUS &&
            ack.ack_seq == callback_fifo[callback_head].seq) {
            callback_head = (uint8_t)((callback_head + 1u) %
                                      PROTOCOL_CALLBACK_FIFO_DEPTH);
            --callback_count;
            protocol_debug.callback_queue_count = callback_count;
        }
        if (motion_status_count != 0u &&
            ack.acked_id == PACKET_ID_MOTIONSTATUS &&
            ack.ack_seq == motion_status_fifo[motion_status_head].seq) {
            motion_status_head = (uint8_t)((motion_status_head + 1u) %
                                           PROTOCOL_MOTION_FIFO_DEPTH);
            --motion_status_count;
            protocol_debug.motion_queue_count = motion_status_count;
        }
        return;
    }

    if (id == PACKET_ID_TASKSTATUS &&
        len == sizeof(Packet_TaskStatus) + 1u) {
        uint8_t seq = rx_data[sizeof(Packet_TaskStatus)];
        Packet_TaskStatus packet;

        if (send_ack(id, seq) == 0u) {
            return;
        }
        if (task_seen != 0u && seq == task_seq) {
            protocol_debug.duplicate_count++;
            return;
        }
        task_seen = 1u;
        task_seq = seq;
        memcpy(&packet, rx_data, sizeof(packet));
        protocol_debug.last_task_id = packet.task_id;
        protocol_debug.last_task_status = packet.task_status;
        on_receive_TaskStatus(&packet);
        return;
    }

    if (id == PACKET_ID_TARGETCONTROL &&
        len == sizeof(Packet_TargetControl) + 1u) {
        uint8_t seq = rx_data[sizeof(Packet_TargetControl)];
        Packet_TargetControl packet;

        if (send_ack(id, seq) == 0u) {
            return;
        }
        if (target_control_seen != 0u && seq == target_control_seq) {
            protocol_debug.duplicate_count++;
            return;
        }
        target_control_seen = 1u;
        target_control_seq = seq;
        memcpy(&packet, rx_data, sizeof(packet));
        protocol_debug.last_target_valid = 1u;
        protocol_debug.last_target_x_mm = packet.x_mm;
        protocol_debug.last_target_y_mm = packet.y_mm;
        protocol_debug.last_target_yaw_deg = packet.yaw_deg;
        on_receive_TargetControl(&packet);
        return;
    }
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

static uint8_t protocol_service_callback(uint32_t now_ms)
{
    CallbackSlot *slot;

    if (callback_count == 0u) {
        return 0u;
    }
    slot = &callback_fifo[callback_head];
    if (slot->sent == 0u) {
        return send_callback_head(now_ms);
    }
    if ((uint32_t)(now_ms - slot->last_tx_ms) >=
        PROTOCOL_RETRY_INTERVAL_MS) {
        if (slot->retries >= PROTOCOL_MAX_RETRIES) {
            callback_head = (uint8_t)((callback_head + 1u) %
                                      PROTOCOL_CALLBACK_FIFO_DEPTH);
            --callback_count;
            protocol_debug.callback_queue_count = callback_count;
            protocol_debug.retry_drop_count++;
            return 1u;
        }
        ++slot->retries;
        return send_callback_head(now_ms);
    }
    return 0u;
}

static uint8_t protocol_service_motion(uint32_t now_ms)
{
    MotionStatusSlot *slot;

    if (motion_status_count == 0u) {
        return 0u;
    }
    slot = &motion_status_fifo[motion_status_head];
    if (slot->sent == 0u) {
        return send_motion_status_head(now_ms);
    }
    if ((uint32_t)(now_ms - slot->last_tx_ms) >=
        PROTOCOL_RETRY_INTERVAL_MS) {
        if (slot->retries >= PROTOCOL_MAX_RETRIES) {
            motion_status_head = (uint8_t)((motion_status_head + 1u) %
                                           PROTOCOL_MOTION_FIFO_DEPTH);
            --motion_status_count;
            protocol_debug.motion_queue_count = motion_status_count;
            protocol_debug.retry_drop_count++;
            return 1u;
        }
        ++slot->retries;
        return send_motion_status_head(now_ms);
    }
    return 0u;
}

void protocol_tick(uint32_t now_ms)
{
    protocol_now_ms = now_ms;
    protocol_debug.connection_ready = connection_ready;
    protocol_debug.link_online = connection_ready;
    protocol_debug.callback_queue_count = callback_count;
    protocol_debug.motion_queue_count = motion_status_count;

    if (connection_ready == 0u) {
        return;
    }

    if (next_queue_selector == 0u) {
        if (protocol_service_callback(now_ms) == 0u) {
            (void)protocol_service_motion(now_ms);
        }
        next_queue_selector = 1u;
    } else {
        if (protocol_service_motion(now_ms) == 0u) {
            (void)protocol_service_callback(now_ms);
        }
        next_queue_selector = 0u;
    }
}

void protocol_reset_connection(void)
{
    rx_state = WAIT_H1;
    rx_id = 0u;
    rx_len = 0u;
    rx_pos = 0u;
    rx_crc = 0u;
    connection_ready = (uint8_t)(PROTOCOL_REQUIRE_HANDSHAKE == 0u);
    task_seen = 0u;
    target_control_seen = 0u;
    callback_head = 0u;
    callback_count = 0u;
    motion_status_head = 0u;
    motion_status_count = 0u;
    next_queue_selector = 0u;
    protocol_now_ms = 0u;
    memset(&protocol_debug, 0, sizeof(protocol_debug));
    protocol_debug.connection_ready = connection_ready;
    protocol_debug.link_online = connection_ready;
}

void protocol_init(void)
{
    callback_next_seq = 0u;
    motion_status_next_seq = 0u;
    protocol_reset_connection();
}
