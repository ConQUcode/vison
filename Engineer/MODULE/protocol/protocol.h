#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define PROTOCOL_HASH UINT32_C(0x8845D84A)
#define FRAME_HEADER1 90u
#define FRAME_HEADER2 165u
#define PROTOCOL_ENABLE_HEARTBEAT 1u
#define PROTOCOL_STRICT_HEARTBEAT 0u
#define PROTOCOL_REQUIRE_HANDSHAKE 0u
#define PROTOCOL_RETRY_INTERVAL_MS 100u
#define PROTOCOL_MAX_RETRIES 3u
#define PROTOCOL_CALLBACK_FIFO_DEPTH 4u
#define PROTOCOL_MOTION_FIFO_DEPTH 4u
#define PROTOCOL_MAX_PAYLOAD_LEN 15u

typedef enum {
    PACKET_ID_TASKSTATUS = 0x01,
    PACKET_ID_CALLBACKSTATUS = 0x02,
    PACKET_ID_TARGETCONTROL = 0x04,
    PACKET_ID_MOTIONSTATUS = 0x05,
    PACKET_ID_ACK = 0xFD,
    PACKET_ID_HEARTBEAT = 0xFE,
    PACKET_ID_HANDSHAKE = 0xFF,
} PacketID;

typedef enum {
    STATUS_END = 0,
    STATUS_START = 1,
    STATUS_FAULT_RETRY = 2,
    STATUS_STOP = 3,
} Status;

typedef enum {
    MOTIONSTATE_ACCEPTED = 1,
    MOTIONSTATE_RUNNING = 2,
    MOTIONSTATE_COMPLETED = 3,
    MOTIONSTATE_FAILED = 4,
} MotionState;

typedef enum {
    MOTIONFAULT_NONE = 0,
    MOTIONFAULT_IK_UNREACHABLE = 1,
    MOTIONFAULT_JOINT_LIMIT = 2,
    MOTIONFAULT_TIMEOUT = 3,
    MOTIONFAULT_COLLISION = 4,
    MOTIONFAULT_GRIP_LOST = 5,
    MOTIONFAULT_LINK_LOSS = 6,
    MOTIONFAULT_ESTOP = 7,
} MotionFault;

#pragma pack(push, 1)
#define PROTOCOL_STATIC_ASSERT(name, cond) \
    typedef char protocol_static_assert_##name[(cond) ? 1 : -1]

typedef struct {
    uint8_t task_id;
    uint8_t task_status;
} Packet_TaskStatus;

typedef struct {
    uint8_t callback_id;
    uint8_t callback_status;
} Packet_CallbackStatus;

typedef struct {
    float x_mm;
    float y_mm;
    float yaw_deg;
} Packet_TargetControl;

typedef struct {
    uint8_t state;
    uint8_t fault;
    float x_mm;
    float y_mm;
    float yaw_deg;
} Packet_MotionStatus;

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
#pragma pack(pop)

typedef struct {
    uint8_t connection_ready;
    uint8_t link_online;
    uint8_t parse_state;
    uint8_t current_rx_id;
    uint8_t current_rx_len;
    uint8_t current_rx_pos;
    uint8_t last_rx_id;
    uint8_t last_rx_len;
    uint8_t last_tx_id;
    uint8_t last_tx_len;
    uint8_t last_ack_seq;
    uint8_t last_ack_id;
    uint8_t last_task_id;
    uint8_t last_task_status;
    uint8_t last_target_valid;
    float last_target_x_mm;
    float last_target_y_mm;
    float last_target_yaw_deg;
    uint32_t rx_frame_count;
    uint32_t crc_fail_count;
    uint32_t length_fail_count;
    uint32_t duplicate_count;
    uint32_t tx_fail_count;
    uint32_t retry_drop_count;
    uint32_t heartbeat_tx_count;
    uint32_t heartbeat_rx_count;
    uint32_t ack_tx_count;
    uint32_t ack_rx_count;
    uint32_t callback_queue_count;
    uint32_t motion_queue_count;
    uint32_t callback_status_tx_count;
    uint32_t motion_status_tx_count;
} Protocol_Debug_s;

PROTOCOL_STATIC_ASSERT(task_status_size, sizeof(Packet_TaskStatus) == 2);
PROTOCOL_STATIC_ASSERT(callback_status_size,
                       sizeof(Packet_CallbackStatus) == 2);
PROTOCOL_STATIC_ASSERT(target_control_size,
                       sizeof(Packet_TargetControl) == 12);
PROTOCOL_STATIC_ASSERT(motion_status_size,
                       sizeof(Packet_MotionStatus) == 14);
PROTOCOL_STATIC_ASSERT(ack_size, sizeof(Packet_Ack) == 2);
PROTOCOL_STATIC_ASSERT(heartbeat_size, sizeof(Packet_Heartbeat) == 4);
PROTOCOL_STATIC_ASSERT(handshake_size, sizeof(Packet_Handshake) == 4);
PROTOCOL_STATIC_ASSERT(float32_size, sizeof(float) == 4);

uint8_t protocol_crc8(const uint8_t *data, size_t len);
void protocol_init(void);
void protocol_reset_connection(void);
void protocol_fsm_feed(uint8_t byte);
void protocol_tick(uint32_t now_ms);
uint8_t protocol_link_is_online(void);
uint8_t protocol_connection_ready(void);
const Protocol_Debug_s *protocol_get_debug(void);
int protocol_send_callback_status(const Packet_CallbackStatus *pkt);
int protocol_send_motion_status(const Packet_MotionStatus *pkt);
size_t protocol_callback_queue_size(void);

uint8_t serial_write(const uint8_t *data, uint16_t len);
void on_receive_TaskStatus(const Packet_TaskStatus *pkt);
void on_receive_TargetControl(const Packet_TargetControl *pkt);

#endif
