#ifndef HSL_SERVO_H
#define HSL_SERVO_H

#include <stdint.h>

#define HSL_SERVO_MIN_ID                         1u
#define HSL_SERVO_MAX_ID                       253u
#define HSL_SERVO_MIN_POSITION                   0u
#define HSL_SERVO_MAX_POSITION                1000u
#define HSL_SERVO_MAX_TIME_MS                 30000u
#define HSL_SERVO_MAX_FEEDBACK_IDS                2u

#define HSL_SERVO_CONTROLLER_BAUD_RATE          9600u
#define HSL_SERVO_DEFAULT_POLL_PERIOD_MS         50u
#define HSL_SERVO_DEFAULT_VOLTAGE_POLL_PERIOD_MS 500u
#define HSL_SERVO_FEEDBACK_STALE_MS              100u
#define HSL_SERVO_OFFLINE_MS                     200u
#define HSL_SERVO_BOARD_VOLTAGE_STALE_MS        1500u
#define HSL_SERVO_BOARD_VOLTAGE_MIN_MV          3000u
#define HSL_SERVO_BOARD_VOLTAGE_MAX_MV         15000u
#define HSL_SERVO_ARRIVAL_TOLERANCE_POS           15u
#define HSL_SERVO_ARRIVAL_VELOCITY_POS_S       100.0f
#define HSL_SERVO_ARRIVAL_STABLE_MS              120u
#define HSL_SERVO_MOTION_TIMEOUT_MARGIN_MS        500u

typedef enum {
    HSL_SERVO_RESULT_OK = 0,
    HSL_SERVO_RESULT_BUSY,
    HSL_SERVO_RESULT_INVALID,
    HSL_SERVO_RESULT_NOT_INITIALIZED,
    HSL_SERVO_RESULT_HAL_ERROR,
    HSL_SERVO_RESULT_TIMEOUT,
    HSL_SERVO_RESULT_FRAME_ERROR,
    HSL_SERVO_RESULT_CHECKSUM_ERROR, /* 兼容保留；控制板协议没有校验字段。 */
    HSL_SERVO_RESULT_ID_MISMATCH,
    HSL_SERVO_RESULT_COMMAND_MISMATCH,
    HSL_SERVO_RESULT_POSITION_RANGE,
    HSL_SERVO_RESULT_VOLTAGE_RANGE,
    HSL_SERVO_RESULT_NOT_FOUND,
    HSL_SERVO_RESULT_UNSUPPORTED
} HSLServo_Result_e;

typedef enum {
    HSL_SERVO_STATE_UNINITIALIZED = 0,
    HSL_SERVO_STATE_IDLE,
    HSL_SERVO_STATE_PREPARE_RX,
    HSL_SERVO_STATE_TX_DMA,
    HSL_SERVO_STATE_WAIT_TX_COMPLETE,
    HSL_SERVO_STATE_WAIT_TX_GAP,
    HSL_SERVO_STATE_WAIT_RESPONSE,
    HSL_SERVO_STATE_VALIDATE,
    HSL_SERVO_STATE_COMPLETE,
    HSL_SERVO_STATE_TIMEOUT,
    HSL_SERVO_STATE_ERROR
} HSLServo_State_e;

typedef enum {
    HSL_SERVO_COMMAND_NONE = 0,
    HSL_SERVO_COMMAND_MOVE = 3,               /* CMD_SERVO_MOVE，0x03。 */
    /* 兼容旧接口的内部标记；控制板协议没有按单个ID停止舵机命令。 */
    HSL_SERVO_COMMAND_STOP = 12,
    HSL_SERVO_COMMAND_BOARD_VOLTAGE_READ = 15, /* 0x0F。 */
    HSL_SERVO_COMMAND_POSITION_READ = 21       /* 0x15。 */
} HSLServo_Command_e;

typedef struct {
    uint8_t id;
    uint8_t online;
    uint8_t feedback_valid;
    uint8_t target_valid;
    uint8_t arrived;
    uint8_t motion_timeout;
    HSLServo_Command_e last_command;
    HSLServo_Result_e last_result;
    uint16_t target_position;
    uint16_t feedback_position;
    int16_t position_error;
    float feedback_velocity_pos_s;
    uint16_t command_time_ms;
    uint32_t last_command_tick;
    uint32_t last_feedback_tick;
    uint32_t feedback_sequence;
    uint32_t arrival_stable_since_tick;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t timeout_count;
    uint32_t frame_fail_count;
} HSLServo_Status_s;

typedef struct {
    uint8_t initialized;
    uint8_t uart_config_valid;
    uint8_t busy;
    uint8_t current_id;
    uint8_t current_id_count;
    uint8_t current_ids[HSL_SERVO_MAX_FEEDBACK_IDS];
    HSLServo_State_e state;
    HSLServo_Command_e current_command;
    HSLServo_Command_e last_command;
    HSLServo_Result_e last_result;
    uint16_t last_position;
    uint8_t last_position_valid;
    uint8_t online;
    uint8_t protocol_self_test_passed;
    uint8_t expected_rx_length;
    uint8_t poll_enabled;
    uint8_t poll_id_count;
    uint8_t poll_ids[HSL_SERVO_MAX_FEEDBACK_IDS];
    uint16_t poll_period_ms;
    uint8_t board_voltage_poll_enabled;
    uint8_t board_voltage_valid;
    uint16_t board_voltage_poll_period_ms;
    uint16_t board_voltage_mv;
    uint32_t uart_baud_rate;
    uint8_t tx_length;
    uint8_t rx_length;
    uint8_t tx_frame[32];
    uint8_t rx_frame[32];
    uint32_t transaction_start_tick;
    uint32_t last_response_tick;
    uint32_t last_poll_tick;
    uint32_t poll_request_count;
    uint32_t last_board_voltage_tick;
    uint32_t board_voltage_request_count;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t timeout_count;
    uint32_t frame_fail_count;
    uint32_t hal_error_count;
    uint32_t recovery_count;
    uint32_t legacy_reject_count;
} HSLServo_Debug_s;

extern HSLServo_Debug_s g_hsl_servo_debug;

uint8_t HSLServoInit(void);
HSLServo_Result_e HSLServoMove(uint8_t id,
                               uint16_t position,
                               uint16_t time_ms);
HSLServo_Result_e HSLServoMove2(uint8_t id1,
                                uint16_t position1,
                                uint8_t id2,
                                uint16_t position2,
                                uint16_t time_ms);
HSLServo_Result_e HSLServoStop(uint8_t id);
HSLServo_Result_e HSLServoRequestPosition(uint8_t id);
HSLServo_Result_e HSLServoRequestPositions(const uint8_t *ids,
                                            uint8_t count);
HSLServo_Result_e HSLServoRequestBoardVoltage(void);
/*
 * period_ms表示每个ID的目标反馈周期；配置多个ID时驱动会拆成单ID帧
 * 依次轮询，避免一台舵机无应答导致其他舵机的在线状态一起失效。
 */
uint8_t HSLServoConfigureFeedbackPolling(const uint8_t *ids,
                                          uint8_t count,
                                          uint16_t period_ms);
void HSLServoDisableFeedbackPolling(void);
uint8_t HSLServoConfigureBoardVoltagePolling(uint16_t period_ms);
void HSLServoDisableBoardVoltagePolling(void);
uint8_t HSLServoGetBoardVoltage(uint16_t *voltage_mv);
uint8_t HSLServoGetStatus(uint8_t id, HSLServo_Status_s *status);
uint8_t HSLServoFeedbackFresh(uint8_t id, uint32_t now_ms);
void HSLServoTask(uint32_t now_ms);

/* Legacy Feetech/SCS APIs remain non-transmitting compatibility stubs. */
void setEnd(uint8_t end);
void setLevel(uint8_t level);
int getLastError(void);
int genWrite(uint8_t id, uint8_t address, uint8_t *data, uint8_t length);
int writeWord(uint8_t id, uint8_t address, uint16_t value);
int WritePosEx2(uint8_t id, int16_t position, uint16_t speed,
                uint8_t acceleration, uint16_t torque);
void SyncWritePosEx2(uint8_t id[], uint8_t id_count, int16_t position[],
                     uint16_t speed[], uint8_t acceleration[],
                     uint16_t torque[]);
int Read(uint8_t id, uint8_t address, uint8_t *data, uint8_t length);
int readByte(uint8_t id, uint8_t address);
int readWord(uint8_t id, uint8_t address);
int Ping(uint8_t id);
void rFlushSCS(void);
void wFlushSCS(void);

#endif
