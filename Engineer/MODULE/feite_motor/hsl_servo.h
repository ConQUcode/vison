#ifndef HSL_SERVO_H
#define HSL_SERVO_H

#include <stdint.h>

#define HSL_SERVO_MIN_ID             1u
#define HSL_SERVO_MAX_ID           253u
#define HSL_SERVO_MIN_POSITION       0u
#define HSL_SERVO_MAX_POSITION    1000u
#define HSL_SERVO_MAX_TIME_MS     30000u

typedef enum {
    HSL_SERVO_RESULT_OK = 0,
    HSL_SERVO_RESULT_BUSY,
    HSL_SERVO_RESULT_INVALID,
    HSL_SERVO_RESULT_NOT_INITIALIZED,
    HSL_SERVO_RESULT_HAL_ERROR,
    HSL_SERVO_RESULT_TIMEOUT,
    HSL_SERVO_RESULT_FRAME_ERROR,
    HSL_SERVO_RESULT_CHECKSUM_ERROR,
    HSL_SERVO_RESULT_ID_MISMATCH,
    HSL_SERVO_RESULT_COMMAND_MISMATCH,
    HSL_SERVO_RESULT_POSITION_RANGE,
    HSL_SERVO_RESULT_NOT_FOUND,
    HSL_SERVO_RESULT_UNSUPPORTED
} HSLServo_Result_e;

typedef enum {
    HSL_SERVO_STATE_UNINITIALIZED = 0,
    HSL_SERVO_STATE_IDLE,
    HSL_SERVO_STATE_TX_DMA,
    HSL_SERVO_STATE_WAIT_TX_COMPLETE,
    HSL_SERVO_STATE_RX_DMA,
    HSL_SERVO_STATE_WAIT_RESPONSE,
    HSL_SERVO_STATE_VALIDATE,
    HSL_SERVO_STATE_COMPLETE,
    HSL_SERVO_STATE_TIMEOUT,
    HSL_SERVO_STATE_ERROR
} HSLServo_State_e;

typedef enum {
    HSL_SERVO_COMMAND_NONE = 0,
    HSL_SERVO_COMMAND_MOVE = 3,
    HSL_SERVO_COMMAND_STOP = 12,
    HSL_SERVO_COMMAND_POSITION_READ = 21
} HSLServo_Command_e;

typedef struct {
    uint8_t id;
    uint8_t online;
    uint8_t position_valid;
    HSLServo_Command_e last_command;
    HSLServo_Result_e last_result;
    uint16_t position;
    uint32_t last_response_tick;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t timeout_count;
    uint32_t checksum_fail_count;
    uint32_t frame_fail_count;
} HSLServo_Status_s;

typedef struct {
    uint8_t initialized;
    uint8_t busy;
    uint8_t current_id;
    HSLServo_State_e state;
    HSLServo_Command_e current_command;
    HSLServo_Command_e last_command;
    HSLServo_Result_e last_result;
    uint16_t last_position;
    uint8_t last_position_valid;
    uint8_t online;
    uint8_t tx_length;
    uint8_t rx_length;
    uint8_t tx_frame[16];
    uint8_t rx_frame[16];
    uint32_t transaction_start_tick;
    uint32_t last_response_tick;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t timeout_count;
    uint32_t checksum_fail_count;
    uint32_t frame_fail_count;
    uint32_t hal_error_count;
    uint32_t legacy_reject_count;
} HSLServo_Debug_s;

extern HSLServo_Debug_s g_hsl_servo_debug;

uint8_t HSLServoInit(void);
HSLServo_Result_e HSLServoMove(uint8_t id,
                               uint16_t position,
                               uint16_t time_ms);
HSLServo_Result_e HSLServoStop(uint8_t id);
HSLServo_Result_e HSLServoRequestPosition(uint8_t id);
uint8_t HSLServoGetStatus(uint8_t id, HSLServo_Status_s *status);
void HSLServoTask(uint32_t now_ms);

/*
 * 旧飞特/SCS接口仅用于保持现有catch.c可编译。本版本统一拒绝且不发送，
 * 防止将旧0..1500位置/速度/力矩参数误解释为幻尔LX协议。
 */
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
