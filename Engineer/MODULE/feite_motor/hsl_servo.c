#include "hsl_servo.h"

#include "bsp_usart.h"
#include "usart.h"

#include <string.h>

#define HSL_SERVO_HEADER                    0x55u
#define HSL_SERVO_MOVE_FRAME_LENGTH            10u
#define HSL_SERVO_MOVE2_FRAME_LENGTH           13u
#define HSL_SERVO_SHORT_FRAME_LENGTH            6u
#define HSL_SERVO_POSITION_REPLY_LENGTH          8u
#define HSL_SERVO_RX_BUFFER_LENGTH              16u
#define HSL_SERVO_TX_BUFFER_LENGTH              16u
#define HSL_SERVO_TX_TIMEOUT_MS                 HAL_MAX_DELAY
#define HSL_SERVO_BOARD_TX_GAP_MS                2u
#define HSL_SERVO_RESPONSE_TIMEOUT_MS           30u
#define HSL_SERVO_STATUS_SLOT_COUNT              8u
#define HSL_SERVO_LEGACY_ERROR_UNSUPPORTED      (-1)
#define HSL_SERVO_BOARD_MOVE_DATA_LENGTH         8u
#define HSL_SERVO_BOARD_MOVE_SERVO_COUNT         1u
#define HSL_SERVO_BOARD_MOVE2_DATA_LENGTH       11u
#define HSL_SERVO_BOARD_MOVE2_SERVO_COUNT        2u

typedef struct {
    uint8_t initialized;
    volatile uint8_t tx_complete;
    volatile uint8_t rx_complete;
    volatile uint8_t uart_error;
    volatile uint16_t rx_size;
    uint8_t rx_armed;
    uint8_t expects_response;
    uint8_t expected_command;
    uint8_t tx_buffer[HSL_SERVO_TX_BUFFER_LENGTH];
    uint8_t rx_buffer[HSL_SERVO_RX_BUFFER_LENGTH];
    uint8_t tx_length;
    uint32_t state_tick;
    HSLServo_Status_s status[HSL_SERVO_STATUS_SLOT_COUNT];
} HSLServo_Runtime_s;

HSLServo_Debug_s g_hsl_servo_debug;

static HSLServo_Runtime_s hsl_runtime;
static int hsl_legacy_last_error = HSL_SERVO_LEGACY_ERROR_UNSUPPORTED;

static uint8_t HSLServoIdValid(uint8_t id)
{
    return id >= HSL_SERVO_MIN_ID && id <= HSL_SERVO_MAX_ID;
}

static uint8_t HSLServoChecksum(const uint8_t *frame, uint8_t length)
{
    uint16_t sum = 0u;
    uint8_t index;

    if (frame == NULL || length < HSL_SERVO_SHORT_FRAME_LENGTH) {
        return 0u;
    }
    for (index = 2u; index < (uint8_t)(length - 1u); ++index) {
        sum += frame[index];
    }
    return (uint8_t)(~sum);
}

static HSLServo_Status_s *HSLServoFindStatus(uint8_t id, uint8_t create)
{
    HSLServo_Status_s *empty = NULL;
    uint8_t index;

    for (index = 0u; index < HSL_SERVO_STATUS_SLOT_COUNT; ++index) {
        HSLServo_Status_s *status = &hsl_runtime.status[index];

        if (status->id == id) {
            return status;
        }
        if (empty == NULL && status->id == 0u) {
            empty = status;
        }
    }
    if (create == 0u || empty == NULL) {
        return NULL;
    }
    memset(empty, 0, sizeof(*empty));
    empty->id = id;
    empty->last_result = HSL_SERVO_RESULT_NOT_FOUND;
    return empty;
}

static HSLServo_Status_s *HSLServoCurrentStatus(void)
{
    return HSLServoFindStatus(g_hsl_servo_debug.current_id, 0u);
}

static void HSLServoUpdateDebugFrames(void)
{
    g_hsl_servo_debug.tx_length = hsl_runtime.tx_length;
    g_hsl_servo_debug.rx_length = (uint8_t)hsl_runtime.rx_size;
    memcpy(g_hsl_servo_debug.tx_frame, hsl_runtime.tx_buffer,
           sizeof(g_hsl_servo_debug.tx_frame));
    memcpy(g_hsl_servo_debug.rx_frame, hsl_runtime.rx_buffer,
           sizeof(g_hsl_servo_debug.rx_frame));
}

static void HSLServoFinish(HSLServo_Result_e result, uint32_t now_ms)
{
    HSLServo_Status_s *status = HSLServoCurrentStatus();

    if (hsl_runtime.rx_armed != 0u) {
        HAL_UART_AbortReceive(&huart6);
        hsl_runtime.rx_armed = 0u;
    }

    g_hsl_servo_debug.last_command =
        g_hsl_servo_debug.current_command;
    g_hsl_servo_debug.last_result = result;
    g_hsl_servo_debug.busy = 0u;
    if (result == HSL_SERVO_RESULT_OK) {
        g_hsl_servo_debug.state = HSL_SERVO_STATE_COMPLETE;
    } else if (result == HSL_SERVO_RESULT_TIMEOUT) {
        g_hsl_servo_debug.state = HSL_SERVO_STATE_TIMEOUT;
    } else {
        g_hsl_servo_debug.state = HSL_SERVO_STATE_ERROR;
    }
    if (status != NULL) {
        status->last_command = g_hsl_servo_debug.current_command;
        status->last_result = result;
        if (g_hsl_servo_debug.current_command ==
            HSL_SERVO_COMMAND_POSITION_READ) {
            if (result == HSL_SERVO_RESULT_OK) {
                status->online = 1u;
                status->position_valid = 1u;
                status->position = g_hsl_servo_debug.last_position;
                status->last_response_tick = now_ms;
            } else {
                status->online = 0u;
            }
        }
        g_hsl_servo_debug.online = status->online;
        g_hsl_servo_debug.last_response_tick = status->last_response_tick;
    }
    g_hsl_servo_debug.current_command = HSL_SERVO_COMMAND_NONE;
    hsl_runtime.rx_armed = 0u;
    hsl_runtime.expects_response = 0u;
}

static HSLServo_Result_e HSLServoQueue(uint8_t id,
                                       HSLServo_Command_e command,
                                       const uint8_t *parameters,
                                       uint8_t parameter_count,
                                       uint8_t expects_response)
{
    uint32_t primask;
    uint8_t length_field;
    uint8_t frame_length;
    uint8_t index;

    if (hsl_runtime.initialized == 0u) {
        return HSL_SERVO_RESULT_NOT_INITIALIZED;
    }
    if (!HSLServoIdValid(id) ||
        parameter_count >
            HSL_SERVO_TX_BUFFER_LENGTH - HSL_SERVO_SHORT_FRAME_LENGTH) {
        return HSL_SERVO_RESULT_INVALID;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    if (g_hsl_servo_debug.busy != 0u) {
        if (primask == 0u) {
            __enable_irq();
        }
        return HSL_SERVO_RESULT_BUSY;
    }
    memset(hsl_runtime.tx_buffer, 0, sizeof(hsl_runtime.tx_buffer));
    length_field = (uint8_t)(parameter_count + 3u);
    frame_length = (uint8_t)(length_field + 3u);
    hsl_runtime.tx_buffer[0] = HSL_SERVO_HEADER;
    hsl_runtime.tx_buffer[1] = HSL_SERVO_HEADER;
    hsl_runtime.tx_buffer[2] = id;
    hsl_runtime.tx_buffer[3] = length_field;
    hsl_runtime.tx_buffer[4] = (uint8_t)command;
    for (index = 0u; index < parameter_count; ++index) {
        hsl_runtime.tx_buffer[5u + index] = parameters[index];
    }
    hsl_runtime.tx_buffer[frame_length - 1u] =
        HSLServoChecksum(hsl_runtime.tx_buffer, frame_length);
    hsl_runtime.tx_length = frame_length;
    hsl_runtime.expects_response = 0u;
    hsl_runtime.expected_command = (uint8_t)command;
    hsl_runtime.tx_complete = 0u;
    hsl_runtime.rx_complete = 0u;
    hsl_runtime.uart_error = 0u;
    hsl_runtime.rx_size = 0u;
    hsl_runtime.rx_armed = 0u;
    hsl_runtime.state_tick = HAL_GetTick();
    memset(hsl_runtime.rx_buffer, 0, sizeof(hsl_runtime.rx_buffer));
    g_hsl_servo_debug.current_id = id;
    g_hsl_servo_debug.current_command = command;
    g_hsl_servo_debug.last_result = HSL_SERVO_RESULT_BUSY;
    g_hsl_servo_debug.busy = 1u;
    g_hsl_servo_debug.state = HSL_SERVO_STATE_TX_DMA;
    g_hsl_servo_debug.transaction_start_tick = hsl_runtime.state_tick;
    HSLServoFindStatus(id, 1u);
    HSLServoUpdateDebugFrames();
    if (primask == 0u) {
        __enable_irq();
    }
    return HSL_SERVO_RESULT_OK;
}

static HSLServo_Result_e HSLServoQueueBoardMove(uint8_t id,
                                                uint16_t position,
                                                uint16_t time_ms)
{
    uint32_t primask;

    if (hsl_runtime.initialized == 0u) {
        return HSL_SERVO_RESULT_NOT_INITIALIZED;
    }
    if (!HSLServoIdValid(id) || position > HSL_SERVO_MAX_POSITION ||
        time_ms > HSL_SERVO_MAX_TIME_MS) {
        return HSL_SERVO_RESULT_INVALID;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (g_hsl_servo_debug.busy != 0u) {
        if (primask == 0u) {
            __enable_irq();
        }
        return HSL_SERVO_RESULT_BUSY;
    }

    memset(hsl_runtime.tx_buffer, 0, sizeof(hsl_runtime.tx_buffer));
    hsl_runtime.tx_buffer[0] = HSL_SERVO_HEADER;
    hsl_runtime.tx_buffer[1] = HSL_SERVO_HEADER;
    hsl_runtime.tx_buffer[2] = HSL_SERVO_BOARD_MOVE_DATA_LENGTH;
    hsl_runtime.tx_buffer[3] = (uint8_t)HSL_SERVO_COMMAND_MOVE;
    hsl_runtime.tx_buffer[4] = HSL_SERVO_BOARD_MOVE_SERVO_COUNT;
    hsl_runtime.tx_buffer[5] = (uint8_t)(time_ms & 0xffu);
    hsl_runtime.tx_buffer[6] = (uint8_t)(time_ms >> 8u);
    hsl_runtime.tx_buffer[7] = id;
    hsl_runtime.tx_buffer[8] = (uint8_t)(position & 0xffu);
    hsl_runtime.tx_buffer[9] = (uint8_t)(position >> 8u);
    hsl_runtime.tx_length = HSL_SERVO_MOVE_FRAME_LENGTH;
    hsl_runtime.expects_response = 0u;
    hsl_runtime.expected_command = (uint8_t)HSL_SERVO_COMMAND_MOVE;
    hsl_runtime.tx_complete = 0u;
    hsl_runtime.rx_complete = 0u;
    hsl_runtime.uart_error = 0u;
    hsl_runtime.rx_size = 0u;
    hsl_runtime.rx_armed = 0u;
    hsl_runtime.state_tick = HAL_GetTick();
    memset(hsl_runtime.rx_buffer, 0, sizeof(hsl_runtime.rx_buffer));
    g_hsl_servo_debug.current_id = id;
    g_hsl_servo_debug.current_command = HSL_SERVO_COMMAND_MOVE;
    g_hsl_servo_debug.last_result = HSL_SERVO_RESULT_BUSY;
    g_hsl_servo_debug.busy = 1u;
    g_hsl_servo_debug.state = HSL_SERVO_STATE_TX_DMA;
    g_hsl_servo_debug.transaction_start_tick = hsl_runtime.state_tick;
    HSLServoFindStatus(id, 1u);
    HSLServoUpdateDebugFrames();
    if (primask == 0u) {
        __enable_irq();
    }
    return HSL_SERVO_RESULT_OK;
}

static HSLServo_Result_e HSLServoValidatePositionReply(uint32_t now_ms)
{
    HSLServo_Result_e candidate_result = HSL_SERVO_RESULT_FRAME_ERROR;
    uint16_t position;
    uint8_t expected_length;
    uint8_t index;

    if (hsl_runtime.rx_size < HSL_SERVO_POSITION_REPLY_LENGTH) {
        return HSL_SERVO_RESULT_FRAME_ERROR;
    }
    for (index = 0u;
         index + HSL_SERVO_POSITION_REPLY_LENGTH <= hsl_runtime.rx_size;
         ++index) {
        const uint8_t *frame = &hsl_runtime.rx_buffer[index];

        if (frame[0] != HSL_SERVO_HEADER ||
            frame[1] != HSL_SERVO_HEADER) {
            continue;
        }
        expected_length = (uint8_t)(frame[3] + 3u);
        if (frame[3] != 5u ||
            expected_length != HSL_SERVO_POSITION_REPLY_LENGTH ||
            index + expected_length > hsl_runtime.rx_size) {
            continue;
        }
        if (frame[2] != g_hsl_servo_debug.current_id) {
            candidate_result = HSL_SERVO_RESULT_ID_MISMATCH;
            continue;
        }
        if (frame[4] != hsl_runtime.expected_command) {
            candidate_result = HSL_SERVO_RESULT_COMMAND_MISMATCH;
            continue;
        }
        if (frame[expected_length - 1u] !=
            HSLServoChecksum(frame, expected_length)) {
            candidate_result = HSL_SERVO_RESULT_CHECKSUM_ERROR;
            continue;
        }
        position = (uint16_t)frame[5] | ((uint16_t)frame[6] << 8u);
        if (position > HSL_SERVO_MAX_POSITION) {
            candidate_result = HSL_SERVO_RESULT_POSITION_RANGE;
            continue;
        }
        g_hsl_servo_debug.last_position = position;
        g_hsl_servo_debug.last_position_valid = 1u;
        g_hsl_servo_debug.last_response_tick = now_ms;
        return HSL_SERVO_RESULT_OK;
    }
    return candidate_result;
}

static void HSLServoRxEvent(uint16_t size)
{
    if (size > HSL_SERVO_RX_BUFFER_LENGTH) {
        size = HSL_SERVO_RX_BUFFER_LENGTH;
    }
    hsl_runtime.rx_size = size;
    hsl_runtime.rx_armed = 0u;
    hsl_runtime.rx_complete = 1u;
}

static void HSLServoTxComplete(void)
{
    hsl_runtime.tx_complete = 1u;
}

static void HSLServoUartError(void)
{
    hsl_runtime.uart_error = 1u;
}

static HAL_StatusTypeDef HSLServoStartReceive(uint32_t now_ms,
                                              uint8_t enter_wait_state)
{
    HAL_StatusTypeDef result;

    if (huart6.hdmarx == NULL) {
        (void)now_ms;
        (void)enter_wait_state;
        return HAL_ERROR;
    }
    /* 清除前一事务或外部转换板回显可能遗留的RXNE/ORE，再交给DMA。 */
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    memset(hsl_runtime.rx_buffer, 0, sizeof(hsl_runtime.rx_buffer));
    hsl_runtime.rx_size = 0u;
    hsl_runtime.rx_complete = 0u;
    result = HAL_UART_Receive_DMA(
        &huart6, hsl_runtime.rx_buffer, HSL_SERVO_RX_BUFFER_LENGTH);
    if (result == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
        hsl_runtime.rx_armed = 1u;
        hsl_runtime.state_tick = now_ms;
        if (enter_wait_state != 0u) {
            g_hsl_servo_debug.state = HSL_SERVO_STATE_WAIT_RESPONSE;
        }
    }
    return result;
}

static HSLServo_Result_e HSLServoTransmitBoardFrame(uint8_t id,
                                                    uint16_t position,
                                                    uint16_t time_ms)
{
    HAL_StatusTypeDef hal_result;
    HSLServo_Status_s *status;

    if (hsl_runtime.initialized == 0u) {
        return HSL_SERVO_RESULT_NOT_INITIALIZED;
    }
    if (!HSLServoIdValid(id) || position > HSL_SERVO_MAX_POSITION ||
        time_ms > HSL_SERVO_MAX_TIME_MS) {
        return HSL_SERVO_RESULT_INVALID;
    }

    memset(hsl_runtime.tx_buffer, 0, sizeof(hsl_runtime.tx_buffer));
    hsl_runtime.tx_buffer[0] = HSL_SERVO_HEADER;
    hsl_runtime.tx_buffer[1] = HSL_SERVO_HEADER;
    hsl_runtime.tx_buffer[2] = HSL_SERVO_BOARD_MOVE_DATA_LENGTH;
    hsl_runtime.tx_buffer[3] = (uint8_t)HSL_SERVO_COMMAND_MOVE;
    hsl_runtime.tx_buffer[4] = HSL_SERVO_BOARD_MOVE_SERVO_COUNT;
    hsl_runtime.tx_buffer[5] = (uint8_t)(time_ms & 0xffu);
    hsl_runtime.tx_buffer[6] = (uint8_t)(time_ms >> 8u);
    hsl_runtime.tx_buffer[7] = id;
    hsl_runtime.tx_buffer[8] = (uint8_t)(position & 0xffu);
    hsl_runtime.tx_buffer[9] = (uint8_t)(position >> 8u);
    hsl_runtime.tx_length = HSL_SERVO_MOVE_FRAME_LENGTH;
    hsl_runtime.expects_response = 0u;
    hsl_runtime.expected_command = (uint8_t)HSL_SERVO_COMMAND_MOVE;
    hsl_runtime.tx_complete = 0u;
    hsl_runtime.rx_complete = 0u;
    hsl_runtime.uart_error = 0u;
    hsl_runtime.rx_size = 0u;
    hsl_runtime.rx_armed = 0u;
    memset(hsl_runtime.rx_buffer, 0, sizeof(hsl_runtime.rx_buffer));

    g_hsl_servo_debug.current_id = id;
    g_hsl_servo_debug.current_command = HSL_SERVO_COMMAND_MOVE;
    g_hsl_servo_debug.last_command = HSL_SERVO_COMMAND_MOVE;
    g_hsl_servo_debug.last_result = HSL_SERVO_RESULT_BUSY;
    g_hsl_servo_debug.busy = 1u;
    g_hsl_servo_debug.state = HSL_SERVO_STATE_TX_DMA;
    g_hsl_servo_debug.transaction_start_tick = HAL_GetTick();
    g_hsl_servo_debug.last_position = position;
    g_hsl_servo_debug.last_position_valid = 1u;
    HSLServoUpdateDebugFrames();

    if (huart6.hdmarx != NULL) {
        HAL_UART_DMAStop(&huart6);
    }

    hal_result = HAL_UART_Transmit(&huart6,
                                   hsl_runtime.tx_buffer,
                                   hsl_runtime.tx_length,
                                   HSL_SERVO_TX_TIMEOUT_MS);
    if (hal_result == HAL_OK) {
        while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) == RESET) {
        }
        HAL_Delay(HSL_SERVO_BOARD_TX_GAP_MS);
        hsl_runtime.tx_complete = 1u;
        g_hsl_servo_debug.tx_count++;
        g_hsl_servo_debug.last_result = HSL_SERVO_RESULT_OK;
        g_hsl_servo_debug.busy = 0u;
        g_hsl_servo_debug.state = HSL_SERVO_STATE_COMPLETE;
        status = HSLServoFindStatus(id, 1u);
        if (status != NULL) {
            status->last_command = HSL_SERVO_COMMAND_MOVE;
            status->last_result = HSL_SERVO_RESULT_OK;
            status->position = position;
            status->position_valid = 1u;
            status->online = 1u;
            status->tx_count++;
        }
        g_hsl_servo_debug.online = 1u;
        g_hsl_servo_debug.current_command = HSL_SERVO_COMMAND_NONE;
        return HSL_SERVO_RESULT_OK;
    }

    g_hsl_servo_debug.hal_error_count++;
    g_hsl_servo_debug.last_result = HSL_SERVO_RESULT_HAL_ERROR;
    g_hsl_servo_debug.busy = 0u;
    g_hsl_servo_debug.state = HSL_SERVO_STATE_ERROR;
    g_hsl_servo_debug.current_command = HSL_SERVO_COMMAND_NONE;
    status = HSLServoFindStatus(id, 1u);
    if (status != NULL) {
        status->last_command = HSL_SERVO_COMMAND_MOVE;
        status->last_result = HSL_SERVO_RESULT_HAL_ERROR;
    }
    return HSL_SERVO_RESULT_HAL_ERROR;
}

static HSLServo_Result_e HSLServoTransmitBoardFrame2(uint8_t id1,
                                                     uint16_t position1,
                                                     uint8_t id2,
                                                     uint16_t position2,
                                                     uint16_t time_ms)
{
    HAL_StatusTypeDef hal_result;
    HSLServo_Status_s *status;

    if (hsl_runtime.initialized == 0u) {
        return HSL_SERVO_RESULT_NOT_INITIALIZED;
    }
    if (!HSLServoIdValid(id1) || !HSLServoIdValid(id2) || id1 == id2 ||
        position1 > HSL_SERVO_MAX_POSITION ||
        position2 > HSL_SERVO_MAX_POSITION ||
        time_ms > HSL_SERVO_MAX_TIME_MS) {
        return HSL_SERVO_RESULT_INVALID;
    }

    memset(hsl_runtime.tx_buffer, 0, sizeof(hsl_runtime.tx_buffer));
    hsl_runtime.tx_buffer[0] = HSL_SERVO_HEADER;
    hsl_runtime.tx_buffer[1] = HSL_SERVO_HEADER;
    hsl_runtime.tx_buffer[2] = HSL_SERVO_BOARD_MOVE2_DATA_LENGTH;
    hsl_runtime.tx_buffer[3] = (uint8_t)HSL_SERVO_COMMAND_MOVE;
    hsl_runtime.tx_buffer[4] = HSL_SERVO_BOARD_MOVE2_SERVO_COUNT;
    hsl_runtime.tx_buffer[5] = (uint8_t)(time_ms & 0xffu);
    hsl_runtime.tx_buffer[6] = (uint8_t)(time_ms >> 8u);
    hsl_runtime.tx_buffer[7] = id1;
    hsl_runtime.tx_buffer[8] = (uint8_t)(position1 & 0xffu);
    hsl_runtime.tx_buffer[9] = (uint8_t)(position1 >> 8u);
    hsl_runtime.tx_buffer[10] = id2;
    hsl_runtime.tx_buffer[11] = (uint8_t)(position2 & 0xffu);
    hsl_runtime.tx_buffer[12] = (uint8_t)(position2 >> 8u);
    hsl_runtime.tx_length = HSL_SERVO_MOVE2_FRAME_LENGTH;
    hsl_runtime.expects_response = 0u;
    hsl_runtime.expected_command = (uint8_t)HSL_SERVO_COMMAND_MOVE;
    hsl_runtime.tx_complete = 0u;
    hsl_runtime.rx_complete = 0u;
    hsl_runtime.uart_error = 0u;
    hsl_runtime.rx_size = 0u;
    hsl_runtime.rx_armed = 0u;
    memset(hsl_runtime.rx_buffer, 0, sizeof(hsl_runtime.rx_buffer));

    g_hsl_servo_debug.current_id = id2;
    g_hsl_servo_debug.current_command = HSL_SERVO_COMMAND_MOVE;
    g_hsl_servo_debug.last_command = HSL_SERVO_COMMAND_MOVE;
    g_hsl_servo_debug.last_result = HSL_SERVO_RESULT_BUSY;
    g_hsl_servo_debug.busy = 1u;
    g_hsl_servo_debug.state = HSL_SERVO_STATE_TX_DMA;
    g_hsl_servo_debug.transaction_start_tick = HAL_GetTick();
    g_hsl_servo_debug.last_position = position2;
    g_hsl_servo_debug.last_position_valid = 1u;
    HSLServoUpdateDebugFrames();

    if (huart6.hdmarx != NULL) {
        HAL_UART_DMAStop(&huart6);
    }

    hal_result = HAL_UART_Transmit(&huart6,
                                   hsl_runtime.tx_buffer,
                                   hsl_runtime.tx_length,
                                   HSL_SERVO_TX_TIMEOUT_MS);
    if (hal_result == HAL_OK) {
        while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) == RESET) {
        }
        HAL_Delay(HSL_SERVO_BOARD_TX_GAP_MS);
        hsl_runtime.tx_complete = 1u;
        g_hsl_servo_debug.tx_count++;
        g_hsl_servo_debug.last_result = HSL_SERVO_RESULT_OK;
        g_hsl_servo_debug.busy = 0u;
        g_hsl_servo_debug.state = HSL_SERVO_STATE_COMPLETE;
        status = HSLServoFindStatus(id1, 1u);
        if (status != NULL) {
            status->last_command = HSL_SERVO_COMMAND_MOVE;
            status->last_result = HSL_SERVO_RESULT_OK;
            status->position = position1;
            status->position_valid = 1u;
            status->online = 1u;
            status->tx_count++;
        }
        status = HSLServoFindStatus(id2, 1u);
        if (status != NULL) {
            status->last_command = HSL_SERVO_COMMAND_MOVE;
            status->last_result = HSL_SERVO_RESULT_OK;
            status->position = position2;
            status->position_valid = 1u;
            status->online = 1u;
            status->tx_count++;
        }
        g_hsl_servo_debug.online = 1u;
        g_hsl_servo_debug.current_command = HSL_SERVO_COMMAND_NONE;
        return HSL_SERVO_RESULT_OK;
    }

    g_hsl_servo_debug.hal_error_count++;
    g_hsl_servo_debug.last_result = HSL_SERVO_RESULT_HAL_ERROR;
    g_hsl_servo_debug.busy = 0u;
    g_hsl_servo_debug.state = HSL_SERVO_STATE_ERROR;
    g_hsl_servo_debug.current_command = HSL_SERVO_COMMAND_NONE;
    status = HSLServoFindStatus(id1, 1u);
    if (status != NULL) {
        status->last_command = HSL_SERVO_COMMAND_MOVE;
        status->last_result = HSL_SERVO_RESULT_HAL_ERROR;
    }
    status = HSLServoFindStatus(id2, 1u);
    if (status != NULL) {
        status->last_command = HSL_SERVO_COMMAND_MOVE;
        status->last_result = HSL_SERVO_RESULT_HAL_ERROR;
    }
    return HSL_SERVO_RESULT_HAL_ERROR;
}

uint8_t HSLServoInit(void)
{
    memset(&hsl_runtime, 0, sizeof(hsl_runtime));
    memset(&g_hsl_servo_debug, 0, sizeof(g_hsl_servo_debug));
    hsl_runtime.initialized = 1u;
    g_hsl_servo_debug.initialized = 1u;
    g_hsl_servo_debug.state = HSL_SERVO_STATE_IDLE;
    g_hsl_servo_debug.last_result = HSL_SERVO_RESULT_OK;
    return 1u;
}

HSLServo_Result_e HSLServoMove(uint8_t id,
                               uint16_t position,
                               uint16_t time_ms)
{
    return HSLServoTransmitBoardFrame(id, position, time_ms);
}

HSLServo_Result_e HSLServoMove2(uint8_t id1,
                                uint16_t position1,
                                uint8_t id2,
                                uint16_t position2,
                                uint16_t time_ms)
{
    return HSLServoTransmitBoardFrame2(id1, position1, id2, position2,
                                      time_ms);
}

HSLServo_Result_e HSLServoStop(uint8_t id)
{
    if (!HSLServoIdValid(id)) {
        return HSL_SERVO_RESULT_INVALID;
    }
    return HSL_SERVO_RESULT_UNSUPPORTED;
}

HSLServo_Result_e HSLServoRequestPosition(uint8_t id)
{
    if (!HSLServoIdValid(id)) {
        return HSL_SERVO_RESULT_INVALID;
    }
    if (huart6.hdmarx == NULL) {
        return HSL_SERVO_RESULT_UNSUPPORTED;
    }
    return HSLServoQueue(id, HSL_SERVO_COMMAND_POSITION_READ,
                         NULL, 0u, 1u);
}

uint8_t HSLServoGetStatus(uint8_t id, HSLServo_Status_s *status)
{
    HSLServo_Status_s *stored;
    uint32_t primask;

    if (!HSLServoIdValid(id) || status == NULL) {
        return 0u;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    stored = HSLServoFindStatus(id, 0u);
    if (stored != NULL) {
        *status = *stored;
    }
    if (primask == 0u) {
        __enable_irq();
    }
    return stored != NULL;
}

void HSLServoTask(uint32_t now_ms)
{
    HAL_StatusTypeDef hal_result;
    HSLServo_Result_e result;
    HSLServo_Status_s *status;

    if (hsl_runtime.initialized == 0u) {
        return;
    }
    if (hsl_runtime.uart_error != 0u) {
        hsl_runtime.uart_error = 0u;
        HAL_UART_AbortReceive(&huart6);
        HAL_UART_AbortTransmit(&huart6);
        g_hsl_servo_debug.hal_error_count++;
        if (g_hsl_servo_debug.busy != 0u) {
            HSLServoFinish(HSL_SERVO_RESULT_HAL_ERROR, now_ms);
        } else {
            g_hsl_servo_debug.last_result = HSL_SERVO_RESULT_HAL_ERROR;
            g_hsl_servo_debug.state = HSL_SERVO_STATE_ERROR;
        }
        return;
    }

    switch (g_hsl_servo_debug.state) {
        case HSL_SERVO_STATE_TX_DMA:
            if (hsl_runtime.expects_response != 0u &&
                hsl_runtime.rx_armed == 0u) {
                hal_result = HSLServoStartReceive(now_ms, 0u);
                if (hal_result == HAL_BUSY) {
                    if ((uint32_t)(now_ms - hsl_runtime.state_tick) >=
                        HSL_SERVO_TX_TIMEOUT_MS) {
                        g_hsl_servo_debug.timeout_count++;
                        status = HSLServoCurrentStatus();
                        if (status != NULL) {
                            status->timeout_count++;
                        }
                        HSLServoFinish(HSL_SERVO_RESULT_TIMEOUT, now_ms);
                    }
                    break;
                }
                if (hal_result != HAL_OK) {
                    g_hsl_servo_debug.hal_error_count++;
                    HSLServoFinish(HSL_SERVO_RESULT_HAL_ERROR, now_ms);
                    break;
                }
            }
            if (hsl_runtime.expects_response == 0u) {
                if (huart6.hdmarx != NULL) {
                    HAL_UART_DMAStop(&huart6);
                }
                hal_result = HAL_UART_Transmit(
                    &huart6, hsl_runtime.tx_buffer, hsl_runtime.tx_length,
                    HSL_SERVO_TX_TIMEOUT_MS);
                if (hal_result == HAL_OK) {
                    while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TC) ==
                           RESET) {
                    }
                    HAL_Delay(HSL_SERVO_BOARD_TX_GAP_MS);
                    hsl_runtime.tx_complete = 1u;
                    hsl_runtime.state_tick = now_ms;
                    g_hsl_servo_debug.state =
                        HSL_SERVO_STATE_WAIT_TX_COMPLETE;
                } else {
                    g_hsl_servo_debug.hal_error_count++;
                    HSLServoFinish(HSL_SERVO_RESULT_HAL_ERROR, now_ms);
                }
                break;
            }
            hal_result = HAL_UART_Transmit_DMA(
                &huart6, hsl_runtime.tx_buffer, hsl_runtime.tx_length);
            if (hal_result == HAL_OK) {
                hsl_runtime.state_tick = now_ms;
                g_hsl_servo_debug.state =
                    HSL_SERVO_STATE_WAIT_TX_COMPLETE;
            } else if (hal_result != HAL_BUSY) {
                g_hsl_servo_debug.hal_error_count++;
                HSLServoFinish(HSL_SERVO_RESULT_HAL_ERROR, now_ms);
            } else if ((uint32_t)(now_ms - hsl_runtime.state_tick) >=
                       HSL_SERVO_TX_TIMEOUT_MS) {
                g_hsl_servo_debug.timeout_count++;
                status = HSLServoCurrentStatus();
                if (status != NULL) {
                    status->timeout_count++;
                }
                HSLServoFinish(HSL_SERVO_RESULT_TIMEOUT, now_ms);
            }
            break;

        case HSL_SERVO_STATE_WAIT_TX_COMPLETE:
            if (hsl_runtime.tx_complete != 0u) {
                hsl_runtime.tx_complete = 0u;
                g_hsl_servo_debug.tx_count++;
                status = HSLServoCurrentStatus();
                if (status != NULL) {
                    status->tx_count++;
                }
                if (hsl_runtime.expects_response == 0u) {
                    HSLServoFinish(HSL_SERVO_RESULT_OK, now_ms);
                } else {
                    hsl_runtime.state_tick = now_ms;
                    g_hsl_servo_debug.state =
                        HSL_SERVO_STATE_WAIT_RESPONSE;
                }
            } else if ((uint32_t)(now_ms - hsl_runtime.state_tick) >=
                       HSL_SERVO_TX_TIMEOUT_MS) {
                HAL_UART_AbortTransmit(&huart6);
                g_hsl_servo_debug.timeout_count++;
                status = HSLServoCurrentStatus();
                if (status != NULL) {
                    status->timeout_count++;
                }
                HSLServoFinish(HSL_SERVO_RESULT_TIMEOUT, now_ms);
            }
            break;

        case HSL_SERVO_STATE_RX_DMA:
            hal_result = HSLServoStartReceive(now_ms, 1u);
            if (hal_result != HAL_OK && hal_result != HAL_BUSY) {
                g_hsl_servo_debug.hal_error_count++;
                HSLServoFinish(HSL_SERVO_RESULT_HAL_ERROR, now_ms);
            } else if ((uint32_t)(now_ms - hsl_runtime.state_tick) >=
                       HSL_SERVO_RESPONSE_TIMEOUT_MS) {
                HAL_UART_AbortReceive(&huart6);
                g_hsl_servo_debug.timeout_count++;
                status = HSLServoCurrentStatus();
                if (status != NULL) {
                    status->timeout_count++;
                }
                HSLServoFinish(HSL_SERVO_RESULT_TIMEOUT, now_ms);
            }
            break;

        case HSL_SERVO_STATE_WAIT_RESPONSE:
        {
            uint16_t received = hsl_runtime.rx_armed != 0u ?
                (uint16_t)(HSL_SERVO_RX_BUFFER_LENGTH -
                    __HAL_DMA_GET_COUNTER(huart6.hdmarx)) :
                hsl_runtime.rx_size;

            if (received > HSL_SERVO_RX_BUFFER_LENGTH) {
                received = HSL_SERVO_RX_BUFFER_LENGTH;
            }
            hsl_runtime.rx_size = received;
            if (received >= HSL_SERVO_POSITION_REPLY_LENGTH &&
                HSLServoValidatePositionReply(now_ms) ==
                    HSL_SERVO_RESULT_OK) {
                if (hsl_runtime.rx_armed != 0u) {
                    HAL_UART_AbortReceive(&huart6);
                    hsl_runtime.rx_armed = 0u;
                }
                g_hsl_servo_debug.rx_count++;
                status = HSLServoCurrentStatus();
                if (status != NULL) {
                    status->rx_count++;
                }
                HSLServoUpdateDebugFrames();
                g_hsl_servo_debug.state = HSL_SERVO_STATE_VALIDATE;
            } else if ((uint32_t)(now_ms - hsl_runtime.state_tick) >=
                       HSL_SERVO_RESPONSE_TIMEOUT_MS) {
                if (hsl_runtime.rx_armed != 0u) {
                    HAL_UART_AbortReceive(&huart6);
                    hsl_runtime.rx_armed = 0u;
                }
                if (received == 0u) {
                    g_hsl_servo_debug.timeout_count++;
                    status = HSLServoCurrentStatus();
                    if (status != NULL) {
                        status->timeout_count++;
                    }
                    HSLServoFinish(HSL_SERVO_RESULT_TIMEOUT, now_ms);
                } else {
                    g_hsl_servo_debug.rx_count++;
                    status = HSLServoCurrentStatus();
                    if (status != NULL) {
                        status->rx_count++;
                    }
                    HSLServoUpdateDebugFrames();
                    g_hsl_servo_debug.state = HSL_SERVO_STATE_VALIDATE;
                }
            }
            break;
        }

        case HSL_SERVO_STATE_VALIDATE:
            hsl_runtime.rx_complete = 0u;
            result = HSLServoValidatePositionReply(now_ms);
            status = HSLServoCurrentStatus();
            if (result == HSL_SERVO_RESULT_CHECKSUM_ERROR) {
                g_hsl_servo_debug.checksum_fail_count++;
                if (status != NULL) {
                    status->checksum_fail_count++;
                }
            } else if (result != HSL_SERVO_RESULT_OK) {
                g_hsl_servo_debug.frame_fail_count++;
                if (status != NULL) {
                    status->frame_fail_count++;
                }
            }
            HSLServoFinish(result, now_ms);
            break;

        case HSL_SERVO_STATE_UNINITIALIZED:
        case HSL_SERVO_STATE_IDLE:
        case HSL_SERVO_STATE_COMPLETE:
        case HSL_SERVO_STATE_TIMEOUT:
        case HSL_SERVO_STATE_ERROR:
        default:
            break;
    }
}

static int HSLServoLegacyRejected(void)
{
    g_hsl_servo_debug.legacy_reject_count++;
    g_hsl_servo_debug.last_result = HSL_SERVO_RESULT_UNSUPPORTED;
    hsl_legacy_last_error = HSL_SERVO_LEGACY_ERROR_UNSUPPORTED;
    return HSL_SERVO_LEGACY_ERROR_UNSUPPORTED;
}

void setEnd(uint8_t end)
{
    (void)end;
    HSLServoLegacyRejected();
}

void setLevel(uint8_t level)
{
    (void)level;
    HSLServoLegacyRejected();
}

int getLastError(void)
{
    return hsl_legacy_last_error;
}

int genWrite(uint8_t id, uint8_t address, uint8_t *data, uint8_t length)
{
    (void)id;
    (void)address;
    (void)data;
    (void)length;
    return HSLServoLegacyRejected();
}

int writeWord(uint8_t id, uint8_t address, uint16_t value)
{
    (void)id;
    (void)address;
    (void)value;
    return HSLServoLegacyRejected();
}

int WritePosEx2(uint8_t id, int16_t position, uint16_t speed,
                uint8_t acceleration, uint16_t torque)
{
    (void)id;
    (void)position;
    (void)speed;
    (void)acceleration;
    (void)torque;
    return HSLServoLegacyRejected();
}

void SyncWritePosEx2(uint8_t id[], uint8_t id_count, int16_t position[],
                     uint16_t speed[], uint8_t acceleration[],
                     uint16_t torque[])
{
    (void)id;
    (void)id_count;
    (void)position;
    (void)speed;
    (void)acceleration;
    (void)torque;
    HSLServoLegacyRejected();
}

int Read(uint8_t id, uint8_t address, uint8_t *data, uint8_t length)
{
    (void)id;
    (void)address;
    (void)data;
    (void)length;
    return HSLServoLegacyRejected();
}

int readByte(uint8_t id, uint8_t address)
{
    (void)id;
    (void)address;
    return HSLServoLegacyRejected();
}

int readWord(uint8_t id, uint8_t address)
{
    (void)id;
    (void)address;
    return HSLServoLegacyRejected();
}

int Ping(uint8_t id)
{
    (void)id;
    return HSLServoLegacyRejected();
}

void rFlushSCS(void)
{
    HSLServoLegacyRejected();
}

void wFlushSCS(void)
{
    HSLServoLegacyRejected();
}
