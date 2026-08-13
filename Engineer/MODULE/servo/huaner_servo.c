/**
 * @file huaner_servo.c
 * @brief 幻儿舵机控制板 USART6 DMA 驱动，负责动作、位置和电压事务。
 */

#include "huaner_servo.h"

#include "bsp_usart.h"
#include "usart.h"

#include <math.h>
#include <string.h>

#define HUANER_SERVO_HEADER                         0x55u
#define HUANER_SERVO_TX_BUFFER_LENGTH                 32u
#define HUANER_SERVO_RX_BUFFER_LENGTH                 32u
#define HUANER_SERVO_STATUS_SLOT_COUNT                 8u
#define HUANER_SERVO_TX_TIMEOUT_MS                    20u
#define HUANER_SERVO_RESPONSE_TIMEOUT_MS              30u
#define HUANER_SERVO_BOARD_TX_GAP_MS                   2u

typedef enum {
    HUANER_TRANSACTION_NONE = 0,
    HUANER_TRANSACTION_MOVE,
    HUANER_TRANSACTION_UNLOAD,
    HUANER_TRANSACTION_POSITION_READ,
    HUANER_TRANSACTION_BOARD_VOLTAGE_READ
} HuanerServo_Transaction_e;

typedef struct {
    uint8_t initialized;
    volatile uint8_t tx_complete;
    volatile uint8_t rx_complete;
    volatile uint8_t uart_error;
    volatile uint16_t rx_size;
    uint8_t rx_armed;
    HuanerServo_Transaction_e transaction;
    uint8_t ids[HUANER_SERVO_MAX_FEEDBACK_IDS];
    uint16_t target_positions[HUANER_SERVO_MAX_FEEDBACK_IDS];
    uint8_t id_count;
    uint16_t command_time_ms;
    uint8_t tx_buffer[HUANER_SERVO_TX_BUFFER_LENGTH];
    uint8_t rx_buffer[HUANER_SERVO_RX_BUFFER_LENGTH];
    uint8_t tx_length;
    uint8_t expected_rx_length;
    uint32_t state_tick;
    uint8_t poll_enabled;
    uint8_t poll_ids[HUANER_SERVO_MAX_FEEDBACK_IDS];
    uint8_t poll_id_count;
    uint8_t poll_next_index;
    uint16_t poll_period_ms;
    uint32_t next_poll_tick;
    uint8_t board_voltage_poll_enabled;
    uint16_t board_voltage_poll_period_ms;
    uint32_t next_board_voltage_poll_tick;
    uint8_t board_voltage_valid;
    uint16_t board_voltage_mv;
    uint32_t last_board_voltage_tick;
    HuanerServo_Status_s status[HUANER_SERVO_STATUS_SLOT_COUNT];
} HuanerServo_Runtime_s;

HuanerServo_Debug_s g_huaner_servo_driver_debug;

static HuanerServo_Runtime_s huaner_runtime;

static uint8_t HuanerServoIdValid(uint8_t id)
{
    return id >= HUANER_SERVO_MIN_ID && id <= HUANER_SERVO_MAX_ID;
}

static uint8_t HuanerServoPositionValid(uint16_t position)
{
    return position <= HUANER_SERVO_MAX_POSITION;
}

static uint8_t HuanerServoUartConfigValid(void)
{
    return huart6.Init.BaudRate == HUANER_SERVO_CONTROLLER_BAUD_RATE &&
           huart6.Init.WordLength == UART_WORDLENGTH_8B &&
           huart6.Init.StopBits == UART_STOPBITS_1 &&
           huart6.Init.Parity == UART_PARITY_NONE &&
           huart6.Init.Mode == UART_MODE_TX_RX &&
           huart6.Init.HwFlowCtl == UART_HWCONTROL_NONE;
}

static HuanerServo_Status_s *HuanerServoFindStatus(uint8_t id, uint8_t create)
{
    HuanerServo_Status_s *empty = NULL;
    uint8_t index;

    for (index = 0u; index < HUANER_SERVO_STATUS_SLOT_COUNT; ++index) {
        HuanerServo_Status_s *status = &huaner_runtime.status[index];

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
    empty->last_result = HUANER_SERVO_RESULT_NOT_FOUND;
    return empty;
}

static uint8_t HuanerServoIdInList(const uint8_t *ids,
                                uint8_t count,
                                uint8_t id)
{
    uint8_t index;

    if (ids == NULL) {
        return 0u;
    }
    for (index = 0u; index < count; ++index) {
        if (ids[index] == id) {
            return 1u;
        }
    }
    return 0u;
}

static void HuanerServoCopyDebugFrames(void)
{
    g_huaner_servo_driver_debug.tx_length = huaner_runtime.tx_length;
    g_huaner_servo_driver_debug.rx_length = (uint8_t)huaner_runtime.rx_size;
    memcpy(g_huaner_servo_driver_debug.tx_frame, huaner_runtime.tx_buffer,
           sizeof(g_huaner_servo_driver_debug.tx_frame));
    memcpy(g_huaner_servo_driver_debug.rx_frame, huaner_runtime.rx_buffer,
           sizeof(g_huaner_servo_driver_debug.rx_frame));
}

static void HuanerServoCopyCurrentIds(void)
{
    uint8_t index;

    g_huaner_servo_driver_debug.current_id_count = huaner_runtime.id_count;
    memset(g_huaner_servo_driver_debug.current_ids, 0,
           sizeof(g_huaner_servo_driver_debug.current_ids));
    for (index = 0u; index < huaner_runtime.id_count; ++index) {
        g_huaner_servo_driver_debug.current_ids[index] = huaner_runtime.ids[index];
    }
    g_huaner_servo_driver_debug.current_id = huaner_runtime.id_count != 0u ?
        huaner_runtime.ids[0] : 0u;
}

static void HuanerServoSetTransactionResult(HuanerServo_Result_e result,
                                         uint32_t now_ms)
{
    uint8_t index;

    for (index = 0u; index < huaner_runtime.id_count; ++index) {
        HuanerServo_Status_s *status =
            HuanerServoFindStatus(huaner_runtime.ids[index], 0u);

        if (status == NULL) {
            continue;
        }
        status->last_command = g_huaner_servo_driver_debug.current_command;
        status->last_result = result;
        if (result == HUANER_SERVO_RESULT_TIMEOUT) {
            status->timeout_count++;
        } else if (result != HUANER_SERVO_RESULT_OK &&
                   g_huaner_servo_driver_debug.current_command ==
                       HUANER_SERVO_COMMAND_POSITION_READ) {
            status->frame_fail_count++;
        }
        if (status->last_feedback_tick != 0u) {
            g_huaner_servo_driver_debug.last_response_tick =
                status->last_feedback_tick;
        }
    }
    (void)now_ms;
}

static void HuanerServoFinish(HuanerServo_Result_e result, uint32_t now_ms)
{
    if (huaner_runtime.rx_armed != 0u) {
        HAL_UART_AbortReceive(&huart6);
        huaner_runtime.rx_armed = 0u;
    }

    HuanerServoSetTransactionResult(result, now_ms);
    g_huaner_servo_driver_debug.last_command = g_huaner_servo_driver_debug.current_command;
    g_huaner_servo_driver_debug.last_result = result;
    g_huaner_servo_driver_debug.busy = 0u;
    if (result == HUANER_SERVO_RESULT_OK) {
        g_huaner_servo_driver_debug.state = HUANER_SERVO_STATE_COMPLETE;
    } else if (result == HUANER_SERVO_RESULT_TIMEOUT) {
        g_huaner_servo_driver_debug.state = HUANER_SERVO_STATE_TIMEOUT;
    } else {
        g_huaner_servo_driver_debug.state = HUANER_SERVO_STATE_ERROR;
    }
    g_huaner_servo_driver_debug.current_command = HUANER_SERVO_COMMAND_NONE;
    huaner_runtime.transaction = HUANER_TRANSACTION_NONE;
    huaner_runtime.rx_complete = 0u;
    huaner_runtime.tx_complete = 0u;
    huaner_runtime.state_tick = now_ms;
    HuanerServoCopyDebugFrames();
}

static void HuanerServoCountTx(void)
{
    uint8_t index;

    g_huaner_servo_driver_debug.tx_count++;
    for (index = 0u; index < huaner_runtime.id_count; ++index) {
        HuanerServo_Status_s *status =
            HuanerServoFindStatus(huaner_runtime.ids[index], 0u);
        if (status != NULL) {
            status->tx_count++;
        }
    }
}

static void HuanerServoCountTimeout(void)
{
    g_huaner_servo_driver_debug.timeout_count++;
}

static void HuanerServoRecoverUart(void)
{
    HAL_UART_AbortReceive(&huart6);
    HAL_UART_AbortTransmit(&huart6);
    huaner_runtime.rx_armed = 0u;
    huaner_runtime.rx_complete = 0u;
    huaner_runtime.tx_complete = 0u;
    huaner_runtime.uart_error = 0u;
    g_huaner_servo_driver_debug.recovery_count++;
}

static void HuanerServoSetMoveTarget(uint8_t id,
                                  uint16_t position,
                                  uint16_t time_ms,
                                  uint32_t now_ms);

static HuanerServo_Result_e HuanerServoBeginTransaction(
    HuanerServo_Transaction_e transaction,
    HuanerServo_Command_e command,
    const uint8_t *ids,
    uint8_t count,
    uint16_t command_time_ms,
    const uint8_t *tx_frame,
    uint8_t tx_length,
    const uint16_t *target_positions)
{
    uint32_t primask;
    uint32_t now_ms;
    uint8_t index;

    if (huaner_runtime.initialized == 0u) {
        return HUANER_SERVO_RESULT_NOT_INITIALIZED;
    }
    if (transaction == HUANER_TRANSACTION_NONE ||
        count > HUANER_SERVO_MAX_FEEDBACK_IDS || tx_frame == NULL ||
        tx_length == 0u || tx_length > HUANER_SERVO_TX_BUFFER_LENGTH ||
        (transaction == HUANER_TRANSACTION_MOVE &&
         target_positions == NULL) ||
        (transaction == HUANER_TRANSACTION_BOARD_VOLTAGE_READ &&
         (ids != NULL || count != 0u)) ||
        (transaction != HUANER_TRANSACTION_BOARD_VOLTAGE_READ &&
         (ids == NULL || count == 0u))) {
        return HUANER_SERVO_RESULT_INVALID;
    }
    for (index = 0u; index < count; ++index) {
        if (!HuanerServoIdValid(ids[index]) ||
            (index != 0u && ids[index] == ids[0])) {
            return HUANER_SERVO_RESULT_INVALID;
        }
    }

    now_ms = HAL_GetTick();
    primask = __get_PRIMASK();
    __disable_irq();
    if (g_huaner_servo_driver_debug.busy != 0u) {
        if (primask == 0u) {
            __enable_irq();
        }
        return HUANER_SERVO_RESULT_BUSY;
    }
    for (index = 0u; index < count; ++index) {
        if (HuanerServoFindStatus(ids[index], 1u) == NULL) {
            if (primask == 0u) {
                __enable_irq();
            }
            return HUANER_SERVO_RESULT_NOT_FOUND;
        }
    }

    huaner_runtime.transaction = transaction;
    huaner_runtime.id_count = count;
    huaner_runtime.command_time_ms = command_time_ms;
    huaner_runtime.tx_complete = 0u;
    huaner_runtime.rx_complete = 0u;
    huaner_runtime.uart_error = 0u;
    huaner_runtime.rx_size = 0u;
    huaner_runtime.rx_armed = 0u;
    if (transaction == HUANER_TRANSACTION_POSITION_READ) {
        huaner_runtime.expected_rx_length = (uint8_t)(5u + 3u * count);
    } else if (transaction == HUANER_TRANSACTION_BOARD_VOLTAGE_READ) {
        huaner_runtime.expected_rx_length = 6u;
    } else {
        huaner_runtime.expected_rx_length = 0u;
    }
    huaner_runtime.state_tick = now_ms;
    memset(huaner_runtime.ids, 0, sizeof(huaner_runtime.ids));
    memset(huaner_runtime.target_positions, 0,
           sizeof(huaner_runtime.target_positions));
    memset(huaner_runtime.tx_buffer, 0, sizeof(huaner_runtime.tx_buffer));
    memset(huaner_runtime.rx_buffer, 0, sizeof(huaner_runtime.rx_buffer));
    memcpy(huaner_runtime.tx_buffer, tx_frame, tx_length);
    huaner_runtime.tx_length = tx_length;
    for (index = 0u; index < count; ++index) {
        huaner_runtime.ids[index] = ids[index];
        if (target_positions != NULL) {
            huaner_runtime.target_positions[index] = target_positions[index];
            HuanerServoSetMoveTarget(ids[index], target_positions[index],
                                  command_time_ms, now_ms);
        }
    }

    g_huaner_servo_driver_debug.current_command = command;
    g_huaner_servo_driver_debug.expected_rx_length =
        huaner_runtime.expected_rx_length;
    g_huaner_servo_driver_debug.last_result = HUANER_SERVO_RESULT_BUSY;
    g_huaner_servo_driver_debug.busy = 1u;
    g_huaner_servo_driver_debug.transaction_start_tick = huaner_runtime.state_tick;
    g_huaner_servo_driver_debug.state =
        (transaction == HUANER_TRANSACTION_MOVE ||
         transaction == HUANER_TRANSACTION_UNLOAD) ?
        HUANER_SERVO_STATE_TX_DMA : HUANER_SERVO_STATE_PREPARE_RX;
    HuanerServoCopyCurrentIds();
    HuanerServoCopyDebugFrames();

    if (primask == 0u) {
        __enable_irq();
    }
    return HUANER_SERVO_RESULT_OK;
}

static void HuanerServoSetMoveTarget(uint8_t id,
                                  uint16_t position,
                                  uint16_t time_ms,
                                  uint32_t now_ms)
{
    HuanerServo_Status_s *status = HuanerServoFindStatus(id, 1u);

    if (status == NULL) {
        return;
    }
    status->target_valid = 1u;
    status->target_position = position;
    status->position_error = status->feedback_valid != 0u ?
        (int16_t)((int32_t)position -
                  (int32_t)status->feedback_position) : 0;
    status->command_time_ms = time_ms;
    status->last_command_tick = now_ms;
    status->arrived = 0u;
    status->motion_timeout = 0u;
    status->arrival_stable_since_tick = 0u;
}

static HAL_StatusTypeDef HuanerServoStartReceive(uint32_t now_ms)
{
    HAL_StatusTypeDef result;

    if (huart6.hdmarx == NULL) {
        return HAL_ERROR;
    }
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    memset(huaner_runtime.rx_buffer, 0, sizeof(huaner_runtime.rx_buffer));
    huaner_runtime.rx_size = 0u;
    huaner_runtime.rx_complete = 0u;
    result = HAL_UARTEx_ReceiveToIdle_DMA(
        &huart6, huaner_runtime.rx_buffer, HUANER_SERVO_RX_BUFFER_LENGTH);
    if (result == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart6.hdmarx, DMA_IT_HT);
        huaner_runtime.rx_armed = 1u;
        huaner_runtime.state_tick = now_ms;
    }
    return result;
}

static void HuanerServoUpdateArrival(HuanerServo_Status_s *status,
                                  uint32_t now_ms)
{
    int32_t absolute_error;

    if (status == NULL || status->target_valid == 0u ||
        status->feedback_valid == 0u) {
        return;
    }
    absolute_error = status->position_error;
    if (absolute_error < 0) {
        absolute_error = -absolute_error;
    }
    if ((uint32_t)absolute_error <= HUANER_SERVO_ARRIVAL_TOLERANCE_POS &&
        fabsf(status->feedback_velocity_pos_s) <=
            HUANER_SERVO_ARRIVAL_VELOCITY_POS_S) {
        if (status->arrival_stable_since_tick == 0u) {
            status->arrival_stable_since_tick = now_ms;
        }
        if ((uint32_t)(now_ms - status->arrival_stable_since_tick) >=
            HUANER_SERVO_ARRIVAL_STABLE_MS) {
            status->arrived = 1u;
            status->motion_timeout = 0u;
        }
    } else {
        status->arrival_stable_since_tick = 0u;
        status->arrived = 0u;
    }
}

static void HuanerServoUpdateFeedback(uint8_t id,
                                   uint16_t position,
                                   uint32_t now_ms)
{
    HuanerServo_Status_s *status = HuanerServoFindStatus(id, 0u);
    uint32_t delta_ms;

    if (status == NULL) {
        return;
    }
    if (status->feedback_valid != 0u && status->last_feedback_tick != 0u) {
        delta_ms = (uint32_t)(now_ms - status->last_feedback_tick);
        if (delta_ms != 0u) {
            status->feedback_velocity_pos_s =
                ((float)position - (float)status->feedback_position) *
                1000.0f / (float)delta_ms;
        }
    } else {
        status->feedback_velocity_pos_s = 0.0f;
    }
    status->feedback_position = position;
    status->feedback_valid = 1u;
    status->online = 1u;
    status->last_feedback_tick = now_ms;
    status->feedback_sequence++;
    status->rx_count++;
    status->last_command = HUANER_SERVO_COMMAND_POSITION_READ;
    status->last_result = HUANER_SERVO_RESULT_OK;
    if (status->target_valid != 0u) {
        status->position_error =
            (int16_t)((int32_t)status->target_position -
                      (int32_t)position);
    }
    HuanerServoUpdateArrival(status, now_ms);

    g_huaner_servo_driver_debug.last_position = position;
    g_huaner_servo_driver_debug.last_position_valid = 1u;
    g_huaner_servo_driver_debug.last_response_tick = now_ms;
    g_huaner_servo_driver_debug.online = 1u;
}

/*
 * 控制板CMD_SERVO_MOVE(0x03)：
 * 55 55 | Length=count*3+5 | 03 | count | time_l time_h |
 * (id position_l position_h)*count。
 */
static uint8_t HuanerServoBuildMoveFrame(const uint8_t *ids,
                                      const uint16_t *positions,
                                      uint8_t count,
                                      uint16_t time_ms,
                                      uint8_t *frame,
                                      uint8_t capacity)
{
    uint8_t index;
    uint8_t total_length;

    if (ids == NULL || positions == NULL || frame == NULL || count == 0u ||
        count > HUANER_SERVO_MAX_FEEDBACK_IDS ||
        time_ms > HUANER_SERVO_MAX_TIME_MS) {
        return 0u;
    }
    total_length = (uint8_t)(7u + 3u * count);
    if (capacity < total_length) {
        return 0u;
    }
    for (index = 0u; index < count; ++index) {
        if (!HuanerServoIdValid(ids[index]) ||
            !HuanerServoPositionValid(positions[index]) ||
            (index != 0u && HuanerServoIdInList(ids, index, ids[index]))) {
            return 0u;
        }
    }

    memset(frame, 0, capacity);
    frame[0] = HUANER_SERVO_HEADER;
    frame[1] = HUANER_SERVO_HEADER;
    frame[2] = (uint8_t)(5u + 3u * count);
    frame[3] = (uint8_t)HUANER_SERVO_COMMAND_MOVE;
    frame[4] = count;
    frame[5] = (uint8_t)(time_ms & 0xffu);
    frame[6] = (uint8_t)(time_ms >> 8u);
    for (index = 0u; index < count; ++index) {
        uint8_t offset = (uint8_t)(7u + 3u * index);

        frame[offset] = ids[index];
        frame[offset + 1u] = (uint8_t)(positions[index] & 0xffu);
        frame[offset + 2u] = (uint8_t)(positions[index] >> 8u);
    }
    return total_length;
}

/* CMD_MULT_SERVO_POS_READ(0x15)，请求参数为count和各舵机ID。 */
static uint8_t HuanerServoBuildPositionRequestFrame(const uint8_t *ids,
                                                  uint8_t count,
                                                  uint8_t *frame,
                                                  uint8_t capacity)
{
    uint8_t index;
    uint8_t total_length;

    if (ids == NULL || frame == NULL || count == 0u ||
        count > HUANER_SERVO_MAX_FEEDBACK_IDS) {
        return 0u;
    }
    total_length = (uint8_t)(5u + count);
    if (capacity < total_length) {
        return 0u;
    }
    for (index = 0u; index < count; ++index) {
        if (!HuanerServoIdValid(ids[index]) ||
            (index != 0u && HuanerServoIdInList(ids, index, ids[index]))) {
            return 0u;
        }
    }
    memset(frame, 0, capacity);
    frame[0] = HUANER_SERVO_HEADER;
    frame[1] = HUANER_SERVO_HEADER;
    frame[2] = (uint8_t)(3u + count);
    frame[3] = (uint8_t)HUANER_SERVO_COMMAND_POSITION_READ;
    frame[4] = count;
    for (index = 0u; index < count; ++index) {
        frame[5u + index] = ids[index];
    }
    return total_length;
}

/*
 * 控制板CMD_MULT_SERVO_UNLOAD(0x14)：
 * 55 55 | Length=count+3 | 14 | count | ID1 ... IDN。
 * 此命令只解除舵机力矩且没有应答，不影响后续0x15位置查询。
 */
static uint8_t HuanerServoBuildUnloadFrame(const uint8_t *ids,
                                           uint8_t count,
                                           uint8_t *frame,
                                           uint8_t capacity)
{
    uint8_t index;
    uint8_t total_length;

    if (ids == NULL || frame == NULL || count == 0u ||
        count > HUANER_SERVO_MAX_FEEDBACK_IDS) {
        return 0u;
    }
    total_length = (uint8_t)(5u + count);
    if (capacity < total_length) {
        return 0u;
    }
    for (index = 0u; index < count; ++index) {
        if (!HuanerServoIdValid(ids[index]) ||
            (index != 0u && HuanerServoIdInList(ids, index, ids[index]))) {
            return 0u;
        }
    }

    memset(frame, 0, capacity);
    frame[0] = HUANER_SERVO_HEADER;
    frame[1] = HUANER_SERVO_HEADER;
    frame[2] = (uint8_t)(3u + count);
    frame[3] = (uint8_t)HUANER_SERVO_COMMAND_UNLOAD;
    frame[4] = count;
    for (index = 0u; index < count; ++index) {
        frame[5u + index] = ids[index];
    }
    return total_length;
}

static HuanerServo_Result_e HuanerServoParsePositionReply(
    const uint8_t *rx_buffer,
    uint16_t rx_size,
    const uint8_t *requested_ids,
    uint8_t requested_count,
    uint8_t *response_ids,
    uint16_t *positions)
{
    uint8_t found[HUANER_SERVO_MAX_FEEDBACK_IDS] = {0u};
    HuanerServo_Result_e candidate = HUANER_SERVO_RESULT_FRAME_ERROR;
    uint16_t start;

    if (rx_buffer == NULL || requested_ids == NULL ||
        response_ids == NULL || positions == NULL ||
        requested_count == 0u ||
        requested_count > HUANER_SERVO_MAX_FEEDBACK_IDS) {
        return HUANER_SERVO_RESULT_INVALID;
    }
    for (start = 0u; start + 5u <= rx_size; ++start) {
        const uint8_t *frame = &rx_buffer[start];
        uint8_t length_field;
        uint8_t count;
        uint16_t total_length;
        uint8_t index;

        if (frame[0] != HUANER_SERVO_HEADER ||
            frame[1] != HUANER_SERVO_HEADER) {
            continue;
        }
        length_field = frame[2];
        total_length = (uint16_t)length_field + 2u;
        if (total_length < 8u ||
            start + total_length > rx_size) {
            continue;
        }
        if (frame[3] != (uint8_t)HUANER_SERVO_COMMAND_POSITION_READ) {
            candidate = HUANER_SERVO_RESULT_COMMAND_MISMATCH;
            continue;
        }
        count = frame[4];
        if (count == 0u || count > HUANER_SERVO_MAX_FEEDBACK_IDS ||
            length_field != (uint8_t)(3u + 3u * count) ||
            count != requested_count) {
            candidate = HUANER_SERVO_RESULT_FRAME_ERROR;
            continue;
        }

        memset(found, 0, sizeof(found));
        for (index = 0u; index < count; ++index) {
            uint8_t offset = (uint8_t)(5u + 3u * index);
            uint8_t requested_index;
            uint8_t matched = 0u;

            response_ids[index] = frame[offset];
            positions[index] = (uint16_t)frame[offset + 1u] |
                ((uint16_t)frame[offset + 2u] << 8u);
            if (!HuanerServoIdInList(requested_ids, requested_count,
                                  response_ids[index])) {
                candidate = HUANER_SERVO_RESULT_ID_MISMATCH;
                break;
            }
            if (!HuanerServoPositionValid(positions[index])) {
                candidate = HUANER_SERVO_RESULT_POSITION_RANGE;
                break;
            }
            for (requested_index = 0u;
                 requested_index < requested_count;
                 ++requested_index) {
                if (requested_ids[requested_index] == response_ids[index]) {
                    if (found[requested_index] != 0u) {
                        candidate = HUANER_SERVO_RESULT_ID_MISMATCH;
                        matched = 2u;
                        break;
                    }
                    found[requested_index] = 1u;
                    matched = 1u;
                    break;
                }
            }
            if (matched != 1u) {
                if (matched == 0u) {
                    candidate = HUANER_SERVO_RESULT_ID_MISMATCH;
                }
                break;
            }
        }
        if (index != count) {
            continue;
        }
        for (index = 0u; index < requested_count; ++index) {
            if (found[index] == 0u) {
                candidate = HUANER_SERVO_RESULT_ID_MISMATCH;
                break;
            }
        }
        if (index != requested_count) {
            continue;
        }
        return HUANER_SERVO_RESULT_OK;
    }
    return candidate;
}

static uint8_t HuanerServoBuildBoardVoltageRequestFrame(uint8_t *frame,
                                                      uint8_t capacity)
{
    if (frame == NULL || capacity < 4u) {
        return 0u;
    }
    memset(frame, 0, capacity);
    frame[0] = HUANER_SERVO_HEADER;
    frame[1] = HUANER_SERVO_HEADER;
    frame[2] = 2u;
    frame[3] = (uint8_t)HUANER_SERVO_COMMAND_BOARD_VOLTAGE_READ;
    return 4u;
}

static HuanerServo_Result_e HuanerServoParseBoardVoltageReply(
    const uint8_t *rx_buffer,
    uint16_t rx_size,
    uint16_t *voltage_mv)
{
    HuanerServo_Result_e candidate = HUANER_SERVO_RESULT_FRAME_ERROR;
    uint16_t start;

    if (rx_buffer == NULL || voltage_mv == NULL) {
        return HUANER_SERVO_RESULT_INVALID;
    }
    for (start = 0u; start + 4u <= rx_size; ++start) {
        const uint8_t *frame = &rx_buffer[start];
        uint8_t length_field;
        uint16_t total_length;
        uint16_t candidate_voltage_mv;

        if (frame[0] != HUANER_SERVO_HEADER ||
            frame[1] != HUANER_SERVO_HEADER) {
            continue;
        }
        length_field = frame[2];
        total_length = (uint16_t)length_field + 2u;
        if (total_length < 4u || start + total_length > rx_size) {
            continue;
        }
        if (frame[3] !=
            (uint8_t)HUANER_SERVO_COMMAND_BOARD_VOLTAGE_READ) {
            candidate = HUANER_SERVO_RESULT_COMMAND_MISMATCH;
            continue;
        }
        if (length_field != 4u || total_length != 6u) {
            candidate = HUANER_SERVO_RESULT_FRAME_ERROR;
            continue;
        }
        candidate_voltage_mv = (uint16_t)frame[4] |
            ((uint16_t)frame[5] << 8u);
        if (candidate_voltage_mv < HUANER_SERVO_BOARD_VOLTAGE_MIN_MV ||
            candidate_voltage_mv > HUANER_SERVO_BOARD_VOLTAGE_MAX_MV) {
            candidate = HUANER_SERVO_RESULT_VOLTAGE_RANGE;
            continue;
        }
        *voltage_mv = candidate_voltage_mv;
        return HUANER_SERVO_RESULT_OK;
    }
    return candidate;
}

static uint8_t HuanerServoProtocolSelfTest(void)
{
    static const uint8_t expected_move1[10] = {
        0x55u, 0x55u, 0x08u, 0x03u, 0x01u,
        0xe8u, 0x03u, 0x01u, 0x20u, 0x03u
    };
    static const uint8_t expected_move2[13] = {
        0x55u, 0x55u, 0x0bu, 0x03u, 0x02u,
        0x20u, 0x03u, 0x02u, 0x20u, 0x03u,
        0x09u, 0x20u, 0x03u
    };
    static const uint8_t expected_request[7] = {
        0x55u, 0x55u, 0x05u, 0x15u, 0x02u, 0x01u, 0x02u
    };
    static const uint8_t expected_unload[7] = {
        0x55u, 0x55u, 0x05u, 0x14u, 0x02u, 0x01u, 0x02u
    };
    static const uint8_t valid_reply[11] = {
        0x55u, 0x55u, 0x09u, 0x15u, 0x02u,
        0x01u, 0xf4u, 0x01u, 0x02u, 0x58u, 0x02u
    };
    static const uint8_t expected_voltage_request[4] = {
        0x55u, 0x55u, 0x02u, 0x0fu
    };
    static const uint8_t valid_voltage_reply[6] = {
        0x55u, 0x55u, 0x04u, 0x0fu, 0xe8u, 0x1cu
    };
    uint8_t move_frame[13];
    uint8_t request[7];
    uint8_t unload[7];
    uint8_t test_reply[sizeof(valid_reply)];
    uint8_t voltage_request[sizeof(expected_voltage_request)];
    uint8_t voltage_test_reply[sizeof(valid_voltage_reply)];
    uint8_t requested_ids[2] = {1u, 2u};
    uint8_t move1_ids[1] = {1u};
    uint8_t move2_ids[2] = {2u, 9u};
    uint16_t move1_positions[1] = {800u};
    uint16_t move2_positions[2] = {800u, 800u};
    uint8_t response_ids[2];
    uint16_t positions[2];
    uint16_t voltage_mv;

    if (HuanerServoBuildMoveFrame(
            move1_ids, move1_positions, 1u, 1000u,
            move_frame, (uint8_t)sizeof(move_frame)) !=
            sizeof(expected_move1) ||
        memcmp(move_frame, expected_move1, sizeof(expected_move1)) != 0) {
        return 0u;
    }
    if (HuanerServoBuildMoveFrame(
            move2_ids, move2_positions, 2u, 800u,
            move_frame, (uint8_t)sizeof(move_frame)) !=
            sizeof(expected_move2) ||
        memcmp(move_frame, expected_move2, sizeof(expected_move2)) != 0) {
        return 0u;
    }

    if (HuanerServoBuildPositionRequestFrame(
            requested_ids, 2u, request, (uint8_t)sizeof(request)) !=
            sizeof(expected_request) ||
        memcmp(request, expected_request, sizeof(expected_request)) != 0) {
        return 0u;
    }
    if (HuanerServoBuildUnloadFrame(
            requested_ids, 2u, unload, (uint8_t)sizeof(unload)) !=
            sizeof(expected_unload) ||
        memcmp(unload, expected_unload, sizeof(expected_unload)) != 0) {
        return 0u;
    }
    if (HuanerServoParsePositionReply(
            valid_reply, (uint16_t)sizeof(valid_reply), requested_ids, 2u,
            response_ids, positions) != HUANER_SERVO_RESULT_OK ||
        response_ids[0] != 1u || positions[0] != 500u ||
        response_ids[1] != 2u || positions[1] != 600u) {
        return 0u;
    }

    memcpy(test_reply, valid_reply, sizeof(test_reply));
    test_reply[3] = 0x14u;
    if (HuanerServoParsePositionReply(
            test_reply, (uint16_t)sizeof(test_reply), requested_ids, 2u,
            response_ids, positions) !=
            HUANER_SERVO_RESULT_COMMAND_MISMATCH) {
        return 0u;
    }
    memcpy(test_reply, valid_reply, sizeof(test_reply));
    test_reply[8] = 1u;
    if (HuanerServoParsePositionReply(
            test_reply, (uint16_t)sizeof(test_reply), requested_ids, 2u,
            response_ids, positions) != HUANER_SERVO_RESULT_ID_MISMATCH) {
        return 0u;
    }
    memcpy(test_reply, valid_reply, sizeof(test_reply));
    test_reply[2] = 6u;
    test_reply[4] = 1u;
    if (HuanerServoParsePositionReply(
            test_reply, 8u, requested_ids, 2u,
            response_ids, positions) != HUANER_SERVO_RESULT_FRAME_ERROR) {
        return 0u;
    }
    if (HuanerServoParsePositionReply(
            valid_reply, (uint16_t)(sizeof(valid_reply) - 1u),
            requested_ids, 2u, response_ids, positions) !=
            HUANER_SERVO_RESULT_FRAME_ERROR) {
        return 0u;
    }
    memcpy(test_reply, valid_reply, sizeof(test_reply));
    test_reply[6] = 0xe9u;
    test_reply[7] = 0x03u;
    if (HuanerServoParsePositionReply(
            test_reply, (uint16_t)sizeof(test_reply), requested_ids, 2u,
            response_ids, positions) != HUANER_SERVO_RESULT_POSITION_RANGE) {
        return 0u;
    }
    if (HuanerServoBuildBoardVoltageRequestFrame(
            voltage_request, (uint8_t)sizeof(voltage_request)) !=
            sizeof(expected_voltage_request) ||
        memcmp(voltage_request, expected_voltage_request,
               sizeof(expected_voltage_request)) != 0) {
        return 0u;
    }
    if (HuanerServoParseBoardVoltageReply(
            valid_voltage_reply, (uint16_t)sizeof(valid_voltage_reply),
            &voltage_mv) != HUANER_SERVO_RESULT_OK || voltage_mv != 7400u) {
        return 0u;
    }
    memcpy(voltage_test_reply, valid_voltage_reply,
           sizeof(voltage_test_reply));
    voltage_test_reply[3] = 0x15u;
    if (HuanerServoParseBoardVoltageReply(
            voltage_test_reply, (uint16_t)sizeof(voltage_test_reply),
            &voltage_mv) != HUANER_SERVO_RESULT_COMMAND_MISMATCH) {
        return 0u;
    }
    memcpy(voltage_test_reply, valid_voltage_reply,
           sizeof(voltage_test_reply));
    voltage_test_reply[2] = 3u;
    if (HuanerServoParseBoardVoltageReply(
            voltage_test_reply, (uint16_t)sizeof(voltage_test_reply),
            &voltage_mv) != HUANER_SERVO_RESULT_FRAME_ERROR) {
        return 0u;
    }
    if (HuanerServoParseBoardVoltageReply(
            valid_voltage_reply,
            (uint16_t)(sizeof(valid_voltage_reply) - 1u),
            &voltage_mv) != HUANER_SERVO_RESULT_FRAME_ERROR) {
        return 0u;
    }
    memcpy(voltage_test_reply, valid_voltage_reply,
           sizeof(voltage_test_reply));
    voltage_test_reply[4] = 0u;
    voltage_test_reply[5] = 0u;
    if (HuanerServoParseBoardVoltageReply(
            voltage_test_reply, (uint16_t)sizeof(voltage_test_reply),
            &voltage_mv) != HUANER_SERVO_RESULT_VOLTAGE_RANGE) {
        return 0u;
    }
    return 1u;
}

static HuanerServo_Result_e HuanerServoValidatePositionReply(uint32_t now_ms)
{
    uint16_t positions[HUANER_SERVO_MAX_FEEDBACK_IDS];
    uint8_t response_ids[HUANER_SERVO_MAX_FEEDBACK_IDS];
    HuanerServo_Result_e result;
    uint8_t index;

    result = HuanerServoParsePositionReply(
        huaner_runtime.rx_buffer, huaner_runtime.rx_size,
        huaner_runtime.ids, huaner_runtime.id_count,
        response_ids, positions);
    if (result != HUANER_SERVO_RESULT_OK) {
        return result;
    }
    for (index = 0u; index < huaner_runtime.id_count; ++index) {
        HuanerServoUpdateFeedback(response_ids[index], positions[index], now_ms);
    }
    g_huaner_servo_driver_debug.rx_count++;
    return HUANER_SERVO_RESULT_OK;
}

static HuanerServo_Result_e HuanerServoValidateBoardVoltageReply(uint32_t now_ms)
{
    HuanerServo_Result_e result;
    uint16_t voltage_mv;

    result = HuanerServoParseBoardVoltageReply(
        huaner_runtime.rx_buffer, huaner_runtime.rx_size, &voltage_mv);
    if (result != HUANER_SERVO_RESULT_OK) {
        return result;
    }
    huaner_runtime.board_voltage_mv = voltage_mv;
    huaner_runtime.board_voltage_valid = 1u;
    huaner_runtime.last_board_voltage_tick = now_ms;
    g_huaner_servo_driver_debug.board_voltage_mv = voltage_mv;
    g_huaner_servo_driver_debug.board_voltage_valid = 1u;
    g_huaner_servo_driver_debug.last_board_voltage_tick = now_ms;
    g_huaner_servo_driver_debug.last_response_tick = now_ms;
    g_huaner_servo_driver_debug.rx_count++;
    return HUANER_SERVO_RESULT_OK;
}

static HuanerServo_Result_e HuanerServoValidateReply(uint32_t now_ms)
{
    if (huaner_runtime.transaction == HUANER_TRANSACTION_POSITION_READ) {
        return HuanerServoValidatePositionReply(now_ms);
    }
    if (huaner_runtime.transaction == HUANER_TRANSACTION_BOARD_VOLTAGE_READ) {
        return HuanerServoValidateBoardVoltageReply(now_ms);
    }
    return HUANER_SERVO_RESULT_INVALID;
}

static void HuanerServoRxEvent(uint16_t size)
{
    if (size > HUANER_SERVO_RX_BUFFER_LENGTH) {
        size = HUANER_SERVO_RX_BUFFER_LENGTH;
    }
    huaner_runtime.rx_size = size;
    huaner_runtime.rx_armed = 0u;
    huaner_runtime.rx_complete = 1u;
}

static void HuanerServoTxComplete(void)
{
    huaner_runtime.tx_complete = 1u;
}

static void HuanerServoUartError(void)
{
    huaner_runtime.uart_error = 1u;
}

uint8_t HuanerServoInit(void)
{
    USART_Async_Callback_Config_s callbacks;

    memset(&huaner_runtime, 0, sizeof(huaner_runtime));
    memset(&g_huaner_servo_driver_debug, 0, sizeof(g_huaner_servo_driver_debug));
    memset(&callbacks, 0, sizeof(callbacks));
    g_huaner_servo_driver_debug.uart_baud_rate = huart6.Init.BaudRate;
    g_huaner_servo_driver_debug.uart_config_valid = HuanerServoUartConfigValid();
    if (g_huaner_servo_driver_debug.uart_config_valid == 0u) {
        g_huaner_servo_driver_debug.state = HUANER_SERVO_STATE_ERROR;
        g_huaner_servo_driver_debug.last_result = HUANER_SERVO_RESULT_HAL_ERROR;
        return 0u;
    }
    g_huaner_servo_driver_debug.protocol_self_test_passed =
        HuanerServoProtocolSelfTest();
    if (g_huaner_servo_driver_debug.protocol_self_test_passed == 0u) {
        g_huaner_servo_driver_debug.state = HUANER_SERVO_STATE_ERROR;
        g_huaner_servo_driver_debug.last_result = HUANER_SERVO_RESULT_FRAME_ERROR;
        return 0u;
    }
    if (huart6.hdmarx == NULL || huart6.hdmatx == NULL) {
        g_huaner_servo_driver_debug.state = HUANER_SERVO_STATE_ERROR;
        g_huaner_servo_driver_debug.last_result = HUANER_SERVO_RESULT_HAL_ERROR;
        return 0u;
    }
    callbacks.usart_handle = &huart6;
    callbacks.rx_event_callback = HuanerServoRxEvent;
    callbacks.tx_complete_callback = HuanerServoTxComplete;
    callbacks.error_callback = HuanerServoUartError;
    if (USARTRegisterAsyncCallbacks(&callbacks) == 0u) {
        g_huaner_servo_driver_debug.state = HUANER_SERVO_STATE_ERROR;
        g_huaner_servo_driver_debug.last_result = HUANER_SERVO_RESULT_HAL_ERROR;
        return 0u;
    }
    huaner_runtime.initialized = 1u;
    g_huaner_servo_driver_debug.initialized = 1u;
    g_huaner_servo_driver_debug.state = HUANER_SERVO_STATE_IDLE;
    g_huaner_servo_driver_debug.last_result = HUANER_SERVO_RESULT_OK;
    return 1u;
}

HuanerServo_Result_e HuanerServoMove(uint8_t id,
                               uint16_t position,
                               uint16_t time_ms)
{
    uint8_t ids[1];
    uint8_t tx_frame[10] = {0u};
    uint8_t tx_length;
    uint16_t target_positions[1];

    if (!HuanerServoIdValid(id) || !HuanerServoPositionValid(position) ||
        time_ms > HUANER_SERVO_MAX_TIME_MS) {
        return HUANER_SERVO_RESULT_INVALID;
    }
    ids[0] = id;
    target_positions[0] = position;
    tx_length = HuanerServoBuildMoveFrame(
        ids, target_positions, 1u, time_ms,
        tx_frame, (uint8_t)sizeof(tx_frame));
    if (tx_length == 0u) {
        return HUANER_SERVO_RESULT_INVALID;
    }
    return HuanerServoBeginTransaction(
        HUANER_TRANSACTION_MOVE, HUANER_SERVO_COMMAND_MOVE, ids, 1u, time_ms,
        tx_frame, tx_length, target_positions);
}

HuanerServo_Result_e HuanerServoMove2(uint8_t id1,
                                uint16_t position1,
                                uint8_t id2,
                                uint16_t position2,
                                uint16_t time_ms)
{
    uint8_t ids[2];
    uint8_t tx_frame[13] = {0u};
    uint8_t tx_length;
    uint16_t target_positions[2];

    if (!HuanerServoIdValid(id1) || !HuanerServoIdValid(id2) || id1 == id2 ||
        !HuanerServoPositionValid(position1) ||
        !HuanerServoPositionValid(position2) ||
        time_ms > HUANER_SERVO_MAX_TIME_MS) {
        return HUANER_SERVO_RESULT_INVALID;
    }
    ids[0] = id1;
    ids[1] = id2;
    target_positions[0] = position1;
    target_positions[1] = position2;
    tx_length = HuanerServoBuildMoveFrame(
        ids, target_positions, 2u, time_ms,
        tx_frame, (uint8_t)sizeof(tx_frame));
    if (tx_length == 0u) {
        return HUANER_SERVO_RESULT_INVALID;
    }
    return HuanerServoBeginTransaction(
        HUANER_TRANSACTION_MOVE, HUANER_SERVO_COMMAND_MOVE, ids, 2u, time_ms,
        tx_frame, tx_length, target_positions);
}

HuanerServo_Result_e HuanerServoStop(uint8_t id)
{
    if (!HuanerServoIdValid(id)) {
        return HUANER_SERVO_RESULT_INVALID;
    }
    return HUANER_SERVO_RESULT_UNSUPPORTED;
}

HuanerServo_Result_e HuanerServoUnload(const uint8_t *ids, uint8_t count)
{
    uint8_t tx_frame[5u + HUANER_SERVO_MAX_FEEDBACK_IDS] = {0u};
    uint8_t tx_length;

    tx_length = HuanerServoBuildUnloadFrame(
        ids, count, tx_frame, (uint8_t)sizeof(tx_frame));
    if (tx_length == 0u) {
        return HUANER_SERVO_RESULT_INVALID;
    }
    return HuanerServoBeginTransaction(
        HUANER_TRANSACTION_UNLOAD, HUANER_SERVO_COMMAND_UNLOAD,
        ids, count, 0u, tx_frame, tx_length, NULL);
}

HuanerServo_Result_e HuanerServoRequestPositions(const uint8_t *ids,
                                            uint8_t count)
{
    uint8_t tx_frame[5u + HUANER_SERVO_MAX_FEEDBACK_IDS] = {0u};
    uint8_t tx_length;

    tx_length = HuanerServoBuildPositionRequestFrame(
        ids, count, tx_frame, (uint8_t)sizeof(tx_frame));
    if (tx_length == 0u) {
        return HUANER_SERVO_RESULT_INVALID;
    }
    return HuanerServoBeginTransaction(
        HUANER_TRANSACTION_POSITION_READ,
        HUANER_SERVO_COMMAND_POSITION_READ, ids, count, 0u,
        tx_frame, tx_length, NULL);
}

HuanerServo_Result_e HuanerServoRequestPosition(uint8_t id)
{
    return HuanerServoRequestPositions(&id, 1u);
}

HuanerServo_Result_e HuanerServoRequestBoardVoltage(void)
{
    uint8_t tx_frame[4] = {0u};
    uint8_t tx_length;

    tx_length = HuanerServoBuildBoardVoltageRequestFrame(
        tx_frame, (uint8_t)sizeof(tx_frame));
    if (tx_length == 0u) {
        return HUANER_SERVO_RESULT_INVALID;
    }
    return HuanerServoBeginTransaction(
        HUANER_TRANSACTION_BOARD_VOLTAGE_READ,
        HUANER_SERVO_COMMAND_BOARD_VOLTAGE_READ,
        NULL, 0u, 0u, tx_frame, tx_length, NULL);
}

uint8_t HuanerServoConfigureFeedbackPolling(const uint8_t *ids,
                                          uint8_t count,
                                          uint16_t period_ms)
{
    uint8_t index;

    if (huaner_runtime.initialized == 0u || ids == NULL || count == 0u ||
        count > HUANER_SERVO_MAX_FEEDBACK_IDS || period_ms == 0u) {
        return 0u;
    }
    for (index = 0u; index < count; ++index) {
        if (!HuanerServoIdValid(ids[index]) ||
            (index != 0u && ids[index] == ids[0]) ||
            HuanerServoFindStatus(ids[index], 1u) == NULL) {
            return 0u;
        }
    }
    memset(huaner_runtime.poll_ids, 0, sizeof(huaner_runtime.poll_ids));
    memset(g_huaner_servo_driver_debug.poll_ids, 0,
           sizeof(g_huaner_servo_driver_debug.poll_ids));
    for (index = 0u; index < count; ++index) {
        huaner_runtime.poll_ids[index] = ids[index];
        g_huaner_servo_driver_debug.poll_ids[index] = ids[index];
    }
    huaner_runtime.poll_id_count = count;
    huaner_runtime.poll_next_index = 0u;
    huaner_runtime.poll_period_ms = period_ms;
    huaner_runtime.next_poll_tick = HAL_GetTick();
    huaner_runtime.poll_enabled = 1u;
    g_huaner_servo_driver_debug.poll_enabled = 1u;
    g_huaner_servo_driver_debug.poll_id_count = count;
    g_huaner_servo_driver_debug.poll_period_ms = period_ms;
    return 1u;
}

void HuanerServoDisableFeedbackPolling(void)
{
    huaner_runtime.poll_enabled = 0u;
    huaner_runtime.poll_next_index = 0u;
    g_huaner_servo_driver_debug.poll_enabled = 0u;
}

uint8_t HuanerServoConfigureBoardVoltagePolling(uint16_t period_ms)
{
    if (huaner_runtime.initialized == 0u || period_ms == 0u) {
        return 0u;
    }
    huaner_runtime.board_voltage_poll_period_ms = period_ms;
    huaner_runtime.next_board_voltage_poll_tick = HAL_GetTick();
    huaner_runtime.board_voltage_poll_enabled = 1u;
    g_huaner_servo_driver_debug.board_voltage_poll_period_ms = period_ms;
    g_huaner_servo_driver_debug.board_voltage_poll_enabled = 1u;
    return 1u;
}

void HuanerServoDisableBoardVoltagePolling(void)
{
    huaner_runtime.board_voltage_poll_enabled = 0u;
    g_huaner_servo_driver_debug.board_voltage_poll_enabled = 0u;
}

uint8_t HuanerServoGetBoardVoltage(uint16_t *voltage_mv)
{
    uint32_t primask;
    uint8_t valid;

    if (voltage_mv == NULL) {
        return 0u;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    valid = huaner_runtime.board_voltage_valid;
    if (valid != 0u) {
        *voltage_mv = huaner_runtime.board_voltage_mv;
    }
    if (primask == 0u) {
        __enable_irq();
    }
    return valid;
}

uint8_t HuanerServoGetStatus(uint8_t id, HuanerServo_Status_s *status)
{
    HuanerServo_Status_s *stored;
    uint32_t primask;

    if (!HuanerServoIdValid(id) || status == NULL) {
        return 0u;
    }
    primask = __get_PRIMASK();
    __disable_irq();
    stored = HuanerServoFindStatus(id, 0u);
    if (stored != NULL) {
        *status = *stored;
    }
    if (primask == 0u) {
        __enable_irq();
    }
    return stored != NULL;
}

uint8_t HuanerServoFeedbackFresh(uint8_t id, uint32_t now_ms)
{
    HuanerServo_Status_s status;

    if (HuanerServoGetStatus(id, &status) == 0u ||
        status.feedback_valid == 0u || status.last_feedback_tick == 0u) {
        return 0u;
    }
    return (uint32_t)(now_ms - status.last_feedback_tick) <=
        HUANER_SERVO_FEEDBACK_STALE_MS;
}

static void HuanerServoServiceHealth(uint32_t now_ms)
{
    uint8_t any_online = 0u;
    uint8_t index;

    for (index = 0u; index < HUANER_SERVO_STATUS_SLOT_COUNT; ++index) {
        HuanerServo_Status_s *status = &huaner_runtime.status[index];

        if (status->id == 0u) {
            continue;
        }
        if (status->last_feedback_tick == 0u ||
            (uint32_t)(now_ms - status->last_feedback_tick) >
                HUANER_SERVO_FEEDBACK_STALE_MS) {
            status->feedback_valid = 0u;
            status->arrived = 0u;
            status->arrival_stable_since_tick = 0u;
        }
        if (status->last_feedback_tick == 0u ||
            (uint32_t)(now_ms - status->last_feedback_tick) >
                HUANER_SERVO_OFFLINE_MS) {
            status->online = 0u;
        }
        if (status->target_valid != 0u && status->arrived == 0u &&
            (uint32_t)(now_ms - status->last_command_tick) >
                (uint32_t)status->command_time_ms +
                    HUANER_SERVO_MOTION_TIMEOUT_MARGIN_MS) {
            status->motion_timeout = 1u;
        }
        if (status->online != 0u) {
            any_online = 1u;
        }
    }
    if (huaner_runtime.board_voltage_valid != 0u &&
        (huaner_runtime.last_board_voltage_tick == 0u ||
         (uint32_t)(now_ms - huaner_runtime.last_board_voltage_tick) >
             HUANER_SERVO_BOARD_VOLTAGE_STALE_MS)) {
        huaner_runtime.board_voltage_valid = 0u;
        g_huaner_servo_driver_debug.board_voltage_valid = 0u;
    }
    g_huaner_servo_driver_debug.online = any_online;
}

static void HuanerServoServicePolling(uint32_t now_ms)
{
    HuanerServo_Result_e result;
    uint16_t poll_slot_period_ms;

    if (g_huaner_servo_driver_debug.busy != 0u ||
        g_huaner_servo_driver_debug.state != HUANER_SERVO_STATE_IDLE ||
        (huaner_runtime.poll_enabled == 0u &&
         huaner_runtime.board_voltage_poll_enabled == 0u)) {
        return;
    }
    if (huaner_runtime.poll_enabled != 0u &&
        huaner_runtime.poll_id_count != 0u &&
        (int32_t)(now_ms - huaner_runtime.next_poll_tick) >= 0) {
        /*
         * 多ID配置按单ID交替查询。一台舵机缺失时，另一台的反馈仍能
         * 独立更新；period_ms仍表示每台舵机的目标刷新周期。
         */
        result = HuanerServoRequestPosition(
            huaner_runtime.poll_ids[huaner_runtime.poll_next_index]);
        if (result == HUANER_SERVO_RESULT_OK) {
            huaner_runtime.poll_next_index++;
            if (huaner_runtime.poll_next_index >= huaner_runtime.poll_id_count) {
                huaner_runtime.poll_next_index = 0u;
            }
            poll_slot_period_ms = (uint16_t)(
                huaner_runtime.poll_period_ms / huaner_runtime.poll_id_count);
            if (poll_slot_period_ms == 0u) {
                poll_slot_period_ms = 1u;
            }
            huaner_runtime.next_poll_tick = now_ms + poll_slot_period_ms;
            g_huaner_servo_driver_debug.last_poll_tick = now_ms;
            g_huaner_servo_driver_debug.poll_request_count++;
            return;
        }
    }
    if (huaner_runtime.board_voltage_poll_enabled != 0u &&
        (int32_t)(now_ms - huaner_runtime.next_board_voltage_poll_tick) >= 0) {
        result = HuanerServoRequestBoardVoltage();
        if (result == HUANER_SERVO_RESULT_OK) {
            huaner_runtime.next_board_voltage_poll_tick =
                now_ms + huaner_runtime.board_voltage_poll_period_ms;
            g_huaner_servo_driver_debug.board_voltage_request_count++;
        }
    }
}

void HuanerServoTask(uint32_t now_ms)
{
    HAL_StatusTypeDef hal_result;
    HuanerServo_Result_e result;

    if (huaner_runtime.initialized == 0u) {
        return;
    }
    HuanerServoServiceHealth(now_ms);

    if (huaner_runtime.uart_error != 0u) {
        HuanerServoRecoverUart();
        g_huaner_servo_driver_debug.hal_error_count++;
        if (g_huaner_servo_driver_debug.busy != 0u) {
            HuanerServoFinish(HUANER_SERVO_RESULT_HAL_ERROR, now_ms);
        } else {
            g_huaner_servo_driver_debug.last_result = HUANER_SERVO_RESULT_HAL_ERROR;
            g_huaner_servo_driver_debug.state = HUANER_SERVO_STATE_ERROR;
            huaner_runtime.state_tick = now_ms;
        }
        return;
    }

    switch (g_huaner_servo_driver_debug.state) {
        case HUANER_SERVO_STATE_PREPARE_RX:
            hal_result = HuanerServoStartReceive(now_ms);
            if (hal_result == HAL_OK) {
                g_huaner_servo_driver_debug.state = HUANER_SERVO_STATE_TX_DMA;
            } else if (hal_result != HAL_BUSY) {
                g_huaner_servo_driver_debug.hal_error_count++;
                HuanerServoRecoverUart();
                HuanerServoFinish(HUANER_SERVO_RESULT_HAL_ERROR, now_ms);
            } else if ((uint32_t)(now_ms - huaner_runtime.state_tick) >=
                       HUANER_SERVO_TX_TIMEOUT_MS) {
                HuanerServoCountTimeout();
                HuanerServoRecoverUart();
                HuanerServoFinish(HUANER_SERVO_RESULT_TIMEOUT, now_ms);
            }
            break;

        case HUANER_SERVO_STATE_TX_DMA:
            hal_result = HAL_UART_Transmit_DMA(
                &huart6, huaner_runtime.tx_buffer, huaner_runtime.tx_length);
            if (hal_result == HAL_OK) {
                huaner_runtime.state_tick = now_ms;
                g_huaner_servo_driver_debug.state =
                    HUANER_SERVO_STATE_WAIT_TX_COMPLETE;
            } else if (hal_result != HAL_BUSY) {
                g_huaner_servo_driver_debug.hal_error_count++;
                HuanerServoRecoverUart();
                HuanerServoFinish(HUANER_SERVO_RESULT_HAL_ERROR, now_ms);
            } else if ((uint32_t)(now_ms - huaner_runtime.state_tick) >=
                       HUANER_SERVO_TX_TIMEOUT_MS) {
                HuanerServoCountTimeout();
                HuanerServoRecoverUart();
                HuanerServoFinish(HUANER_SERVO_RESULT_TIMEOUT, now_ms);
            }
            break;

        case HUANER_SERVO_STATE_WAIT_TX_COMPLETE:
            if (huaner_runtime.tx_complete != 0u) {
                huaner_runtime.tx_complete = 0u;
                HuanerServoCountTx();
                huaner_runtime.state_tick = now_ms;
                if (huaner_runtime.transaction ==
                        HUANER_TRANSACTION_POSITION_READ ||
                    huaner_runtime.transaction ==
                        HUANER_TRANSACTION_BOARD_VOLTAGE_READ) {
                    g_huaner_servo_driver_debug.state =
                        huaner_runtime.rx_complete != 0u ?
                        HUANER_SERVO_STATE_VALIDATE :
                        HUANER_SERVO_STATE_WAIT_RESPONSE;
                } else {
                    g_huaner_servo_driver_debug.state =
                        HUANER_SERVO_STATE_WAIT_TX_GAP;
                }
            } else if ((uint32_t)(now_ms - huaner_runtime.state_tick) >=
                       HUANER_SERVO_TX_TIMEOUT_MS) {
                HuanerServoCountTimeout();
                HuanerServoRecoverUart();
                HuanerServoFinish(HUANER_SERVO_RESULT_TIMEOUT, now_ms);
            }
            break;

        case HUANER_SERVO_STATE_WAIT_TX_GAP:
            if ((uint32_t)(now_ms - huaner_runtime.state_tick) >=
                HUANER_SERVO_BOARD_TX_GAP_MS) {
                HuanerServoFinish(HUANER_SERVO_RESULT_OK, now_ms);
            }
            break;

        case HUANER_SERVO_STATE_WAIT_RESPONSE:
            if (huaner_runtime.rx_complete != 0u) {
                g_huaner_servo_driver_debug.state = HUANER_SERVO_STATE_VALIDATE;
            } else if ((uint32_t)(now_ms - huaner_runtime.state_tick) >=
                       HUANER_SERVO_RESPONSE_TIMEOUT_MS) {
                HuanerServoCountTimeout();
                HuanerServoRecoverUart();
                HuanerServoFinish(HUANER_SERVO_RESULT_TIMEOUT, now_ms);
            }
            break;

        case HUANER_SERVO_STATE_VALIDATE:
            result = HuanerServoValidateReply(now_ms);
            if (result != HUANER_SERVO_RESULT_OK) {
                g_huaner_servo_driver_debug.frame_fail_count++;
            }
            HuanerServoCopyDebugFrames();
            HuanerServoFinish(result, now_ms);
            break;

        case HUANER_SERVO_STATE_COMPLETE:
        case HUANER_SERVO_STATE_TIMEOUT:
        case HUANER_SERVO_STATE_ERROR:
            if (g_huaner_servo_driver_debug.busy == 0u) {
                g_huaner_servo_driver_debug.state = HUANER_SERVO_STATE_IDLE;
            }
            break;

        case HUANER_SERVO_STATE_UNINITIALIZED:
        case HUANER_SERVO_STATE_IDLE:
        default:
            break;
    }

    HuanerServoServicePolling(now_ms);
}
