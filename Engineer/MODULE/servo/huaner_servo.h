/**
 * @file huaner_servo.h
 * @brief 幻儿舵机控制板的公开接口、反馈状态和通信调参项。
 */

#ifndef HUANER_SERVO_H
#define HUANER_SERVO_H

#include <stdint.h>

/* 控制板支持的 ID、位置值和动作时间硬边界，业务层不得绕过。 */
#define HUANER_SERVO_MIN_ID                         1u
#define HUANER_SERVO_MAX_ID                       253u
#define HUANER_SERVO_MIN_POSITION                   0u
#define HUANER_SERVO_MAX_POSITION                1000u
#define HUANER_SERVO_MAX_TIME_MS                 30000u
#define HUANER_SERVO_MAX_FEEDBACK_IDS                2u

/* 必须与 CubeMX USART6 的 9600、8N1、收发模式一致。 */
#define HUANER_SERVO_CONTROLLER_BAUD_RATE          9600u
/* 每台舵机位置反馈周期；减小会增加 9600 波特率总线占用。 */
#define HUANER_SERVO_DEFAULT_POLL_PERIOD_MS         50u
/* 控制板供电电压读取周期，不参与夹爪堵转判断。 */
#define HUANER_SERVO_DEFAULT_VOLTAGE_POLL_PERIOD_MS 500u
/* 100 ms 后反馈不再用于闭环，200 ms 后舵机标记离线。 */
#define HUANER_SERVO_FEEDBACK_STALE_MS              100u
#define HUANER_SERVO_OFFLINE_MS                     200u
#define HUANER_SERVO_BOARD_VOLTAGE_STALE_MS        1500u
/* 电压回复的合理范围，仅用于拒绝坏帧，不是欠压保护阈值。 */
#define HUANER_SERVO_BOARD_VOLTAGE_MIN_MV          3000u
#define HUANER_SERVO_BOARD_VOLTAGE_MAX_MV         15000u
/* 到位需同时满足位置误差、反馈速度和连续稳定时间。 */
#define HUANER_SERVO_ARRIVAL_TOLERANCE_POS           15u
/* 临时调试：ID1俯仰舵机中位偏差偏大，单独放宽到位误差，不影响ID2夹爪。 */
#define HUANER_SERVO_ID1_ARRIVAL_TOLERANCE_POS       25u
#define HUANER_SERVO_ARRIVAL_VELOCITY_POS_S       100.0f
#define HUANER_SERVO_ARRIVAL_STABLE_MS              120u
#define HUANER_SERVO_MOTION_TIMEOUT_MARGIN_MS        500u

typedef enum {
    HUANER_SERVO_RESULT_OK = 0,
    HUANER_SERVO_RESULT_BUSY,
    HUANER_SERVO_RESULT_INVALID,
    HUANER_SERVO_RESULT_NOT_INITIALIZED,
    HUANER_SERVO_RESULT_HAL_ERROR,
    HUANER_SERVO_RESULT_TIMEOUT,
    HUANER_SERVO_RESULT_FRAME_ERROR,
    HUANER_SERVO_RESULT_CHECKSUM_ERROR, /* 兼容保留；控制板协议没有校验字段。 */
    HUANER_SERVO_RESULT_ID_MISMATCH,
    HUANER_SERVO_RESULT_COMMAND_MISMATCH,
    HUANER_SERVO_RESULT_POSITION_RANGE,
    HUANER_SERVO_RESULT_VOLTAGE_RANGE,
    HUANER_SERVO_RESULT_NOT_FOUND,
    HUANER_SERVO_RESULT_UNSUPPORTED
} HuanerServo_Result_e;

typedef enum {
    HUANER_SERVO_STATE_UNINITIALIZED = 0,
    HUANER_SERVO_STATE_IDLE,
    HUANER_SERVO_STATE_PREPARE_RX,
    HUANER_SERVO_STATE_TX_DMA,
    HUANER_SERVO_STATE_WAIT_TX_COMPLETE,
    HUANER_SERVO_STATE_WAIT_TX_GAP,
    HUANER_SERVO_STATE_WAIT_RESPONSE,
    HUANER_SERVO_STATE_VALIDATE,
    HUANER_SERVO_STATE_COMPLETE,
    HUANER_SERVO_STATE_TIMEOUT,
    HUANER_SERVO_STATE_ERROR
} HuanerServo_State_e;

typedef enum {
    HUANER_SERVO_COMMAND_NONE = 0,
    HUANER_SERVO_COMMAND_MOVE = 3,               /* CMD_SERVO_MOVE，0x03。 */
    /* 兼容旧接口的内部标记；控制板协议没有按单个ID停止舵机命令。 */
    HUANER_SERVO_COMMAND_STOP = 12,
    HUANER_SERVO_COMMAND_BOARD_VOLTAGE_READ = 15, /* 0x0F。 */
    HUANER_SERVO_COMMAND_UNLOAD = 20,              /* CMD_MULT_SERVO_UNLOAD，0x14。 */
    HUANER_SERVO_COMMAND_POSITION_READ = 21       /* 0x15。 */
} HuanerServo_Command_e;

typedef struct {
    uint8_t id;
    uint8_t online;
    uint8_t feedback_valid;
    uint8_t target_valid;
    uint8_t arrived;
    uint8_t motion_timeout;
    HuanerServo_Command_e last_command;
    HuanerServo_Result_e last_result;
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
} HuanerServo_Status_s;

typedef struct {
    uint8_t initialized;
    uint8_t uart_config_valid;
    uint8_t busy;
    uint8_t current_id;
    uint8_t current_id_count;
    uint8_t current_ids[HUANER_SERVO_MAX_FEEDBACK_IDS];
    HuanerServo_State_e state;
    HuanerServo_Command_e current_command;
    HuanerServo_Command_e last_command;
    HuanerServo_Result_e last_result;
    uint16_t last_position;
    uint8_t last_position_valid;
    uint8_t online;
    uint8_t protocol_self_test_passed;
    uint8_t expected_rx_length;
    uint8_t poll_enabled;
    uint8_t poll_id_count;
    uint8_t poll_ids[HUANER_SERVO_MAX_FEEDBACK_IDS];
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
} HuanerServo_Debug_s;

extern HuanerServo_Debug_s g_huaner_servo_driver_debug;

/** 初始化 USART6 回调、协议自检和内部事务状态；成功返回 1。 */
uint8_t HuanerServoInit(void);
/** 发送单舵机位置动作，position 为 0..1000，time_ms 为动作时间。 */
HuanerServo_Result_e HuanerServoMove(uint8_t id,
                               uint16_t position,
                               uint16_t time_ms);
/** 在控制板的一帧中同步发送两个舵机目标。 */
HuanerServo_Result_e HuanerServoMove2(uint8_t id1,
                                uint16_t position1,
                                uint8_t id2,
                                uint16_t position2,
                                uint16_t time_ms);
HuanerServo_Result_e HuanerServoStop(uint8_t id);
/**
 * @brief 通过控制板0x14命令卸载指定舵机，使其失去保持力矩。
 * @param ids 舵机ID数组，ID必须有效且不能重复。
 * @param count 舵机数量，范围1..HUANER_SERVO_MAX_FEEDBACK_IDS。
 * @return 命令进入USART6异步发送队列返回OK；该命令本身没有应答帧。
 * @note 卸载不会关闭控制板，之后仍可用0x15继续查询舵机位置。
 */
HuanerServo_Result_e HuanerServoUnload(const uint8_t *ids, uint8_t count);
/** 请求单个舵机位置；回复由 HuanerServoTask 异步解析。 */
HuanerServo_Result_e HuanerServoRequestPosition(uint8_t id);
HuanerServo_Result_e HuanerServoRequestPositions(const uint8_t *ids,
                                            uint8_t count);
HuanerServo_Result_e HuanerServoRequestBoardVoltage(void);
/*
 * period_ms表示每个ID的目标反馈周期；配置多个ID时驱动会拆成单ID帧
 * 依次轮询，避免一台舵机无应答导致其他舵机的在线状态一起失效。
 */
/** 启用内部交替单 ID 位置轮询，period_ms 是每个 ID 的目标周期。 */
uint8_t HuanerServoConfigureFeedbackPolling(const uint8_t *ids,
                                          uint8_t count,
                                          uint16_t period_ms);
void HuanerServoDisableFeedbackPolling(void);
uint8_t HuanerServoConfigureBoardVoltagePolling(uint16_t period_ms);
void HuanerServoDisableBoardVoltagePolling(void);
uint8_t HuanerServoGetBoardVoltage(uint16_t *voltage_mv);
/** 复制指定 ID 的最新状态；找不到该 ID 时返回 0。 */
uint8_t HuanerServoGetStatus(uint8_t id, HuanerServo_Status_s *status);
/** 判断指定 ID 的位置反馈是否仍处于 100 ms 新鲜窗口。 */
uint8_t HuanerServoFeedbackFresh(uint8_t id, uint32_t now_ms);
/** 1 kHz 推进 DMA 事务、解析回复、健康监督和自动轮询。 */
void HuanerServoTask(uint32_t now_ms);

#endif
