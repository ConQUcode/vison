/**
 * @file dmmotor.h
 * @brief 达妙电机位置速度模式、特殊模式命令和反馈健康接口。
 */

#ifndef DMMOTOR_H
#define DMMOTOR_H

#include <stdint.h>

#include "bsp_can.h"
#include "daemon.h"
#include "motor_def.h"

#define DM_MOTOR_CNT                   5u
#define DM_FEEDBACK_TIMEOUT_MS       100u
#define DM_CONTROL_PERIOD_MS           2u
#define DM_MAX_CONSECUTIVE_TX_FAIL     5u
#define DM_STATE_DISABLED              0u
#define DM_STATE_MOTOR_MODE            1u

#define DM_P_MIN     (-12.5f)
#define DM_P_MAX       12.5f
#define DM_V_MIN     (-30.0f)
#define DM_V_MAX       30.0f
#define DM_T_MIN     (-10.0f)
#define DM_T_MAX       10.0f

typedef enum {
    DM_INIT_OK = 0,
    DM_INIT_ERROR_ARGUMENT,
    DM_INIT_ERROR_CAPACITY,
    DM_INIT_ERROR_ID,
    DM_INIT_ERROR_MODE,
    DM_INIT_ERROR_DUPLICATE,
    DM_INIT_ERROR_ALLOC,
    DM_INIT_ERROR_CAN_REGISTER
} DM_Init_Error_e;

typedef enum {
    DM_CMD_MOTOR_MODE    = 0xfcu,
    DM_CMD_RESET_MODE    = 0xfdu,
    DM_CMD_ZERO_POSITION = 0xfeu,
    DM_CMD_CLEAR_ERROR   = 0xfbu
} DMMotor_Mode_e;

typedef struct {
    CAN_HandleTypeDef *can_handle;
    uint16_t motor_id;
    uint16_t master_id;
    Motor_Type_e motor_type;
    Motor_Controll_Type_e control_type;
    Motor_Reverse_Flag_e direction;
} DM_Motor_Init_Config_s;

typedef struct {
    uint8_t motor_id;
    uint8_t state;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    float mos_temperature_c;
    float rotor_temperature_c;
    uint8_t feedback_valid;
    uint32_t rx_count;
    uint32_t invalid_rx_count;
    uint32_t last_feedback_tick;
} DM_Motor_Measure_s;

typedef struct {
    volatile DM_Motor_Measure_s measure;
    CAN_Instance *motor_can_instance;
    Daemon_Instance *motor_daemon;

    uint16_t motor_id;
    uint16_t master_id;
    uint16_t command_id;
    Motor_Type_e motor_type;
    Motor_Controll_Type_e control_type;
    Motor_Reverse_Flag_e direction;

    volatile float position_ref_rad;
    volatile float velocity_limit_rad_s;
    volatile uint8_t target_synced;
    volatile uint8_t control_enabled;
    volatile uint8_t mode_request_pending;
    volatile uint8_t mode_entered;
    volatile uint8_t fault_latched;
    volatile uint8_t offline_latched;
    volatile uint8_t tx_fault_latched;

    volatile uint32_t tx_count;
    volatile uint32_t tx_fail_count;
    volatile uint32_t consecutive_tx_fail;
    volatile uint32_t mode_command_count;
    volatile uint32_t mode_request_rx_count;
    volatile uint16_t last_mode_tx_id;
} DM_MotorInstance;

extern volatile DM_Init_Error_e g_dm_motor_last_init_error;

/** 注册一个达妙电机；失败返回 NULL 并更新 g_dm_motor_last_init_error。 */
DM_MotorInstance *DMMotorInit(const DM_Motor_Init_Config_s *config);
/** 允许 1 kHz 调用，内部按 2 ms 轮转发送已启用电机目标。 */
void DMMotorControl(uint32_t now_ms);

uint8_t DMMotorSetPositionSpeed(DM_MotorInstance *motor,
                                float position_rad,
                                float velocity_limit_rad_s);
uint8_t DMMotorHoldCurrentPosition(DM_MotorInstance *motor);
/** 发送 Enter Motor Mode 特殊帧；成功发送不等于已收到模式确认。 */
uint8_t DMMotorEnterMode(DM_MotorInstance *motor);
uint8_t DMMotorEnterModeAndHoldOpenLoop(DM_MotorInstance *motor);
uint8_t DMMotorDisable(DM_MotorInstance *motor);
uint8_t DMMotorClearFault(DM_MotorInstance *motor);
void DMMotorResetSoftwareFault(DM_MotorInstance *motor);

uint8_t DMMotorFeedbackValid(const DM_MotorInstance *motor);
/** 判断反馈帧有效且距离最近反馈不超过配置超时。 */
uint8_t DMMotorIsOnline(const DM_MotorInstance *motor, uint32_t now_ms);
uint8_t DMMotorModeConfirmed(const DM_MotorInstance *motor);
uint8_t DMMotorHasActiveStateFault(const DM_MotorInstance *motor);

#endif
