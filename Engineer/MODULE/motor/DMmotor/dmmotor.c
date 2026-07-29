#include "dmmotor.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static uint8_t dm_motor_count;
static DM_MotorInstance *dm_motor_instances[DM_MOTOR_CNT];
static uint32_t dm_last_control_tick;
static uint8_t dm_rotation_start;

volatile DM_Init_Error_e g_dm_motor_last_init_error = DM_INIT_OK;

static float DMMotorUintToFloat(uint16_t value,
                                float min_value,
                                float max_value,
                                uint8_t bits)
{
    float span = max_value - min_value;
    uint32_t full_scale = ((uint32_t)1u << bits) - 1u;

    return (float)value * span / (float)full_scale + min_value;
}

static uint8_t DMMotorTransmit(DM_MotorInstance *motor)
{
    uint8_t result;

    if (motor == NULL || motor->motor_can_instance == NULL) {
        return 0u;
    }
    result = CANTransmit(motor->motor_can_instance, 0.1f);
    if (result != 0u) {
        motor->tx_count++;
        motor->consecutive_tx_fail = 0u;
        return 1u;
    }

    motor->tx_fail_count++;
    motor->consecutive_tx_fail++;
    if (motor->consecutive_tx_fail >= DM_MAX_CONSECUTIVE_TX_FAIL) {
        motor->tx_fault_latched = 1u;
        motor->fault_latched = 1u;
    }
    return 0u;
}

static uint8_t DMMotorSendModeCommand(DM_MotorInstance *motor,
                                      DMMotor_Mode_e command)
{
    uint32_t saved_std_id;
    uint32_t saved_tx_id;
    uint8_t result;

    if (motor == NULL || motor->motor_can_instance == NULL) {
        return 0u;
    }
    memset(motor->motor_can_instance->tx_buff, 0xff, 7u);
    motor->motor_can_instance->tx_buff[7] = (uint8_t)command;
    /*
     * 达妙特殊命令使用原始Motor ID：1/2/3；位置速度控制帧才使用
     * 0x100+Motor ID：0x101/0x102/0x103。只在本次同步发送期间切换
     * header，HAL_CAN_AddTxMessage返回后立即恢复位置速度发送ID。
     */
    saved_std_id = motor->motor_can_instance->txconf.StdId;
    saved_tx_id = motor->motor_can_instance->tx_id;
    motor->motor_can_instance->txconf.StdId = motor->motor_id;
    motor->motor_can_instance->tx_id = motor->motor_id;
    motor->last_mode_tx_id = motor->motor_id;
    motor->mode_command_count++;
    result = DMMotorTransmit(motor);
    motor->motor_can_instance->txconf.StdId = saved_std_id;
    motor->motor_can_instance->tx_id = saved_tx_id;
    return result;
}

static void DMMotorDecode(CAN_Instance *motor_can)
{
    DM_MotorInstance *motor;
    uint8_t feedback_motor_id;
    uint8_t feedback_state;
    uint16_t raw_position;
    uint16_t raw_velocity;
    uint16_t raw_torque;
    uint32_t next_rx_count;

    if (motor_can == NULL || motor_can->id == NULL) {
        return;
    }
    motor = (DM_MotorInstance *)motor_can->id;
    if (motor_can->rx_len != 8u) {
        motor->measure.invalid_rx_count++;
        return;
    }

    feedback_motor_id = motor_can->rx_buff[0] & 0x0fu;
    if (feedback_motor_id != (uint8_t)motor->motor_id) {
        motor->measure.invalid_rx_count++;
        return;
    }

    raw_position = (uint16_t)(((uint16_t)motor_can->rx_buff[1] << 8) |
                              motor_can->rx_buff[2]);
    raw_velocity = (uint16_t)(((uint16_t)motor_can->rx_buff[3] << 4) |
                              (motor_can->rx_buff[4] >> 4));
    raw_torque = (uint16_t)((((uint16_t)motor_can->rx_buff[4] & 0x0fu) << 8) |
                            motor_can->rx_buff[5]);

    feedback_state = (motor_can->rx_buff[0] >> 4) & 0x0fu;
    next_rx_count = motor->measure.rx_count + 1u;
    motor->measure.motor_id = feedback_motor_id;
    motor->measure.state = feedback_state;
    motor->measure.position_rad = DMMotorUintToFloat(
        raw_position, DM_P_MIN, DM_P_MAX, 16u);
    motor->measure.velocity_rad_s = DMMotorUintToFloat(
        raw_velocity, DM_V_MIN, DM_V_MAX, 12u);
    motor->measure.torque_nm = DMMotorUintToFloat(
        raw_torque, DM_T_MIN, DM_T_MAX, 12u);
    motor->measure.mos_temperature_c = (float)motor_can->rx_buff[6];
    motor->measure.rotor_temperature_c = (float)motor_can->rx_buff[7];
    motor->measure.last_feedback_tick = HAL_GetTick();
    motor->measure.rx_count = next_rx_count;
    motor->measure.feedback_valid = 1u;
    motor->offline_latched = 0u;
    /*
     * Enter Motor Mode帧成功进入CAN邮箱不等于电机已经使能。只有请求后
     * 收到的新反馈明确报告state=1，才允许位置速度控制开始发送。
     */
    if (motor->mode_request_pending != 0u &&
        next_rx_count > motor->mode_request_rx_count &&
        feedback_state == DM_STATE_MOTOR_MODE) {
        motor->mode_request_pending = 0u;
        motor->mode_entered = 1u;
        if (motor->target_synced != 0u && motor->fault_latched == 0u) {
            motor->control_enabled = 1u;
        }
    }
    if (motor->motor_daemon != NULL) {
        DaemonReload(motor->motor_daemon);
    }
}

static uint8_t DMMotorConfigIsValid(const DM_Motor_Init_Config_s *config,
                                    uint16_t command_id)
{
    uint8_t index;

    if (config == NULL || config->can_handle == NULL) {
        g_dm_motor_last_init_error = DM_INIT_ERROR_ARGUMENT;
        return 0u;
    }
    if (dm_motor_count >= DM_MOTOR_CNT) {
        g_dm_motor_last_init_error = DM_INIT_ERROR_CAPACITY;
        return 0u;
    }
    if (config->motor_id == 0u || config->motor_id > 15u ||
        config->master_id == 0u || config->master_id > 0x7ffu) {
        g_dm_motor_last_init_error = DM_INIT_ERROR_ID;
        return 0u;
    }
    if (config->motor_type != DM4310 && config->motor_type != DM4340) {
        g_dm_motor_last_init_error = DM_INIT_ERROR_ARGUMENT;
        return 0u;
    }
    if (config->control_type != MOTOR_CONTROL_POSITION_AND_SPEED) {
        g_dm_motor_last_init_error = DM_INIT_ERROR_MODE;
        return 0u;
    }
    for (index = 0u; index < dm_motor_count; ++index) {
        const DM_MotorInstance *other = dm_motor_instances[index];

        if (other != NULL &&
            other->motor_can_instance != NULL &&
            other->motor_can_instance->can_handle == config->can_handle &&
            (other->command_id == command_id ||
             other->master_id == config->master_id)) {
            g_dm_motor_last_init_error = DM_INIT_ERROR_DUPLICATE;
            return 0u;
        }
    }
    return 1u;
}

DM_MotorInstance *DMMotorInit(const DM_Motor_Init_Config_s *config)
{
    uint16_t command_id;
    DM_MotorInstance *motor;
    CAN_Init_Config_s can_config;

    command_id = config != NULL ?
        (uint16_t)(0x100u + config->motor_id) : 0u;
    if (!DMMotorConfigIsValid(config, command_id)) {
        return NULL;
    }

    motor = (DM_MotorInstance *)malloc(sizeof(DM_MotorInstance));
    if (motor == NULL) {
        g_dm_motor_last_init_error = DM_INIT_ERROR_ALLOC;
        return NULL;
    }
    memset(motor, 0, sizeof(*motor));

    motor->motor_id = config->motor_id;
    motor->master_id = config->master_id;
    motor->command_id = command_id;
    motor->motor_type = config->motor_type;
    motor->control_type = config->control_type;
    motor->direction = config->direction;

    memset(&can_config, 0, sizeof(can_config));
    can_config.can_handle = config->can_handle;
    can_config.tx_id = command_id;
    can_config.rx_id = config->master_id;
    can_config.can_module_callback = DMMotorDecode;
    can_config.id = motor;
    motor->motor_can_instance = CANRegister(&can_config);
    if (motor->motor_can_instance == NULL) {
        free(motor);
        g_dm_motor_last_init_error = DM_INIT_ERROR_CAN_REGISTER;
        return NULL;
    }
    CANSetDLC(motor->motor_can_instance, 8u);

    dm_motor_instances[dm_motor_count++] = motor;
    g_dm_motor_last_init_error = DM_INIT_OK;
    return motor;
}

uint8_t DMMotorSetPositionSpeed(DM_MotorInstance *motor,
                                float position_rad,
                                float velocity_limit_rad_s)
{
    if (motor == NULL || motor->control_type !=
            MOTOR_CONTROL_POSITION_AND_SPEED ||
        !isfinite(position_rad) || !isfinite(velocity_limit_rad_s) ||
        position_rad < DM_P_MIN || position_rad > DM_P_MAX ||
        velocity_limit_rad_s <= 0.0f ||
        velocity_limit_rad_s > DM_V_MAX) {
        return 0u;
    }
    /*
     * module层始终使用达妙电机原始坐标。机械安装方向只在arm适配层
     * 转换，避免反馈与命令各自乘符号后出现双重反向。位置速度模式的
     * 第二个float是正值速度上限，不是带方向的速度指令。
     */
    motor->position_ref_rad = position_rad;
    motor->velocity_limit_rad_s = velocity_limit_rad_s;
    motor->target_synced = 1u;
    if (motor->mode_entered != 0u && motor->fault_latched == 0u) {
        motor->control_enabled = 1u;
    }
    return 1u;
}

uint8_t DMMotorHoldCurrentPosition(DM_MotorInstance *motor)
{
    if (motor == NULL || motor->measure.feedback_valid == 0u ||
        !isfinite(motor->measure.position_rad)) {
        return 0u;
    }
    motor->position_ref_rad = motor->measure.position_rad;
    motor->velocity_limit_rad_s = 0.01f;
    motor->target_synced = 1u;
    if (motor->mode_entered != 0u && motor->fault_latched == 0u) {
        motor->control_enabled = 1u;
    }
    return 1u;
}

uint8_t DMMotorEnterMode(DM_MotorInstance *motor)
{
    uint8_t result;

    if (motor == NULL || motor->motor_can_instance == NULL ||
        motor->fault_latched != 0u) {
        return 0u;
    }
    motor->control_enabled = 0u;
    motor->mode_entered = 0u;
    motor->mode_request_pending = 1u;
    motor->mode_request_rx_count = motor->measure.rx_count;
    result = DMMotorSendModeCommand(motor, DM_CMD_MOTOR_MODE);
    if (result == 0u) {
        motor->mode_request_pending = 0u;
    }
    return result;
}

uint8_t DMMotorEnterModeAndHoldOpenLoop(DM_MotorInstance *motor)
{
    uint8_t result;

    if (motor == NULL || motor->motor_can_instance == NULL ||
        motor->target_synced == 0u || motor->fault_latched != 0u) {
        return 0u;
    }
    motor->control_enabled = 0u;
    motor->mode_request_pending = 0u;
    motor->mode_entered = 0u;
    result = DMMotorSendModeCommand(motor, DM_CMD_MOTOR_MODE);
    if (result != 0u) {
        /*
         * 仅供三轴使能台架模式使用：当前位置目标已经预先同步，故使能帧
         * 入邮箱后立即开放连续位置速度保持帧，不等待尚未验证的state码。
         */
        motor->mode_entered = 1u;
        motor->control_enabled = 1u;
    }
    return result;
}

uint8_t DMMotorDisable(DM_MotorInstance *motor)
{
    uint8_t result;

    if (motor == NULL) {
        return 0u;
    }
    motor->control_enabled = 0u;
    motor->target_synced = 0u;
    motor->mode_request_pending = 0u;
    motor->mode_entered = 0u;
    result = DMMotorSendModeCommand(motor, DM_CMD_RESET_MODE);
    return result;
}

uint8_t DMMotorClearFault(DM_MotorInstance *motor)
{
    if (motor == NULL) {
        return 0u;
    }
    return DMMotorSendModeCommand(motor, DM_CMD_CLEAR_ERROR);
}

void DMMotorResetSoftwareFault(DM_MotorInstance *motor)
{
    if (motor == NULL) {
        return;
    }
    motor->fault_latched = 0u;
    motor->offline_latched = 0u;
    motor->tx_fault_latched = 0u;
    motor->consecutive_tx_fail = 0u;
    motor->control_enabled = 0u;
    motor->target_synced = 0u;
    motor->mode_request_pending = 0u;
    motor->mode_entered = 0u;
    motor->mode_request_rx_count = 0u;
    /* 故障复位必须等待复位后的新反馈，不能复用故障前的缓存帧。 */
    motor->measure.feedback_valid = 0u;
}

uint8_t DMMotorFeedbackValid(const DM_MotorInstance *motor)
{
    return motor != NULL && motor->measure.feedback_valid != 0u;
}

uint8_t DMMotorIsOnline(const DM_MotorInstance *motor, uint32_t now_ms)
{
    return DMMotorFeedbackValid(motor) &&
           (uint32_t)(now_ms - motor->measure.last_feedback_tick) <=
               DM_FEEDBACK_TIMEOUT_MS;
}

uint8_t DMMotorModeConfirmed(const DM_MotorInstance *motor)
{
    return motor != NULL && motor->measure.feedback_valid != 0u &&
           motor->mode_request_pending == 0u && motor->mode_entered != 0u &&
           motor->measure.state == DM_STATE_MOTOR_MODE;
}

uint8_t DMMotorHasActiveStateFault(const DM_MotorInstance *motor)
{
    if (motor == NULL || motor->measure.feedback_valid == 0u) {
        return 0u;
    }
    return motor->measure.state != 0u && motor->measure.state != 1u;
}

void DMMotorControl(uint32_t now_ms)
{
    uint8_t offset;

    if ((uint32_t)(now_ms - dm_last_control_tick) < DM_CONTROL_PERIOD_MS) {
        return;
    }
    dm_last_control_tick = now_ms;

    for (offset = 0u; offset < dm_motor_count; ++offset) {
        uint8_t index = (uint8_t)((dm_rotation_start + offset) %
                                  dm_motor_count);
        DM_MotorInstance *motor = dm_motor_instances[index];
        float position_ref;
        float velocity_ref;

        if (motor == NULL || motor->motor_can_instance == NULL) {
            continue;
        }
        if (motor->measure.feedback_valid != 0u &&
            !DMMotorIsOnline(motor, now_ms)) {
            motor->offline_latched = 1u;
            motor->fault_latched = 1u;
            motor->control_enabled = 0u;
        }
        if (motor->control_enabled == 0u ||
            motor->target_synced == 0u ||
            motor->measure.feedback_valid == 0u ||
            motor->fault_latched != 0u) {
            continue;
        }

        position_ref = motor->position_ref_rad;
        velocity_ref = motor->velocity_limit_rad_s;
        memcpy(motor->motor_can_instance->tx_buff,
               &position_ref, sizeof(position_ref));
        memcpy(motor->motor_can_instance->tx_buff + 4u,
               &velocity_ref, sizeof(velocity_ref));
        DMMotorTransmit(motor);
    }
    if (dm_motor_count != 0u) {
        dm_rotation_start = (uint8_t)((dm_rotation_start + 1u) %
                                      dm_motor_count);
    }
}
