#include "arm_tool.h"

#include "arm_config.h"
#include "gpio.h"
#include "hsl_servo.h"
#include "stm32f4xx_hal.h"

#include <math.h>
#include <string.h>

#define ARM_TOOL_PI          3.14159265358979323846f
#define ARM_TOOL_DEG_TO_RAD  (ARM_TOOL_PI / 180.0f)
#define ARM_TOOL_RAD_TO_DEG  (180.0f / ARM_TOOL_PI)

Arm_Tool_State_s g_arm_tool_debug;

typedef struct {
    uint8_t valid[2];
    uint16_t position[2];
    uint16_t time_ms[2];
    uint32_t queued_tick[2];
    uint8_t next_single_index;
} Arm_Tool_Tx_Scheduler_s;

static Arm_Tool_Tx_Scheduler_s arm_tool_tx_scheduler;

static uint8_t ArmToolServoIndex(uint8_t servo_id)
{
    return servo_id == ARM_TOOL_SERVO1_ID ? 0u : 1u;
}

static float ArmToolServo2MaxYawAbsDeg(void)
{
    return fmaxf(fabsf(ARM_USB_YAW_MIN_DEG),
                 fabsf(ARM_USB_YAW_MAX_DEG));
}

static float ArmToolServo2HalfRangePos(void)
{
    return ((float)ARM_TOOL_SERVO2_POS_MAX -
            (float)ARM_TOOL_SERVO2_POS_MIN);
}

static uint8_t ArmToolServoAngleFiniteAndInRange(uint8_t servo_id,
                                                  float angle_deg)
{
    if (!isfinite(angle_deg)) {
        return 0u;
    }
    if (servo_id == ARM_TOOL_SERVO2_ID) {
        return angle_deg >= ARM_TOOL_SERVO2_LOGIC_MIN_DEG &&
               angle_deg <= ARM_TOOL_SERVO2_LOGIC_MAX_DEG;
    }
    return servo_id == ARM_TOOL_SERVO1_ID &&
           angle_deg >= ARM_TOOL_SERVO_DEG_MIN &&
           angle_deg <= ARM_TOOL_SERVO_DEG_MAX;
}

static void ArmToolClearPending(uint8_t index)
{
    arm_tool_tx_scheduler.valid[index] = 0u;
    g_arm_tool_debug.tx_pending[index] = 0u;
}

static void ArmToolRecordTxSuccess(uint8_t index)
{
    g_arm_tool_debug.tx_count[index]++;
    g_arm_tool_debug.servo_online[index] = 1u;
}

static void ArmToolRecordTxFailure(uint8_t index)
{
    g_arm_tool_debug.tx_fail_count[index]++;
    g_arm_tool_debug.servo_online[index] = 0u;
    ArmToolClearPending(index);
}

static void ArmToolDispatchPending(uint32_t now_ms)
{
    HSLServo_Result_e result;
    uint8_t index;

    if (g_hsl_servo_debug.busy != 0u ||
        (arm_tool_tx_scheduler.valid[0] == 0u &&
         arm_tool_tx_scheduler.valid[1] == 0u)) {
        return;
    }

    if (arm_tool_tx_scheduler.valid[0] != 0u &&
        arm_tool_tx_scheduler.valid[1] != 0u &&
        arm_tool_tx_scheduler.time_ms[0] ==
            arm_tool_tx_scheduler.time_ms[1]) {
        result = HSLServoMove2(
            ARM_TOOL_SERVO1_ID, arm_tool_tx_scheduler.position[0],
            ARM_TOOL_SERVO2_ID, arm_tool_tx_scheduler.position[1],
            arm_tool_tx_scheduler.time_ms[0]);
        if (result == HSL_SERVO_RESULT_BUSY) {
            return;
        }
        if (result == HSL_SERVO_RESULT_OK) {
            ArmToolRecordTxSuccess(0u);
            ArmToolRecordTxSuccess(1u);
            ArmToolClearPending(0u);
            ArmToolClearPending(1u);
            g_arm_tool_debug.tx_dual_frame_count++;
            g_arm_tool_debug.tool_ready = 1u;
            g_arm_tool_debug.error_code = ARM_TOOL_ERROR_NONE;
        } else {
            ArmToolRecordTxFailure(0u);
            ArmToolRecordTxFailure(1u);
            g_arm_tool_debug.tool_ready = 0u;
            g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_TX;
        }
        return;
    }

    if (arm_tool_tx_scheduler.valid[0] != 0u &&
        arm_tool_tx_scheduler.valid[1] != 0u) {
        if (arm_tool_tx_scheduler.queued_tick[0] ==
            arm_tool_tx_scheduler.queued_tick[1]) {
            index = arm_tool_tx_scheduler.next_single_index;
        } else {
            index = (int32_t)(arm_tool_tx_scheduler.queued_tick[0] -
                              arm_tool_tx_scheduler.queued_tick[1]) <= 0 ?
                0u : 1u;
        }
    } else {
        index = arm_tool_tx_scheduler.valid[0] != 0u ? 0u : 1u;
    }

    result = HSLServoMove(
        index == 0u ? ARM_TOOL_SERVO1_ID : ARM_TOOL_SERVO2_ID,
        arm_tool_tx_scheduler.position[index],
        arm_tool_tx_scheduler.time_ms[index]);
    if (result == HSL_SERVO_RESULT_BUSY) {
        return;
    }
    if (result == HSL_SERVO_RESULT_OK) {
        ArmToolRecordTxSuccess(index);
        ArmToolClearPending(index);
        arm_tool_tx_scheduler.next_single_index = index == 0u ? 1u : 0u;
        g_arm_tool_debug.tx_single_frame_count++;
        g_arm_tool_debug.tool_ready =
            g_arm_tool_debug.servo_online[0] != 0u &&
            g_arm_tool_debug.servo_online[1] != 0u;
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_NONE;
    } else {
        ArmToolRecordTxFailure(index);
        g_arm_tool_debug.tool_ready = 0u;
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_TX;
    }
    (void)now_ms;
}

static Arm_Command_Result_e ArmToolSendServo(uint8_t servo_id,
                                             float angle_deg,
                                             uint16_t time_ms,
                                             uint8_t force_send)
{
    uint8_t index = ArmToolServoIndex(servo_id);
    uint16_t pos;
    uint32_t now_ms = HAL_GetTick();

#if ARM_TOOL_ENABLE == 0u
    (void)servo_id;
    (void)angle_deg;
    (void)time_ms;
    (void)force_send;
    return ARM_COMMAND_UNSUPPORTED;
#else
    if (g_arm_tool_debug.initialized == 0u) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_INVALID_ARGUMENT;
        return ARM_COMMAND_NOT_READY;
    }
    if ((servo_id != ARM_TOOL_SERVO1_ID && servo_id != ARM_TOOL_SERVO2_ID) ||
        !ArmToolServoAngleFiniteAndInRange(servo_id, angle_deg) ||
        time_ms > HSL_SERVO_MAX_TIME_MS) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_RANGE;
        return ARM_COMMAND_INVALID;
    }

    if (force_send == 0u) {
        /*
         * 周期和角度死区是两个独立门槛：轨迹层可能每1ms调用一次，
         * 必须先硬性限频，再判断是否真的有足够角度变化需要更新。
         */
        if ((uint32_t)(now_ms - g_arm_tool_debug.last_update_tick) <
            ARM_TOOL_SERVO_UPDATE_PERIOD_MS) {
            return ARM_COMMAND_OK;
        }
        if (fabsf(angle_deg - g_arm_tool_debug.servo_target_deg[index]) <
            ARM_TOOL_SERVO_COMMAND_DEADBAND_DEG) {
            return ARM_COMMAND_OK;
        }
    }

    pos = ArmToolAngleDegToPos(servo_id, angle_deg);
    if (arm_tool_tx_scheduler.valid[index] != 0u) {
        g_arm_tool_debug.tx_pending_overwrite_count[index]++;
    }
    arm_tool_tx_scheduler.valid[index] = 1u;
    arm_tool_tx_scheduler.position[index] = pos;
    arm_tool_tx_scheduler.time_ms[index] = time_ms;
    arm_tool_tx_scheduler.queued_tick[index] = now_ms;
    g_arm_tool_debug.tx_pending[index] = 1u;
    g_arm_tool_debug.tx_pending_pos[index] = pos;
    g_arm_tool_debug.tx_pending_time_ms[index] = time_ms;
    g_arm_tool_debug.servo_target_deg[index] = angle_deg;
    g_arm_tool_debug.servo_target_pos[index] = pos;
    g_arm_tool_debug.last_update_tick = now_ms;
    return ARM_COMMAND_OK;
#endif
}

uint16_t ArmToolAngleDegToPos(uint8_t servo_id, float angle_deg)
{
    float neutral_pos;
    float pos_per_deg;
    float pos_f;
    float pos_min;
    float pos_max;
    float angle_min;
    float angle_max;
    float angle_range;

    if (!isfinite(angle_deg) ||
        (servo_id != ARM_TOOL_SERVO1_ID &&
         servo_id != ARM_TOOL_SERVO2_ID)) {
        return ARM_TOOL_SERVO_POS_MIN;
    }
    angle_min = servo_id == ARM_TOOL_SERVO2_ID ?
        ARM_TOOL_SERVO2_LOGIC_MIN_DEG : ARM_TOOL_SERVO_DEG_MIN;
    angle_max = servo_id == ARM_TOOL_SERVO2_ID ?
        ARM_TOOL_SERVO2_LOGIC_MAX_DEG : ARM_TOOL_SERVO_DEG_MAX;
    angle_range = servo_id == ARM_TOOL_SERVO2_ID ?
        ARM_TOOL_SERVO2_RANGE_DEG :
        (ARM_TOOL_SERVO_DEG_MAX - ARM_TOOL_SERVO_DEG_MIN);
    if (angle_deg < angle_min) {
        angle_deg = angle_min;
    }
    if (angle_deg > angle_max) {
        angle_deg = angle_max;
    }
    neutral_pos = servo_id == ARM_TOOL_SERVO1_ID ?
        (float)ARM_TOOL_SERVO1_NEUTRAL_POS :
        (float)ARM_TOOL_SERVO2_NEUTRAL_POS;
    pos_min = servo_id == ARM_TOOL_SERVO2_ID ?
        (float)ARM_TOOL_SERVO2_POS_MIN : (float)ARM_TOOL_SERVO_POS_MIN;
    pos_max = servo_id == ARM_TOOL_SERVO2_ID ?
        (float)ARM_TOOL_SERVO2_POS_MAX : (float)ARM_TOOL_SERVO_POS_MAX;
    pos_per_deg = (pos_max - pos_min) / angle_range;
    pos_f = neutral_pos +
        (angle_deg - ARM_TOOL_SERVO_NEUTRAL_DEG) * pos_per_deg;
    if (pos_f < pos_min) {
        return (uint16_t)pos_min;
    }
    if (pos_f > pos_max) {
        return (uint16_t)pos_max;
    }
    return (uint16_t)(pos_f + 0.5f);
}

float ArmToolServo1AngleForVerticalDown(float small_link_pitch_deg)
{
    if (!isfinite(small_link_pitch_deg) ||
        fabsf(ARM_TOOL_SERVO1_DIRECTION) < 0.000001f) {
        return NAN;
    }
#if ARM_TOOL_VERTICAL_COMPENSATION_ENABLE == 0u
    return ARM_TOOL_SERVO_NEUTRAL_DEG;
#else
    return ARM_TOOL_SERVO_NEUTRAL_DEG +
        ARM_TOOL_SERVO1_COMPENSATION_SCALE *
        small_link_pitch_deg / ARM_TOOL_SERVO1_DIRECTION;
#endif
}

uint8_t ArmToolServo1AngleValid(float angle_deg)
{
    return ArmToolServoAngleFiniteAndInRange(ARM_TOOL_SERVO1_ID,
                                              angle_deg);
}

void ArmToolInit(void)
{
#if ARM_TOOL_ENABLE != 0u
    uint8_t initialized;

    memset(&g_arm_tool_debug, 0, sizeof(g_arm_tool_debug));
    memset(&arm_tool_tx_scheduler, 0, sizeof(arm_tool_tx_scheduler));
    initialized = HSLServoInit();
    HAL_GPIO_WritePin(ARM_MAGNET_GPIO_PORT, ARM_MAGNET_GPIO_PIN,
                      ARM_MAGNET_INACTIVE_LEVEL);
    g_arm_tool_debug.initialized = initialized;
    g_arm_tool_debug.magnet_on = 0u;
    g_arm_tool_debug.vertical_down_enabled =
        ARM_TOOL_VERTICAL_COMPENSATION_ENABLE != 0u ? 1u : 0u;
    g_arm_tool_debug.servo_target_deg[0] = ARM_TOOL_SERVO1_INIT_DEG;
    g_arm_tool_debug.servo_target_deg[1] = ARM_TOOL_SERVO2_FIXED_DEG;
    g_arm_tool_debug.servo2_base_compensation_enabled =
        ARM_TOOL_SERVO2_BASE_COMPENSATION_ENABLE != 0u ? 1u : 0u;
    g_arm_tool_debug.servo2_target_in_range = 1u;
    g_arm_tool_debug.servo2_world_yaw_target_deg = 0.0f;
    g_arm_tool_debug.servo2_q1_feedback_deg = 0.0f;
    g_arm_tool_debug.servo2_base_compensation_deg = 0.0f;
    g_arm_tool_debug.servo2_relative_target_deg = 0.0f;
    g_arm_tool_debug.servo2_logic_target_deg = ARM_TOOL_SERVO2_FIXED_DEG;
    g_arm_tool_debug.servo2_compensated_target_pos =
        ARM_TOOL_SERVO2_NEUTRAL_POS;
    g_arm_tool_debug.servo2_tracking_tick = HAL_GetTick();
    g_arm_tool_debug.servo1_compensation_target_deg =
        ARM_TOOL_SERVO1_INIT_DEG;
    g_arm_tool_debug.servo1_slew_tick = HAL_GetTick();
    g_arm_tool_debug.servo1_slew_active = 0u;
    g_arm_tool_debug.servo_target_pos[0] =
        ArmToolAngleDegToPos(ARM_TOOL_SERVO1_ID,
                             ARM_TOOL_SERVO1_INIT_DEG);
    g_arm_tool_debug.servo_target_pos[1] =
        ARM_TOOL_SERVO2_NEUTRAL_POS;
    g_arm_tool_debug.tool_ready = 0u;
    g_arm_tool_debug.init_state = initialized != 0u ?
        ARM_TOOL_INIT_SERVO1_PENDING : ARM_TOOL_INIT_ERROR;
    g_arm_tool_debug.last_update_tick = HAL_GetTick();
    if (initialized == 0u) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_TX;
    }
#else
    memset(&g_arm_tool_debug, 0, sizeof(g_arm_tool_debug));
    g_arm_tool_debug.init_state = ARM_TOOL_INIT_DISABLED;
#endif
}

void ArmToolTask(uint32_t now_ms)
{
    HSLServoTask(now_ms);
#if ARM_TOOL_ENABLE != 0u
    switch (g_arm_tool_debug.init_state) {
        case ARM_TOOL_INIT_SERVO1_PENDING:
        {
            Arm_Command_Result_e result;

            if ((uint32_t)(now_ms - g_arm_tool_debug.last_update_tick) <
                ARM_TOOL_SERVO_INIT_START_DELAY_MS) {
                break;
            }
            result = ArmToolSendServo(
                ARM_TOOL_SERVO1_ID, ARM_TOOL_SERVO1_INIT_DEG,
                ARM_TOOL_SERVO_INIT_TIME_MS, 1u);
            if (result == ARM_COMMAND_OK) {
                g_arm_tool_debug.init_result[0] = result;
                g_arm_tool_debug.init_step = 1u;
                g_arm_tool_debug.init_state = ARM_TOOL_INIT_SERVO1_WAIT;
            } else if (result != ARM_COMMAND_BUSY) {
                g_arm_tool_debug.init_result[0] = result;
                g_arm_tool_debug.init_state = ARM_TOOL_INIT_ERROR;
            }
            break;
        }

        case ARM_TOOL_INIT_SERVO1_WAIT:
            if (g_hsl_servo_debug.busy == 0u &&
                g_arm_tool_debug.tx_pending[0] == 0u &&
                (uint32_t)(now_ms - g_arm_tool_debug.last_update_tick) >=
                    ARM_TOOL_SERVO_UPDATE_PERIOD_MS) {
                g_arm_tool_debug.init_state = ARM_TOOL_INIT_SERVO2_PENDING;
            }
            break;

        case ARM_TOOL_INIT_SERVO2_PENDING:
        {
            Arm_Command_Result_e result = ArmToolSendServo(
                ARM_TOOL_SERVO2_ID, ARM_TOOL_SERVO2_FIXED_DEG,
                ARM_TOOL_SERVO_INIT_TIME_MS, 1u);

            if (result == ARM_COMMAND_OK) {
                g_arm_tool_debug.init_result[1] = result;
                g_arm_tool_debug.init_step = 2u;
                g_arm_tool_debug.init_state = ARM_TOOL_INIT_SERVO2_WAIT;
            } else if (result != ARM_COMMAND_BUSY) {
                g_arm_tool_debug.init_result[1] = result;
                g_arm_tool_debug.init_state = ARM_TOOL_INIT_ERROR;
            }
            break;
        }

        case ARM_TOOL_INIT_SERVO2_WAIT:
            if (g_hsl_servo_debug.busy == 0u &&
                g_arm_tool_debug.tx_pending[1] == 0u) {
                g_arm_tool_debug.init_repeat_count++;
                g_arm_tool_debug.tool_ready =
                    g_arm_tool_debug.servo_online[0] != 0u &&
                    g_arm_tool_debug.servo_online[1] != 0u;
                if (g_arm_tool_debug.init_repeat_count <
                    ARM_TOOL_SERVO_INIT_REPEAT_COUNT) {
                    g_arm_tool_debug.last_update_tick = now_ms;
                    g_arm_tool_debug.init_state =
                        ARM_TOOL_INIT_RETRY_WAIT;
                } else {
                    g_arm_tool_debug.init_state =
                        g_arm_tool_debug.tool_ready != 0u ?
                        ARM_TOOL_INIT_DONE : ARM_TOOL_INIT_ERROR;
                }
            }
            break;

        case ARM_TOOL_INIT_RETRY_WAIT:
            if ((uint32_t)(now_ms - g_arm_tool_debug.last_update_tick) >=
                ARM_TOOL_SERVO_INIT_RETRY_PERIOD_MS) {
                g_arm_tool_debug.init_state = ARM_TOOL_INIT_SERVO1_PENDING;
            }
            break;

        case ARM_TOOL_INIT_DONE:
        case ARM_TOOL_INIT_ERROR:
        case ARM_TOOL_INIT_DISABLED:
        default:
            break;
    }

    if (g_arm_tool_debug.init_state == ARM_TOOL_INIT_DONE &&
        g_arm_tool_debug.servo2_repeat_remaining != 0u &&
        (uint32_t)(now_ms - g_arm_tool_debug.servo2_repeat_tick) >=
            ARM_TOOL_SERVO2_REPEAT_PERIOD_MS) {
        g_arm_tool_debug.servo2_repeat_tick = now_ms;
        if (arm_tool_tx_scheduler.valid[1] != 0u) {
            g_arm_tool_debug.tx_pending_overwrite_count[1]++;
        }
        arm_tool_tx_scheduler.valid[1] = 1u;
        arm_tool_tx_scheduler.position[1] =
            g_arm_tool_debug.servo2_repeat_pos;
        arm_tool_tx_scheduler.time_ms[1] =
            g_arm_tool_debug.servo2_repeat_time_ms;
        arm_tool_tx_scheduler.queued_tick[1] = now_ms;
        g_arm_tool_debug.tx_pending[1] = 1u;
        g_arm_tool_debug.tx_pending_pos[1] =
            g_arm_tool_debug.servo2_repeat_pos;
        g_arm_tool_debug.tx_pending_time_ms[1] =
            g_arm_tool_debug.servo2_repeat_time_ms;
        g_arm_tool_debug.servo2_repeat_remaining--;
    }

    if (g_arm_tool_debug.init_state == ARM_TOOL_INIT_DONE &&
        g_arm_tool_debug.servo1_slew_active != 0u &&
        (uint32_t)(now_ms - g_arm_tool_debug.servo1_slew_tick) >=
            ARM_TOOL_SERVO1_SLEW_PERIOD_MS) {
        float current_deg = g_arm_tool_debug.servo_target_deg[0];
        float target_deg =
            g_arm_tool_debug.servo1_compensation_target_deg;
        float delta_deg = target_deg - current_deg;
#if ARM_TOOL_SERVO1_FULL_SPEED_TRACK_ENABLE == 0u
        float elapsed_s =
            (float)(now_ms - g_arm_tool_debug.servo1_slew_tick) / 1000.0f;
        float max_step_deg = ARM_TOOL_SERVO1_SLEW_RATE_DEG_S * elapsed_s;
#endif
        float next_deg;

        if (fabsf(delta_deg) <= ARM_TOOL_SERVO_COMMAND_DEADBAND_DEG) {
            g_arm_tool_debug.servo1_slew_active = 0u;
        } else {
#if ARM_TOOL_SERVO1_FULL_SPEED_TRACK_ENABLE != 0u
            /* 全速模式不做软件角速度斜坡，直接发送最新补偿目标。 */
            next_deg = target_deg;
#else
            if (delta_deg > max_step_deg) {
                next_deg = current_deg + max_step_deg;
            } else if (delta_deg < -max_step_deg) {
                next_deg = current_deg - max_step_deg;
            } else {
                next_deg = target_deg;
            }
#endif
            if (ArmToolSendServo(ARM_TOOL_SERVO1_ID, next_deg,
                                 ARM_TOOL_SERVO_TRACK_TIME_MS, 1u) ==
                ARM_COMMAND_OK) {
                g_arm_tool_debug.servo1_slew_tick = now_ms;
                if (fabsf(target_deg - next_deg) <=
                    ARM_TOOL_SERVO_COMMAND_DEADBAND_DEG) {
                    g_arm_tool_debug.servo1_slew_active = 0u;
                }
            }
        }
    }

    /*
     * 先收集本周期ID1竖直补偿、ID2底座/yaw补偿的最新目标，
     * 再统一发送；两槽同时有效且运动时间一致时自动合成一帧。
     */
    ArmToolDispatchPending(now_ms);

#else
    (void)now_ms;
#endif
}

void ArmToolSetMagnet(uint8_t on)
{
#if ARM_TOOL_ENABLE != 0u
    HAL_GPIO_WritePin(ARM_MAGNET_GPIO_PORT, ARM_MAGNET_GPIO_PIN,
        on != 0u ? ARM_MAGNET_ACTIVE_LEVEL : ARM_MAGNET_INACTIVE_LEVEL);
    g_arm_tool_debug.magnet_on = on != 0u ? 1u : 0u;
#else
    (void)on;
#endif
}

void ArmToolStopServo1Tracking(void)
{
#if ARM_TOOL_ENABLE != 0u
    g_arm_tool_debug.servo1_compensation_target_deg =
        g_arm_tool_debug.servo_target_deg[0];
    g_arm_tool_debug.servo1_slew_active = 0u;
    ArmToolClearPending(0u);
#endif
}

void ArmToolClearPendingCommands(void)
{
#if ARM_TOOL_ENABLE != 0u
    ArmToolClearPending(0u);
    ArmToolClearPending(1u);
    g_arm_tool_debug.servo1_slew_active = 0u;
    g_arm_tool_debug.servo2_repeat_remaining = 0u;
#endif
}

uint8_t ArmToolTxIdle(void)
{
#if ARM_TOOL_ENABLE != 0u
    return g_hsl_servo_debug.busy == 0u &&
           arm_tool_tx_scheduler.valid[0] == 0u &&
           arm_tool_tx_scheduler.valid[1] == 0u;
#else
    return 1u;
#endif
}

Arm_Command_Result_e ArmToolSetServo1Angle(float angle_deg)
{
    g_arm_tool_debug.vertical_down_enabled = 0u;
    g_arm_tool_debug.servo1_slew_active = 0u;
    return ArmToolSendServo(ARM_TOOL_SERVO1_ID, angle_deg,
                            ARM_TOOL_SERVO_TRACK_TIME_MS, 1u);
}

Arm_Command_Result_e ArmToolSetServo2Angle(float angle_deg)
{
    if (!ArmToolServoAngleFiniteAndInRange(ARM_TOOL_SERVO2_ID, angle_deg)) {
        return ARM_COMMAND_INVALID;
    }
    return ArmToolSendServo(ARM_TOOL_SERVO2_ID, angle_deg,
                            ARM_USB_YAW_MOVE_TIME_MS, 1u);
}

Arm_Command_Result_e ArmToolSetServo2Position(uint16_t position,
                                              uint16_t time_ms)
{
    float max_yaw_deg;
    float half_range_pos;
    uint32_t now_ms;

#if ARM_TOOL_ENABLE == 0u
    (void)position;
    (void)time_ms;
    return ARM_COMMAND_UNSUPPORTED;
#else
    if (g_arm_tool_debug.initialized == 0u) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_INVALID_ARGUMENT;
        return ARM_COMMAND_NOT_READY;
    }
    if (position > ARM_TOOL_SERVO2_POS_MAX ||
        time_ms > HSL_SERVO_MAX_TIME_MS) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_RANGE;
        return ARM_COMMAND_INVALID;
    }
    max_yaw_deg = ArmToolServo2MaxYawAbsDeg();
    half_range_pos = ArmToolServo2HalfRangePos();
    if (max_yaw_deg <= 0.000001f ||
        half_range_pos <= 0.000001f ||
        fabsf(ARM_TOOL_SERVO2_YAW_DIRECTION) <= 0.000001f) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_RANGE;
        return ARM_COMMAND_INVALID;
    }
    now_ms = HAL_GetTick();
    if (arm_tool_tx_scheduler.valid[1] != 0u) {
        g_arm_tool_debug.tx_pending_overwrite_count[1]++;
    }
    arm_tool_tx_scheduler.valid[1] = 1u;
    arm_tool_tx_scheduler.position[1] = position;
    arm_tool_tx_scheduler.time_ms[1] = time_ms;
    arm_tool_tx_scheduler.queued_tick[1] = now_ms;
    g_arm_tool_debug.tx_pending[1] = 1u;
    g_arm_tool_debug.tx_pending_pos[1] = position;
    g_arm_tool_debug.tx_pending_time_ms[1] = time_ms;
    g_arm_tool_debug.servo_target_deg[1] =
        ARM_USB_YAW_NEUTRAL_DEG +
        ((float)position - (float)ARM_TOOL_SERVO2_NEUTRAL_POS) *
        ARM_TOOL_SERVO2_RANGE_DEG /
        half_range_pos /
        ARM_TOOL_SERVO2_YAW_DIRECTION;
    g_arm_tool_debug.servo_target_pos[1] = position;
    g_arm_tool_debug.last_update_tick = now_ms;
    g_arm_tool_debug.servo2_repeat_pos = position;
    g_arm_tool_debug.servo2_repeat_time_ms = time_ms;
    g_arm_tool_debug.servo2_repeat_tick = g_arm_tool_debug.last_update_tick;
    g_arm_tool_debug.servo2_repeat_remaining =
        ARM_TOOL_SERVO2_REPEAT_COUNT > 0u ?
        (uint8_t)(ARM_TOOL_SERVO2_REPEAT_COUNT - 1u) : 0u;
    return ARM_COMMAND_OK;
#endif
}

Arm_Command_Result_e ArmToolSetServo2WorldYawTarget(float world_yaw_deg)
{
#if ARM_TOOL_ENABLE == 0u
    (void)world_yaw_deg;
    return ARM_COMMAND_UNSUPPORTED;
#else
    if (!isfinite(world_yaw_deg) ||
        world_yaw_deg < ARM_USB_YAW_MIN_DEG ||
        world_yaw_deg > ARM_USB_YAW_MAX_DEG) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_RANGE;
        return ARM_COMMAND_INVALID;
    }
    g_arm_tool_debug.servo2_world_yaw_target_deg = world_yaw_deg;
    /* 旧目标的补发不得覆盖新的q1动态补偿目标。 */
    g_arm_tool_debug.servo2_repeat_remaining = 0u;
    return ARM_COMMAND_OK;
#endif
}

uint8_t ArmToolServo2WorldYawValidForQ1(float world_yaw_deg,
                                        float q1_deg)
{
    float compensation_deg;
    float host_yaw_deg;
    float logic_deg;

    if (!isfinite(world_yaw_deg) || !isfinite(q1_deg)) {
        return 0u;
    }
    compensation_deg = ARM_TOOL_SERVO2_BASE_COMPENSATION_ENABLE != 0u ?
        -ARM_TOOL_SERVO2_BASE_COMPENSATION_SCALE * q1_deg : 0.0f;
    host_yaw_deg = ARM_TOOL_SERVO2_APPLY_HOST_YAW_GAIN(world_yaw_deg);
    logic_deg = ARM_TOOL_SERVO_NEUTRAL_DEG + host_yaw_deg +
        compensation_deg;
    return ArmToolServoAngleFiniteAndInRange(ARM_TOOL_SERVO2_ID,
                                              logic_deg);
}

Arm_Command_Result_e ArmToolTrackServo2WorldYaw(float q1_feedback_deg,
                                                uint32_t now_ms)
{
#if ARM_TOOL_ENABLE == 0u
    (void)q1_feedback_deg;
    (void)now_ms;
    return ARM_COMMAND_UNSUPPORTED;
#else
    float compensation_deg;
    float host_yaw_deg;
    float relative_deg;
    float logic_deg;
    Arm_Command_Result_e result;

    if (g_arm_tool_debug.init_state != ARM_TOOL_INIT_DONE ||
        g_arm_tool_debug.servo2_base_compensation_enabled == 0u) {
        return ARM_COMMAND_NOT_READY;
    }
    if (!isfinite(q1_feedback_deg)) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_INVALID_ARGUMENT;
        return ARM_COMMAND_INVALID;
    }

    compensation_deg = -ARM_TOOL_SERVO2_BASE_COMPENSATION_SCALE *
        q1_feedback_deg;
    host_yaw_deg = ARM_TOOL_SERVO2_APPLY_HOST_YAW_GAIN(
        g_arm_tool_debug.servo2_world_yaw_target_deg);
    relative_deg = host_yaw_deg +
        compensation_deg;
    logic_deg = ARM_TOOL_SERVO_NEUTRAL_DEG + relative_deg;

    g_arm_tool_debug.servo2_q1_feedback_deg = q1_feedback_deg;
    g_arm_tool_debug.servo2_base_compensation_deg = compensation_deg;
    g_arm_tool_debug.servo2_relative_target_deg = relative_deg;
    g_arm_tool_debug.servo2_logic_target_deg = logic_deg;

    if (!ArmToolServoAngleFiniteAndInRange(ARM_TOOL_SERVO2_ID, logic_deg)) {
        if (g_arm_tool_debug.servo2_target_in_range != 0u) {
            g_arm_tool_debug.servo2_limit_reject_count++;
        }
        g_arm_tool_debug.servo2_target_in_range = 0u;
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_RANGE;
        return ARM_COMMAND_INVALID;
    }
    g_arm_tool_debug.servo2_target_in_range = 1u;
    if (g_arm_tool_debug.error_code == ARM_TOOL_ERROR_SERVO_RANGE) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_NONE;
    }
    g_arm_tool_debug.servo2_compensated_target_pos =
        ArmToolAngleDegToPos(ARM_TOOL_SERVO2_ID, logic_deg);

    if ((uint32_t)(now_ms - g_arm_tool_debug.servo2_tracking_tick) <
        ARM_TOOL_SERVO2_TRACK_UPDATE_PERIOD_MS ||
        fabsf(logic_deg - g_arm_tool_debug.servo_target_deg[1]) <
            ARM_TOOL_SERVO2_TRACK_DEADBAND_DEG) {
        return ARM_COMMAND_OK;
    }

    result = ArmToolSendServo(ARM_TOOL_SERVO2_ID, logic_deg,
                              ARM_TOOL_SERVO2_TRACK_TIME_MS, 1u);
    if (result == ARM_COMMAND_OK) {
        g_arm_tool_debug.servo2_tracking_tick = now_ms;
        g_arm_tool_debug.servo2_compensation_tx_count++;
    }
    return result;
#endif
}

float ArmToolGetServo2WorldYawTarget(void)
{
    return g_arm_tool_debug.servo2_world_yaw_target_deg;
}

Arm_Command_Result_e ArmToolSetVerticalDownFromPitch(
    float small_link_pitch_deg)
{
    float servo_deg = ArmToolServo1AngleForVerticalDown(
        small_link_pitch_deg);

    if (!ArmToolServo1AngleValid(servo_deg)) {
        g_arm_tool_debug.tool_ready = 0u;
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_RANGE;
        return ARM_COMMAND_PREFLIGHT_FAILED;
    }
    g_arm_tool_debug.vertical_down_enabled =
        ARM_TOOL_VERTICAL_COMPENSATION_ENABLE != 0u ? 1u : 0u;
    g_arm_tool_debug.tool_arm_pitch_deg =
        ARM_TOOL_VERTICAL_COMPENSATION_ENABLE != 0u ?
            0.0f : small_link_pitch_deg;
    g_arm_tool_debug.servo1_compensation_target_deg = servo_deg;
    if (fabsf(servo_deg - g_arm_tool_debug.servo_target_deg[0]) >
        ARM_TOOL_SERVO_COMMAND_DEADBAND_DEG) {
        if (g_arm_tool_debug.servo1_slew_active == 0u) {
            g_arm_tool_debug.servo1_slew_tick = HAL_GetTick();
        }
        g_arm_tool_debug.servo1_slew_active = 1u;
    }
    return ARM_COMMAND_OK;
}

uint8_t ArmToolGetTipFromWrist(const Arm_Position_s *wrist,
                               float base_yaw_deg,
                               float tool_arm_pitch_deg,
                               Arm_Position_s *tip)
{
    float yaw_rad;
    float pitch_rad;
    float radial_offset;
    float z_offset;

    if (wrist == NULL || tip == NULL ||
        !isfinite(wrist->x_mm) || !isfinite(wrist->y_mm) ||
        !isfinite(wrist->z_mm) || !isfinite(base_yaw_deg) ||
        !isfinite(tool_arm_pitch_deg)) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_INVALID_ARGUMENT;
        return 0u;
    }

    yaw_rad = base_yaw_deg * ARM_TOOL_DEG_TO_RAD;
    pitch_rad = tool_arm_pitch_deg * ARM_TOOL_DEG_TO_RAD;
    radial_offset = ARM_TOOL_SERVO1_ARM_LENGTH_MM * cosf(pitch_rad) +
        ARM_TOOL_MAGNET_OFFSET_MM * cosf(pitch_rad - ARM_TOOL_PI / 2.0f);
    z_offset = ARM_TOOL_SERVO1_ARM_LENGTH_MM * sinf(pitch_rad) +
        ARM_TOOL_MAGNET_OFFSET_MM * sinf(pitch_rad - ARM_TOOL_PI / 2.0f);

    tip->x_mm = wrist->x_mm + radial_offset * cosf(yaw_rad);
    tip->y_mm = wrist->y_mm + radial_offset * sinf(yaw_rad);
    tip->z_mm = wrist->z_mm + z_offset;
    g_arm_tool_debug.wrist_center_mm = *wrist;
    g_arm_tool_debug.tool_tip_mm = *tip;
    g_arm_tool_debug.tool_arm_pitch_deg = tool_arm_pitch_deg;
    return 1u;
}

uint8_t ArmToolGetWristFromTipVerticalDown(const Arm_Position_s *tip,
                                           Arm_Position_s *wrist)
{
    float radial;
    float wrist_radial;

    if (tip == NULL || wrist == NULL ||
        !isfinite(tip->x_mm) || !isfinite(tip->y_mm) ||
        !isfinite(tip->z_mm)) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_INVALID_ARGUMENT;
        return 0u;
    }

    radial = sqrtf(tip->x_mm * tip->x_mm + tip->y_mm * tip->y_mm);
    if (!isfinite(radial) ||
        radial <= ARM_TOOL_SERVO1_ARM_LENGTH_MM) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_GEOMETRY;
        return 0u;
    }

    wrist_radial = radial - ARM_TOOL_SERVO1_ARM_LENGTH_MM;
    wrist->x_mm = tip->x_mm * wrist_radial / radial;
    wrist->y_mm = tip->y_mm * wrist_radial / radial;
    wrist->z_mm = tip->z_mm + ARM_TOOL_MAGNET_OFFSET_MM;
    if (!isfinite(wrist->x_mm) || !isfinite(wrist->y_mm) ||
        !isfinite(wrist->z_mm)) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_GEOMETRY;
        return 0u;
    }
    return 1u;
}

const Arm_Tool_State_s *ArmToolGetState(void)
{
    return &g_arm_tool_debug;
}
