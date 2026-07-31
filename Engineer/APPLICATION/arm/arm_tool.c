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

static uint8_t ArmToolServoAngleFiniteAndInRange(float angle_deg)
{
    return isfinite(angle_deg) &&
           angle_deg >= ARM_TOOL_SERVO_DEG_MIN &&
           angle_deg <= ARM_TOOL_SERVO_DEG_MAX;
}

static Arm_Command_Result_e ArmToolSendServo(uint8_t servo_id,
                                             float angle_deg,
                                             uint16_t time_ms,
                                             uint8_t force_send)
{
    uint8_t index = ArmToolServoIndex(servo_id);
    uint16_t pos;
    HSLServo_Result_e result;
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
        !ArmToolServoAngleFiniteAndInRange(angle_deg) ||
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
    result = HSLServoMove(servo_id, pos, time_ms);
    if (result == HSL_SERVO_RESULT_BUSY) {
        return ARM_COMMAND_BUSY;
    }
    if (result != HSL_SERVO_RESULT_OK) {
        g_arm_tool_debug.tx_fail_count[index]++;
        g_arm_tool_debug.servo_online[index] = 0u;
        g_arm_tool_debug.tool_ready = 0u;
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_TX;
        return ARM_COMMAND_NOT_READY;
    }

    g_arm_tool_debug.tx_count[index]++;
    g_arm_tool_debug.servo_online[index] = 1u;
    g_arm_tool_debug.servo_target_deg[index] = angle_deg;
    g_arm_tool_debug.servo_target_pos[index] = pos;
    g_arm_tool_debug.last_update_tick = now_ms;
    g_arm_tool_debug.tool_ready =
        g_arm_tool_debug.servo_online[0] != 0u &&
        g_arm_tool_debug.servo_online[1] != 0u;
    g_arm_tool_debug.error_code = ARM_TOOL_ERROR_NONE;
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

    if (!isfinite(angle_deg) ||
        (servo_id != ARM_TOOL_SERVO1_ID &&
         servo_id != ARM_TOOL_SERVO2_ID)) {
        return ARM_TOOL_SERVO_POS_MIN;
    }
    if (angle_deg < ARM_TOOL_SERVO_DEG_MIN) {
        angle_deg = ARM_TOOL_SERVO_DEG_MIN;
    }
    if (angle_deg > ARM_TOOL_SERVO_DEG_MAX) {
        angle_deg = ARM_TOOL_SERVO_DEG_MAX;
    }
    neutral_pos = servo_id == ARM_TOOL_SERVO1_ID ?
        (float)ARM_TOOL_SERVO1_NEUTRAL_POS :
        (float)ARM_TOOL_SERVO2_NEUTRAL_POS;
    pos_min = servo_id == ARM_TOOL_SERVO2_ID ?
        (float)ARM_TOOL_SERVO2_POS_MIN : (float)ARM_TOOL_SERVO_POS_MIN;
    pos_max = servo_id == ARM_TOOL_SERVO2_ID ?
        (float)ARM_TOOL_SERVO2_POS_MAX : (float)ARM_TOOL_SERVO_POS_MAX;
    pos_per_deg = (pos_max - pos_min) /
        (ARM_TOOL_SERVO_DEG_MAX - ARM_TOOL_SERVO_DEG_MIN);
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
    return ArmToolServoAngleFiniteAndInRange(angle_deg);
}

void ArmToolInit(void)
{
#if ARM_TOOL_ENABLE != 0u
    uint8_t initialized;

    memset(&g_arm_tool_debug, 0, sizeof(g_arm_tool_debug));
    initialized = HSLServoInit();
    HAL_GPIO_WritePin(ARM_MAGNET_GPIO_PORT, ARM_MAGNET_GPIO_PIN,
                      ARM_MAGNET_INACTIVE_LEVEL);
    g_arm_tool_debug.initialized = initialized;
    g_arm_tool_debug.magnet_on = 0u;
    g_arm_tool_debug.vertical_down_enabled =
        ARM_TOOL_VERTICAL_COMPENSATION_ENABLE != 0u ? 1u : 0u;
    g_arm_tool_debug.servo_target_deg[0] = ARM_TOOL_SERVO1_INIT_DEG;
    g_arm_tool_debug.servo_target_deg[1] = ARM_TOOL_SERVO2_FIXED_DEG;
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
            if (g_hsl_servo_debug.busy == 0u) {
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
        g_hsl_servo_debug.busy == 0u &&
        (uint32_t)(now_ms - g_arm_tool_debug.servo2_repeat_tick) >=
            ARM_TOOL_SERVO2_REPEAT_PERIOD_MS) {
        HSLServo_Result_e result;

        result = HSLServoMove(
            ARM_TOOL_SERVO2_ID,
            g_arm_tool_debug.servo2_repeat_pos,
            g_arm_tool_debug.servo2_repeat_time_ms);

        g_arm_tool_debug.servo2_repeat_tick = now_ms;
        if (result == HSL_SERVO_RESULT_OK) {
            g_arm_tool_debug.tx_count[1]++;
            g_arm_tool_debug.servo_online[1] = 1u;
            g_arm_tool_debug.servo2_repeat_remaining--;
            g_arm_tool_debug.error_code = ARM_TOOL_ERROR_NONE;
        } else if (result != HSL_SERVO_RESULT_BUSY) {
            g_arm_tool_debug.tx_fail_count[1]++;
            g_arm_tool_debug.servo_online[1] = 0u;
            g_arm_tool_debug.tool_ready = 0u;
            g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_TX;
            g_arm_tool_debug.servo2_repeat_remaining = 0u;
        }
    }

    if (g_arm_tool_debug.init_state == ARM_TOOL_INIT_DONE &&
        g_arm_tool_debug.servo1_slew_active != 0u &&
        g_hsl_servo_debug.busy == 0u &&
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
    if (!ArmToolServoAngleFiniteAndInRange(angle_deg)) {
        return ARM_COMMAND_INVALID;
    }
    return ArmToolSendServo(ARM_TOOL_SERVO2_ID, angle_deg,
                            ARM_USB_YAW_MOVE_TIME_MS, 1u);
}

Arm_Command_Result_e ArmToolSetServo2Position(uint16_t position,
                                              uint16_t time_ms)
{
    HSLServo_Result_e result;
    float max_yaw_deg;
    float half_range_pos;

#if ARM_TOOL_ENABLE == 0u
    (void)position;
    (void)time_ms;
    return ARM_COMMAND_UNSUPPORTED;
#else
    if (g_arm_tool_debug.initialized == 0u) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_INVALID_ARGUMENT;
        return ARM_COMMAND_NOT_READY;
    }
    if (position < ARM_TOOL_SERVO2_POS_MIN ||
        position > ARM_TOOL_SERVO2_POS_MAX ||
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
    result = HSLServoMove(ARM_TOOL_SERVO2_ID, position, time_ms);
    if (result == HSL_SERVO_RESULT_BUSY) {
        return ARM_COMMAND_BUSY;
    }
    if (result != HSL_SERVO_RESULT_OK) {
        g_arm_tool_debug.tx_fail_count[1]++;
        g_arm_tool_debug.servo_online[1] = 0u;
        g_arm_tool_debug.tool_ready = 0u;
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_TX;
        return ARM_COMMAND_NOT_READY;
    }

    g_arm_tool_debug.tx_count[1]++;
    g_arm_tool_debug.servo_online[1] = 1u;
    g_arm_tool_debug.servo_target_deg[1] =
        ARM_USB_YAW_NEUTRAL_DEG +
        ((float)position - (float)ARM_TOOL_SERVO2_NEUTRAL_POS) *
        (ARM_TOOL_SERVO_DEG_MAX - ARM_TOOL_SERVO_DEG_MIN) /
        half_range_pos /
        ARM_TOOL_SERVO2_YAW_DIRECTION;
    g_arm_tool_debug.servo_target_pos[1] = position;
    g_arm_tool_debug.last_update_tick = HAL_GetTick();
    g_arm_tool_debug.servo2_repeat_pos = position;
    g_arm_tool_debug.servo2_repeat_time_ms = time_ms;
    g_arm_tool_debug.servo2_repeat_tick = g_arm_tool_debug.last_update_tick;
    g_arm_tool_debug.servo2_repeat_remaining =
        ARM_TOOL_SERVO2_REPEAT_COUNT > 0u ?
        (uint8_t)(ARM_TOOL_SERVO2_REPEAT_COUNT - 1u) : 0u;
    g_arm_tool_debug.tool_ready =
        g_arm_tool_debug.servo_online[0] != 0u &&
        g_arm_tool_debug.servo_online[1] != 0u;
    g_arm_tool_debug.error_code = ARM_TOOL_ERROR_NONE;
    return ARM_COMMAND_OK;
#endif
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
