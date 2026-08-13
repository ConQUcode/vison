/**
 * @file arm_tool.c
 * @brief 驱动两台幻儿舵机，并根据反馈推进俯仰和夹爪闭环状态机。
 */

#include "arm_tool.h"

#include "arm_config.h"
#include "huaner_servo.h"
#include "stm32f4xx_hal.h"

#include <math.h>
#include <string.h>

#if ARM_TOOL_ENABLE == 0u
/* 关闭末端工具时，这些状态机入口保留给后续恢复联调，但不会进入运行链路。 */
#define ARM_TOOL_DISABLED_UNUSED __attribute__((unused))
#else
#define ARM_TOOL_DISABLED_UNUSED
#endif

#define ARM_TOOL_PI         3.14159265358979323846f
#define ARM_TOOL_DEG_TO_RAD (ARM_TOOL_PI / 180.0f)

typedef struct {
    uint8_t valid[2];
    uint16_t position[2];
    uint16_t time_ms[2];
    uint32_t queued_tick[2];
    uint8_t next_single_index;
} Arm_Tool_Tx_Scheduler_s;

typedef enum {
    ARM_GRIPPER_RELIEF_NONE = 0,
    ARM_GRIPPER_RELIEF_CONTACT,
    ARM_GRIPPER_RELIEF_JAM,
    ARM_GRIPPER_RELIEF_BOOT,
    ARM_GRIPPER_RELIEF_TIMEOUT
} Arm_Gripper_Relief_Outcome_e;

Arm_Tool_State_s g_arm_tool_debug;
Arm_Servo_Angle_Debug_s g_arm_servo_angle_debug;
Arm_Gripper_Stall_Debug_s g_arm_gripper_stall_debug;

static Arm_Tool_Tx_Scheduler_s arm_tool_tx_scheduler;
static uint8_t arm_tool_feedback_only_mode;
static uint8_t arm_tool_feedback_unload_in_progress;
static Arm_Gripper_Relief_Outcome_e arm_gripper_relief_outcome;
static uint8_t ArmToolPositionInRange(uint8_t servo_id,
                                      uint16_t position);

static uint8_t ArmToolNear(float a, float b, float tolerance)
{
    return isfinite(a) && isfinite(b) && fabsf(a - b) <= tolerance;
}

static float ArmToolServoPositionToDeg(uint16_t position)
{
    return ((float)position - (float)ARM_TOOL_SERVO_POS_MIN) *
        ARM_TOOL_SERVO_RANGE_DEG /
        (float)(ARM_TOOL_SERVO_POS_MAX - ARM_TOOL_SERVO_POS_MIN);
}

/** 只刷新ID2堵转专项Watch，避免查看完整工具状态结构。 */
static void ArmToolUpdateGripperStallDebug(void)
{
    g_arm_gripper_stall_debug.stall_detected =
        g_arm_tool_debug.gripper_stall_latched;
    g_arm_gripper_stall_debug.relief_attempt_count =
        g_arm_tool_debug.gripper_relief_attempt_count;
    g_arm_gripper_stall_debug.next_stage_ready =
        g_arm_tool_debug.gripper_state == ARM_GRIPPER_READY ||
        g_arm_tool_debug.gripper_state == ARM_GRIPPER_OPEN ||
        g_arm_tool_debug.gripper_state == ARM_GRIPPER_HELD_CONTACT ||
        g_arm_tool_debug.gripper_state == ARM_GRIPPER_CLOSED_EMPTY ||
        g_arm_tool_debug.gripper_state == ARM_GRIPPER_FORCED_HELD;
    g_arm_gripper_stall_debug.target_deg =
        ArmToolServoPositionToDeg(g_arm_tool_debug.gripper_target_pos);
    g_arm_gripper_stall_debug.current_deg =
        g_arm_tool_debug.servo_feedback_valid[1] != 0u ?
        ArmToolServoPositionToDeg(g_arm_tool_debug.gripper_feedback_pos) :
        NAN;
}

static uint8_t ARM_TOOL_DISABLED_UNUSED ArmToolSelfTest(void)
{
    Arm_Position_s wrist = {100.0f, 200.0f, 300.0f};
    Arm_Position_s center;
    Arm_Position_s recovered;
    uint16_t position = 0u;
    uint32_t fail_mask = 0u;

    /* 当前实机方向：相对俯仰-90deg对应875，+90deg对应125。 */
    if (!ArmToolPitchPositionForPose(-90.0f, 0.0f, &position) ||
        position != 875u) {
        fail_mask |= 1u << 0;
    }
    if (!ArmToolPitchPositionForPose(0.0f, 0.0f, &position) ||
        position != 500u) {
        fail_mask |= 1u << 1;
    }
    if (!ArmToolPitchPositionForPose(90.0f, 0.0f, &position) ||
        position != 125u) {
        fail_mask |= 1u << 2;
    }
    if (ArmToolPitchPositionForPose(-90.24f, 0.0f, &position) ||
        ArmToolPitchPositionForPose(90.24f, 0.0f, &position)) {
        fail_mask |= 1u << 3;
    }
    if (!ArmToolPositionInRange(ARM_GRIPPER_SERVO_ID,
                                ARM_GRIPPER_DEFAULT_POS) ||
        !ArmToolPositionInRange(ARM_GRIPPER_SERVO_ID,
                                ARM_GRIPPER_CLOSE_POS) ||
        ArmToolPositionInRange(ARM_GRIPPER_SERVO_ID,
                               ARM_GRIPPER_SERVO_MIN_POS - 1u) ||
        ArmToolPositionInRange(ARM_GRIPPER_SERVO_ID,
                               ARM_GRIPPER_SERVO_MAX_POS + 1u)) {
        fail_mask |= 1u << 4;
    }
    if (!ArmToolGetCenterFromWrist(&wrist, 0.0f, 0.0f, &center) ||
        !ArmToolNear(center.x_mm,
                     100.0f + ARM_TOOL_PITCH_AXIS_TO_CENTER_MM, 0.001f) ||
        !ArmToolNear(center.y_mm, 200.0f, 0.001f) ||
        !ArmToolNear(center.z_mm, 300.0f, 0.001f)) {
        fail_mask |= 1u << 5;
    }
    if (!ArmToolGetCenterFromWrist(&wrist, 0.0f, -90.0f, &center) ||
        !ArmToolNear(center.x_mm, 100.0f, 0.001f) ||
        !ArmToolNear(center.z_mm,
                     300.0f - ARM_TOOL_PITCH_AXIS_TO_CENTER_MM, 0.001f)) {
        fail_mask |= 1u << 6;
    }
    if (!ArmToolGetCenterFromWrist(&wrist, 0.0f, 90.0f, &center) ||
        !ArmToolNear(center.x_mm, 100.0f, 0.001f) ||
        !ArmToolNear(center.z_mm,
                     300.0f + ARM_TOOL_PITCH_AXIS_TO_CENTER_MM, 0.001f)) {
        fail_mask |= 1u << 7;
    }
    if (!ArmToolGetCenterFromWrist(&wrist, 90.0f, 0.0f, &center) ||
        !ArmToolNear(center.x_mm, 100.0f, 0.001f) ||
        !ArmToolNear(center.y_mm,
                     200.0f + ARM_TOOL_PITCH_AXIS_TO_CENTER_MM, 0.001f) ||
        !ArmToolGetWristFromCenter(&center, 90.0f, 0.0f, &recovered) ||
        !ArmToolNear(recovered.x_mm, wrist.x_mm, 0.001f) ||
        !ArmToolNear(recovered.y_mm, wrist.y_mm, 0.001f) ||
        !ArmToolNear(recovered.z_mm, wrist.z_mm, 0.001f)) {
        fail_mask |= 1u << 8;
    }

    g_arm_tool_debug.self_test_fail_mask = fail_mask;
    g_arm_tool_debug.self_test_passed = fail_mask == 0u ? 1u : 0u;
    return g_arm_tool_debug.self_test_passed;
}

static uint8_t ArmToolServoIndex(uint8_t servo_id)
{
    return servo_id == ARM_TOOL_PITCH_SERVO_ID ? 0u : 1u;
}

static uint8_t ArmToolPositionInRange(uint8_t servo_id, uint16_t position)
{
    if (servo_id == ARM_TOOL_PITCH_SERVO_ID) {
        return position >= ARM_TOOL_PITCH_SERVO_MIN_POS &&
               position <= ARM_TOOL_PITCH_SERVO_MAX_POS;
    }
    if (servo_id == ARM_GRIPPER_SERVO_ID) {
        return position >= ARM_GRIPPER_SERVO_MIN_POS &&
               position <= ARM_GRIPPER_SERVO_MAX_POS;
    }
    return 0u;
}

static void ArmToolClearPending(uint8_t index)
{
    arm_tool_tx_scheduler.valid[index] = 0u;
    g_arm_tool_debug.tx_pending[index] = 0u;
}

static void ArmToolRecordTxFailure(uint8_t index)
{
    g_arm_tool_debug.tx_fail_count[index]++;
    ArmToolClearPending(index);
    g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_TX;
}

static Arm_Command_Result_e ArmToolQueuePosition(uint8_t servo_id,
                                                  uint16_t position,
                                                  uint16_t time_ms,
                                                  uint8_t force_send)
{
    uint8_t index;
    uint32_t now_ms;

    if (g_arm_tool_debug.initialized == 0u) {
        return ARM_COMMAND_NOT_READY;
    }
    if (!ArmToolPositionInRange(servo_id, position) ||
        time_ms > HUANER_SERVO_MAX_TIME_MS) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_RANGE;
        return ARM_COMMAND_INVALID;
    }

    index = ArmToolServoIndex(servo_id);
    now_ms = HAL_GetTick();
    if (force_send == 0u) {
        uint16_t old_position = g_arm_tool_debug.servo_target_pos[index];
        uint16_t delta = old_position > position ?
            (uint16_t)(old_position - position) :
            (uint16_t)(position - old_position);

        if ((uint32_t)(now_ms - g_arm_tool_debug.tool_pitch_update_tick) <
                ARM_TOOL_PITCH_UPDATE_PERIOD_MS ||
            delta < ARM_TOOL_PITCH_COMMAND_DEADBAND_POS) {
            return ARM_COMMAND_OK;
        }
    }

    if (arm_tool_tx_scheduler.valid[index] != 0u) {
        g_arm_tool_debug.tx_pending_overwrite_count[index]++;
    }
    arm_tool_tx_scheduler.valid[index] = 1u;
    arm_tool_tx_scheduler.position[index] = position;
    arm_tool_tx_scheduler.time_ms[index] = time_ms;
    arm_tool_tx_scheduler.queued_tick[index] = now_ms;
    g_arm_tool_debug.tx_pending[index] = 1u;
    g_arm_tool_debug.tx_pending_pos[index] = position;
    g_arm_tool_debug.tx_pending_time_ms[index] = time_ms;
    g_arm_tool_debug.servo_target_pos[index] = position;
    /* 新目标尚未发送/到位，禁止沿用上一目标留下的arrived=1。 */
    g_arm_tool_debug.servo_arrived[index] = 0u;
    g_arm_tool_debug.servo_motion_timeout[index] = 0u;
    g_arm_tool_debug.last_update_tick = now_ms;
    if (servo_id == ARM_TOOL_PITCH_SERVO_ID) {
        g_arm_tool_debug.tool_pitch_servo_pos = position;
        g_arm_tool_debug.tool_pitch_update_tick = now_ms;
    } else {
        g_arm_tool_debug.gripper_target_pos = position;
    }
    return ARM_COMMAND_OK;
}

static void ARM_TOOL_DISABLED_UNUSED ArmToolDispatchPending(void)
{
    HuanerServo_Result_e result;
    uint8_t index;

    if (g_huaner_servo_driver_debug.busy != 0u ||
        (arm_tool_tx_scheduler.valid[0] == 0u &&
         arm_tool_tx_scheduler.valid[1] == 0u)) {
        return;
    }

    if (arm_tool_tx_scheduler.valid[0] != 0u &&
        arm_tool_tx_scheduler.valid[1] != 0u &&
        arm_tool_tx_scheduler.time_ms[0] ==
            arm_tool_tx_scheduler.time_ms[1]) {
        result = HuanerServoMove2(
            ARM_TOOL_PITCH_SERVO_ID, arm_tool_tx_scheduler.position[0],
            ARM_GRIPPER_SERVO_ID, arm_tool_tx_scheduler.position[1],
            arm_tool_tx_scheduler.time_ms[0]);
        if (result == HUANER_SERVO_RESULT_BUSY) {
            return;
        }
        if (result == HUANER_SERVO_RESULT_OK) {
            g_arm_tool_debug.tx_count[0]++;
            g_arm_tool_debug.tx_count[1]++;
            g_arm_tool_debug.servo_arrived[0] = 0u;
            g_arm_tool_debug.servo_arrived[1] = 0u;
            ArmToolClearPending(0u);
            ArmToolClearPending(1u);
            g_arm_tool_debug.tx_dual_frame_count++;
        } else {
            ArmToolRecordTxFailure(0u);
            ArmToolRecordTxFailure(1u);
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

    result = HuanerServoMove(
        index == 0u ? ARM_TOOL_PITCH_SERVO_ID : ARM_GRIPPER_SERVO_ID,
        arm_tool_tx_scheduler.position[index],
        arm_tool_tx_scheduler.time_ms[index]);
    if (result == HUANER_SERVO_RESULT_BUSY) {
        return;
    }
    if (result == HUANER_SERVO_RESULT_OK) {
        g_arm_tool_debug.tx_count[index]++;
        g_arm_tool_debug.servo_arrived[index] = 0u;
        ArmToolClearPending(index);
        arm_tool_tx_scheduler.next_single_index = index == 0u ? 1u : 0u;
        g_arm_tool_debug.tx_single_frame_count++;
    } else {
        ArmToolRecordTxFailure(index);
    }
}

static void ARM_TOOL_DISABLED_UNUSED ArmToolRefreshServoFeedback(uint32_t now_ms)
{
    const uint8_t ids[2] = {
        ARM_TOOL_PITCH_SERVO_ID, ARM_GRIPPER_SERVO_ID
    };
    uint8_t index;

    for (index = 0u; index < 2u; ++index) {
        HuanerServo_Status_s status;

        if (HuanerServoGetStatus(ids[index], &status) == 0u) {
            g_arm_tool_debug.servo_online[index] = 0u;
            g_arm_tool_debug.servo_feedback_valid[index] = 0u;
            g_arm_tool_debug.servo_arrived[index] = 0u;
            continue;
        }
        g_arm_tool_debug.servo_online[index] = status.online;
        g_arm_tool_debug.servo_feedback_valid[index] =
            HuanerServoFeedbackFresh(ids[index], now_ms);
        g_arm_tool_debug.servo_arrived[index] = status.arrived;
        g_arm_tool_debug.servo_motion_timeout[index] =
            status.motion_timeout;
        g_arm_tool_debug.servo_feedback_pos[index] =
            status.feedback_position;
        g_arm_tool_debug.servo_position_error[index] =
            status.position_error;
        g_arm_tool_debug.servo_feedback_velocity_pos_s[index] =
            status.feedback_velocity_pos_s;
        g_arm_tool_debug.servo_last_feedback_tick[index] =
            status.last_feedback_tick;
        g_arm_tool_debug.servo_feedback_sequence[index] =
            status.feedback_sequence;
    }

    g_arm_tool_debug.gripper_feedback_pos =
        g_arm_tool_debug.servo_feedback_pos[1];
    g_arm_tool_debug.gripper_position_error =
        g_arm_tool_debug.servo_position_error[1];
    if (g_arm_tool_debug.servo_feedback_valid[0] != 0u &&
        isfinite(g_arm_tool_debug.small_link_pitch_deg)) {
        g_arm_tool_debug.tool_pitch_feedback_deg = ArmToolPitchFromFeedback(
            g_arm_tool_debug.small_link_pitch_deg,
            g_arm_tool_debug.servo_feedback_pos[0]);
    }

    g_arm_servo_angle_debug.servo1_communication_ok =
        g_arm_tool_debug.servo_online[0] != 0u &&
        g_arm_tool_debug.servo_feedback_valid[0] != 0u;
    g_arm_servo_angle_debug.servo2_communication_ok =
        g_arm_tool_debug.servo_online[1] != 0u &&
        g_arm_tool_debug.servo_feedback_valid[1] != 0u;
    g_arm_servo_angle_debug.servo1_current_deg =
        g_arm_servo_angle_debug.servo1_communication_ok != 0u ?
            ArmToolServoPositionToDeg(
                g_arm_tool_debug.servo_feedback_pos[0]) : NAN;
    g_arm_servo_angle_debug.servo2_current_deg =
        g_arm_servo_angle_debug.servo2_communication_ok != 0u ?
            ArmToolServoPositionToDeg(
                g_arm_tool_debug.servo_feedback_pos[1]) : NAN;
    g_arm_servo_angle_debug.servo1_target_deg =
        ArmToolServoPositionToDeg(g_arm_tool_debug.servo_target_pos[0]);
    g_arm_servo_angle_debug.servo2_target_deg =
        ArmToolServoPositionToDeg(g_arm_tool_debug.servo_target_pos[1]);
}

void ArmToolUpdateSmallLinkPitch(float small_link_pitch_deg)
{
    g_arm_tool_debug.small_link_pitch_deg = small_link_pitch_deg;
    if (g_arm_tool_debug.servo_feedback_valid[0] != 0u &&
        isfinite(small_link_pitch_deg)) {
        g_arm_tool_debug.tool_pitch_feedback_deg = ArmToolPitchFromFeedback(
            small_link_pitch_deg,
            g_arm_tool_debug.servo_feedback_pos[0]);
    }
}

float ArmToolSmallLinkPitchFromJoint(const float q_deg[3])
{
    if (q_deg == NULL || !isfinite(q_deg[1]) || !isfinite(q_deg[2])) {
        return NAN;
    }
    return q_deg[1] + (-180.0f - q_deg[2]);
}

uint8_t ArmToolPitchPositionForPose(float tool_pitch_deg,
                                     float small_link_pitch_deg,
                                     uint16_t *position)
{
    float relative_pitch_deg;
    float position_f;

    if (position == NULL || !isfinite(tool_pitch_deg) ||
        !isfinite(small_link_pitch_deg)) {
        return 0u;
    }
    relative_pitch_deg = tool_pitch_deg - small_link_pitch_deg;
    if (relative_pitch_deg < ARM_TOOL_PITCH_RELATIVE_MIN_DEG ||
        relative_pitch_deg > ARM_TOOL_PITCH_RELATIVE_MAX_DEG) {
        return 0u;
    }
    position_f = (float)ARM_TOOL_PITCH_NEUTRAL_POS +
        ARM_TOOL_PITCH_DIRECTION * relative_pitch_deg *
        (float)(ARM_TOOL_SERVO_POS_MAX - ARM_TOOL_SERVO_POS_MIN) /
        ARM_TOOL_SERVO_RANGE_DEG;
    if (!isfinite(position_f) ||
        position_f < (float)ARM_TOOL_PITCH_SERVO_MIN_POS ||
        position_f > (float)ARM_TOOL_PITCH_SERVO_MAX_POS) {
        return 0u;
    }
    *position = (uint16_t)(position_f + 0.5f);
    return ArmToolPositionInRange(ARM_TOOL_PITCH_SERVO_ID, *position);
}

uint8_t ArmToolPitchValidForPose(float tool_pitch_deg,
                                  const float q_deg[3])
{
    uint16_t position;
    float small_link_pitch_deg = ArmToolSmallLinkPitchFromJoint(q_deg);

    return ArmToolPitchPositionForPose(tool_pitch_deg,
                                        small_link_pitch_deg,
                                        &position);
}

float ArmToolPitchFromFeedback(float small_link_pitch_deg,
                               uint16_t position)
{
    if (!isfinite(small_link_pitch_deg) ||
        position < ARM_TOOL_PITCH_SERVO_MIN_POS ||
        position > ARM_TOOL_PITCH_SERVO_MAX_POS ||
        fabsf(ARM_TOOL_PITCH_DIRECTION) < 0.000001f) {
        return NAN;
    }
    return small_link_pitch_deg +
        ((float)position - (float)ARM_TOOL_PITCH_NEUTRAL_POS) *
        ARM_TOOL_SERVO_RANGE_DEG /
        (float)(ARM_TOOL_SERVO_POS_MAX - ARM_TOOL_SERVO_POS_MIN) /
        ARM_TOOL_PITCH_DIRECTION;
}

Arm_Command_Result_e ArmToolSetPitchDeg(float tool_pitch_deg)
{
    uint16_t position;
    Arm_Command_Result_e result;

    if (!isfinite(tool_pitch_deg) ||
        tool_pitch_deg < ARM_USB_TOOL_PITCH_MIN_DEG ||
        tool_pitch_deg > ARM_USB_TOOL_PITCH_MAX_DEG) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_RANGE;
        return ARM_COMMAND_INVALID;
    }
    if (g_arm_tool_debug.init_state != ARM_TOOL_INIT_DONE) {
        return ARM_COMMAND_NOT_READY;
    }
    if (!ArmToolPitchPositionForPose(
            tool_pitch_deg, g_arm_tool_debug.small_link_pitch_deg,
            &position)) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_RANGE;
        return ARM_COMMAND_PREFLIGHT_FAILED;
    }
    /*
     * 这是一次显式工具命令，必须强制建立新目标，不能被20 ms跟踪限频
     * 或旧目标死区吞掉。后续主臂运动中的连续补偿仍由TrackPitch限频。
     */
    result = ArmToolQueuePosition(ARM_TOOL_PITCH_SERVO_ID, position,
                                  ARM_TOOL_PITCH_TRACK_TIME_MS, 1u);
    if (result != ARM_COMMAND_OK) {
        return result;
    }
    g_arm_tool_debug.tool_pitch_target_deg = tool_pitch_deg;
    g_arm_tool_debug.tool_pitch_target_valid = 1u;
    return ARM_COMMAND_OK;
}

Arm_Command_Result_e ArmToolHoldCurrentPitch(float small_link_pitch_deg)
{
    float pitch_deg;

    if (g_arm_tool_debug.servo_feedback_valid[0] == 0u) {
        return ARM_COMMAND_NOT_READY;
    }
    pitch_deg = ArmToolPitchFromFeedback(
        small_link_pitch_deg, g_arm_tool_debug.servo_feedback_pos[0]);
    if (!isfinite(pitch_deg)) {
        return ARM_COMMAND_INVALID;
    }
    g_arm_tool_debug.tool_pitch_target_deg = pitch_deg;
    g_arm_tool_debug.tool_pitch_target_valid = 1u;
    return ARM_COMMAND_OK;
}

Arm_Command_Result_e ArmToolTrackPitch(float tool_pitch_deg,
                                        float small_link_pitch_deg,
                                        uint32_t now_ms)
{
    uint16_t position;
    Arm_Command_Result_e result;

    /* 保留时间参数以兼容轨迹调用；发送节拍由QueuePosition统一读取。 */
    (void)now_ms;

    g_arm_tool_debug.small_link_pitch_deg = small_link_pitch_deg;
    if (!ArmToolPitchPositionForPose(tool_pitch_deg, small_link_pitch_deg,
                                     &position)) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_RANGE;
        return ARM_COMMAND_PREFLIGHT_FAILED;
    }
    if (g_arm_tool_debug.init_state != ARM_TOOL_INIT_DONE) {
        return ARM_COMMAND_NOT_READY;
    }
    result = ArmToolQueuePosition(ARM_TOOL_PITCH_SERVO_ID, position,
                                  ARM_TOOL_PITCH_TRACK_TIME_MS, 0u);
    if (result == ARM_COMMAND_OK) {
        g_arm_tool_debug.tool_pitch_target_deg = tool_pitch_deg;
        g_arm_tool_debug.tool_pitch_target_valid = 1u;
    }
    return result;
}

static uint8_t ArmToolGripperStateBusy(Arm_Gripper_State_e state)
{
    return state == ARM_GRIPPER_BOOTING ||
           state == ARM_GRIPPER_READYING ||
           state == ARM_GRIPPER_OPENING ||
           state == ARM_GRIPPER_CLOSING ||
           state == ARM_GRIPPER_CONTACT_SUSPECTED ||
           state == ARM_GRIPPER_RELIEVING;
}

static void ArmToolResetStallWindow(uint32_t now_ms, uint16_t position)
{
    g_arm_tool_debug.gripper_stall_window_start_tick = now_ms;
    g_arm_tool_debug.gripper_stall_window_min_pos = position;
    g_arm_tool_debug.gripper_stall_window_max_pos = position;
    g_arm_tool_debug.gripper_stall_candidate = 0u;
}

static Arm_Command_Result_e ArmToolBeginGripperMotion(
    Arm_Gripper_Command_e command,
    Arm_Gripper_State_e moving_state,
    Arm_Gripper_State_e target_state,
    uint16_t target_position,
    uint16_t time_ms)
{
    Arm_Command_Result_e result;
    uint32_t now_ms = HAL_GetTick();

    if (g_arm_tool_debug.servo_feedback_valid[1] == 0u) {
        return ARM_COMMAND_NOT_READY;
    }
    if (ArmToolGripperStateBusy(g_arm_tool_debug.gripper_state) != 0u) {
        return ARM_COMMAND_BUSY;
    }
    result = ArmToolQueuePosition(ARM_GRIPPER_SERVO_ID, target_position,
                                  time_ms, 1u);
    if (result != ARM_COMMAND_OK) {
        return result;
    }

    g_arm_tool_debug.gripper_command = command;
    g_arm_tool_debug.gripper_state = moving_state;
    g_arm_tool_debug.gripper_target_state = target_state;
    g_arm_tool_debug.gripper_action_start_tick = now_ms;
    g_arm_tool_debug.gripper_settle_start_tick = 0u;
    g_arm_tool_debug.gripper_close_start_pos =
        g_arm_tool_debug.gripper_feedback_pos;
    g_arm_tool_debug.gripper_stall_latched = 0u;
    g_arm_tool_debug.gripper_stall_candidate = 0u;
    g_arm_tool_debug.gripper_fault_latched = 0u;
    g_arm_tool_debug.gripper_boot_stall = 0u;
    g_arm_tool_debug.gripper_last_feedback_sequence =
        g_arm_tool_debug.servo_feedback_sequence[1];
    arm_gripper_relief_outcome = ARM_GRIPPER_RELIEF_NONE;
    ArmToolResetStallWindow(now_ms,
                            g_arm_tool_debug.gripper_feedback_pos);
    return ARM_COMMAND_OK;
}

Arm_Command_Result_e ArmToolSetGripper(Arm_Gripper_Command_e command)
{
    if (command == ARM_GRIPPER_COMMAND_HOLD) {
        return ARM_COMMAND_OK;
    }
    if (g_arm_tool_debug.init_state != ARM_TOOL_INIT_DONE) {
        return ARM_COMMAND_NOT_READY;
    }
    if (command == ARM_GRIPPER_COMMAND_READY) {
        return ArmToolBeginGripperMotion(
            command, ARM_GRIPPER_READYING, ARM_GRIPPER_READY,
            ARM_GRIPPER_READY_POS, ARM_GRIPPER_MOVE_TIME_MS);
    }
    if (command == ARM_GRIPPER_COMMAND_OPEN) {
        return ArmToolBeginGripperMotion(
            command, ARM_GRIPPER_OPENING, ARM_GRIPPER_OPEN,
            ARM_GRIPPER_OPEN_POS, ARM_GRIPPER_MOVE_TIME_MS);
    }
    if (command == ARM_GRIPPER_COMMAND_CLOSE) {
        return ArmToolBeginGripperMotion(
            command, ARM_GRIPPER_CLOSING, ARM_GRIPPER_CLOSED_EMPTY,
            ARM_GRIPPER_CLOSE_POS, ARM_GRIPPER_MOVE_TIME_MS);
    }
    return ARM_COMMAND_INVALID;
}

static uint8_t ArmToolGripperFeedbackArrived(void)
{
    return g_arm_tool_debug.servo_feedback_valid[1] != 0u &&
           g_arm_tool_debug.tx_pending[1] == 0u &&
           g_arm_tool_debug.servo_arrived[1] != 0u &&
           g_arm_tool_debug.servo_target_pos[1] ==
               g_arm_tool_debug.gripper_target_pos;
}

/* ID2闭合使用比底层通用舵机更严格的到位窗口，避免小误差掩盖持续顶压。 */
static uint8_t ArmToolGripperCloseArrived(uint32_t now_ms)
{
    int32_t error = (int32_t)g_arm_tool_debug.gripper_target_pos -
        (int32_t)g_arm_tool_debug.gripper_feedback_pos;

    if (error < 0) {
        error = -error;
    }
    if (g_arm_tool_debug.servo_feedback_valid[1] == 0u ||
        g_arm_tool_debug.tx_pending[1] != 0u ||
        (uint32_t)error > ARM_GRIPPER_CLOSE_ARRIVAL_ERROR_POS) {
        g_arm_tool_debug.gripper_settle_start_tick = 0u;
        return 0u;
    }
    if (g_arm_tool_debug.gripper_settle_start_tick == 0u) {
        g_arm_tool_debug.gripper_settle_start_tick = now_ms;
        return 0u;
    }
    return (uint32_t)(now_ms -
        g_arm_tool_debug.gripper_settle_start_tick) >=
        ARM_GRIPPER_CLOSE_ARRIVAL_STABLE_MS;
}

static uint8_t ArmToolQueueNextRelief(uint32_t now_ms)
{
    uint16_t feedback = g_arm_tool_debug.gripper_feedback_pos;
    uint16_t relief_base =
        g_arm_tool_debug.gripper_relief_attempt_count == 0u ?
        feedback : g_arm_tool_debug.gripper_relief_target_pos;
    uint16_t target = relief_base >
            (ARM_GRIPPER_SERVO_MIN_POS + ARM_GRIPPER_RELIEF_STEP_POS) ?
        (uint16_t)(relief_base - ARM_GRIPPER_RELIEF_STEP_POS) :
        ARM_GRIPPER_SERVO_MIN_POS;
    Arm_Command_Result_e result;

    if (g_arm_tool_debug.gripper_relief_attempt_count >=
        ARM_GRIPPER_RELIEF_MAX_ATTEMPTS) {
        return 0u;
    }
    result = ArmToolQueuePosition(
        ARM_GRIPPER_SERVO_ID, target,
        ARM_GRIPPER_RELIEF_MOVE_TIME_MS, 1u);
    if (result != ARM_COMMAND_OK) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_TX;
        return 0u;
    }
    g_arm_tool_debug.gripper_relief_attempt_count++;
    g_arm_tool_debug.gripper_relief_start_pos = feedback;
    g_arm_tool_debug.gripper_relief_target_pos = target;
    g_arm_tool_debug.gripper_action_start_tick = now_ms;
    g_arm_tool_debug.gripper_settle_start_tick = 0u;
    return 1u;
}

static void ArmToolStartRelief(Arm_Gripper_Relief_Outcome_e outcome,
                               uint32_t now_ms)
{
    g_arm_tool_debug.gripper_relief_attempt_count = 0u;
    if (ArmToolQueueNextRelief(now_ms) == 0u) {
        g_arm_tool_debug.gripper_state = ARM_GRIPPER_FAULT;
        g_arm_tool_debug.gripper_fault_latched = 1u;
        if (g_arm_tool_debug.init_state != ARM_TOOL_INIT_DONE) {
            g_arm_tool_debug.init_state = ARM_TOOL_INIT_ERROR;
        }
        return;
    }
    arm_gripper_relief_outcome = outcome;
    g_arm_tool_debug.gripper_state = ARM_GRIPPER_RELIEVING;
    g_arm_tool_debug.gripper_target_state =
        outcome == ARM_GRIPPER_RELIEF_CONTACT ?
        ARM_GRIPPER_HELD_CONTACT : ARM_GRIPPER_FAULT;
    if (g_arm_tool_debug.init_state != ARM_TOOL_INIT_DONE) {
        g_arm_tool_debug.init_state = ARM_TOOL_INIT_RELIEVING;
    }
}

static uint8_t ArmToolUpdateStallWindow(uint32_t now_ms)
{
    uint16_t position = g_arm_tool_debug.gripper_feedback_pos;
    uint16_t span;
    int32_t error;

    if (g_arm_tool_debug.servo_feedback_sequence[1] ==
        g_arm_tool_debug.gripper_last_feedback_sequence) {
        return 0u;
    }
    g_arm_tool_debug.gripper_last_feedback_sequence =
        g_arm_tool_debug.servo_feedback_sequence[1];
    if (position < g_arm_tool_debug.gripper_stall_window_min_pos) {
        g_arm_tool_debug.gripper_stall_window_min_pos = position;
    }
    if (position > g_arm_tool_debug.gripper_stall_window_max_pos) {
        g_arm_tool_debug.gripper_stall_window_max_pos = position;
    }
    if ((uint32_t)(now_ms -
            g_arm_tool_debug.gripper_stall_window_start_tick) <
        ARM_GRIPPER_STALL_WINDOW_MS) {
        return 0u;
    }

    span = (uint16_t)(g_arm_tool_debug.gripper_stall_window_max_pos -
                      g_arm_tool_debug.gripper_stall_window_min_pos);
    error = (int32_t)g_arm_tool_debug.gripper_target_pos -
            (int32_t)position;
    g_arm_tool_debug.gripper_stall_candidate =
        error >= (int32_t)ARM_GRIPPER_STALL_ERROR_POS &&
        span <= ARM_GRIPPER_STALL_MAX_POSITION_SPAN ? 1u : 0u;
    if (g_arm_tool_debug.gripper_stall_candidate != 0u) {
        return 1u;
    }
    ArmToolResetStallWindow(now_ms, position);
    return 0u;
}

static void ArmToolHandleReliefAttemptTimeout(uint32_t now_ms)
{
    if (ArmToolQueueNextRelief(now_ms) == 0u) {
        uint8_t contact_attempts_exhausted =
            arm_gripper_relief_outcome == ARM_GRIPPER_RELIEF_CONTACT &&
            g_arm_tool_debug.gripper_relief_attempt_count >=
                ARM_GRIPPER_RELIEF_MAX_ATTEMPTS;

        g_arm_tool_debug.gripper_jam_count++;
        g_arm_tool_debug.gripper_state =
            contact_attempts_exhausted != 0u ?
                ARM_GRIPPER_FORCED_HELD : ARM_GRIPPER_FAULT;
        g_arm_tool_debug.gripper_fault_latched =
            contact_attempts_exhausted != 0u ? 0u : 1u;
        if (contact_attempts_exhausted != 0u) {
            g_arm_tool_debug.gripper_forced_held_count++;
        }
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_GRIPPER_STALL;
        if (g_arm_tool_debug.init_state == ARM_TOOL_INIT_RELIEVING) {
            g_arm_tool_debug.init_state = ARM_TOOL_INIT_ERROR;
        }
        arm_gripper_relief_outcome = ARM_GRIPPER_RELIEF_NONE;
    }
}

static void ArmToolProcessClosing(uint32_t now_ms, uint8_t boot_motion)
{
    uint32_t elapsed_ms = now_ms -
        g_arm_tool_debug.gripper_action_start_tick;
    uint32_t deadline_ms = boot_motion != 0u ?
        ARM_GRIPPER_BOOT_DEADLINE_MS : ARM_GRIPPER_CLOSE_DEADLINE_MS;

    if (g_arm_tool_debug.servo_feedback_valid[1] == 0u) {
        /* 反馈失鲜不能绕过动作总截止时间，否则会永久停在CLOSING。 */
        if (g_arm_tool_debug.servo_online[1] == 0u ||
            elapsed_ms >= deadline_ms) {
            g_arm_tool_debug.gripper_state = ARM_GRIPPER_FAULT;
            g_arm_tool_debug.gripper_fault_latched = 1u;
            g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_FEEDBACK;
            if (elapsed_ms >= deadline_ms) {
                g_arm_tool_debug.gripper_timeout_count++;
            }
            if (boot_motion != 0u) {
                g_arm_tool_debug.init_state = ARM_TOOL_INIT_ERROR;
            }
        }
        return;
    }
    if (boot_motion == 0u && ArmToolGripperCloseArrived(now_ms) != 0u) {
        g_arm_tool_debug.gripper_state = ARM_GRIPPER_CLOSED_EMPTY;
        g_arm_tool_debug.gripper_target_state = ARM_GRIPPER_CLOSED_EMPTY;
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_NONE;
        return;
    }
    if (boot_motion != 0u && ArmToolGripperFeedbackArrived() != 0u) {
        return;
    }
    if (elapsed_ms < ARM_GRIPPER_STALL_START_IGNORE_MS) {
        /* 忽略期只观察启动，不把这些样本带入后续300 ms停滞窗口。 */
        ArmToolResetStallWindow(now_ms,
                                g_arm_tool_debug.gripper_feedback_pos);
    } else if (ArmToolUpdateStallWindow(now_ms) != 0u) {
        g_arm_tool_debug.gripper_stall_latched = 1u;
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_GRIPPER_STALL;
        if (boot_motion != 0u) {
            g_arm_tool_debug.gripper_boot_stall = 1u;
            g_arm_tool_debug.gripper_jam_count++;
            ArmToolStartRelief(ARM_GRIPPER_RELIEF_BOOT, now_ms);
        } else {
            g_arm_tool_debug.gripper_state =
                ARM_GRIPPER_CONTACT_SUSPECTED;
            g_arm_tool_debug.gripper_contact_count++;
            ArmToolStartRelief(ARM_GRIPPER_RELIEF_CONTACT, now_ms);
        }
        return;
    }
    if (elapsed_ms >= deadline_ms) {
        g_arm_tool_debug.gripper_timeout_count++;
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_TIMEOUT;
        ArmToolStartRelief(boot_motion != 0u ?
            ARM_GRIPPER_RELIEF_BOOT : ARM_GRIPPER_RELIEF_TIMEOUT, now_ms);
    }
}

static void ArmToolProcessRelief(uint32_t now_ms)
{
    int32_t relief_error;
    uint32_t elapsed_ms = now_ms -
        g_arm_tool_debug.gripper_action_start_tick;

    if (g_arm_tool_debug.servo_feedback_valid[1] == 0u) {
        if (g_arm_tool_debug.servo_online[1] == 0u) {
            g_arm_tool_debug.gripper_state = ARM_GRIPPER_FAULT;
            g_arm_tool_debug.gripper_fault_latched = 1u;
            g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_FEEDBACK;
            if (g_arm_tool_debug.init_state == ARM_TOOL_INIT_RELIEVING) {
                g_arm_tool_debug.init_state = ARM_TOOL_INIT_ERROR;
            }
        } else if (elapsed_ms >= ARM_GRIPPER_RELIEF_ATTEMPT_TIMEOUT_MS) {
            /* 短时失鲜仍推进本次回退，四次接触卸力最终进入FORCED_HELD。 */
            g_arm_tool_debug.gripper_settle_start_tick = 0u;
            ArmToolHandleReliefAttemptTimeout(now_ms);
        }
        return;
    }
    relief_error = (int32_t)g_arm_tool_debug.gripper_feedback_pos -
        (int32_t)g_arm_tool_debug.gripper_relief_target_pos;
    if (relief_error < 0) {
        relief_error = -relief_error;
    }
    if (g_arm_tool_debug.tx_pending[1] != 0u ||
        (uint32_t)relief_error > ARM_GRIPPER_RELIEF_ARRIVAL_ERROR_POS) {
        g_arm_tool_debug.gripper_settle_start_tick = 0u;
        if (elapsed_ms >= ARM_GRIPPER_RELIEF_ATTEMPT_TIMEOUT_MS) {
            ArmToolHandleReliefAttemptTimeout(now_ms);
        }
        return;
    }
    if (g_arm_tool_debug.gripper_settle_start_tick == 0u) {
        g_arm_tool_debug.gripper_settle_start_tick = now_ms;
        return;
    }
    if ((uint32_t)(now_ms -
            g_arm_tool_debug.gripper_settle_start_tick) <
        ARM_GRIPPER_SETTLE_MS) {
        return;
    }

    if (arm_gripper_relief_outcome == ARM_GRIPPER_RELIEF_CONTACT) {
        g_arm_tool_debug.gripper_state = ARM_GRIPPER_HELD_CONTACT;
        g_arm_tool_debug.gripper_target_state = ARM_GRIPPER_HELD_CONTACT;
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_NONE;
    } else if (arm_gripper_relief_outcome == ARM_GRIPPER_RELIEF_JAM) {
        g_arm_tool_debug.gripper_state = ARM_GRIPPER_JAMMED;
        g_arm_tool_debug.gripper_fault_latched = 1u;
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_GRIPPER_STALL;
    } else {
        g_arm_tool_debug.gripper_state = ARM_GRIPPER_FAULT;
        g_arm_tool_debug.gripper_fault_latched = 1u;
        if (g_arm_tool_debug.init_state == ARM_TOOL_INIT_RELIEVING) {
            g_arm_tool_debug.init_state = ARM_TOOL_INIT_ERROR;
        }
    }
    arm_gripper_relief_outcome = ARM_GRIPPER_RELIEF_NONE;
}

static void ARM_TOOL_DISABLED_UNUSED ArmToolProcessGripper(uint32_t now_ms)
{
    switch (g_arm_tool_debug.gripper_state) {
        case ARM_GRIPPER_BOOTING:
            ArmToolProcessClosing(now_ms, 1u);
            break;

        case ARM_GRIPPER_CLOSING:
            ArmToolProcessClosing(now_ms, 0u);
            break;

        case ARM_GRIPPER_RELIEVING:
            ArmToolProcessRelief(now_ms);
            break;

        case ARM_GRIPPER_READYING:
        case ARM_GRIPPER_OPENING:
            if (g_arm_tool_debug.servo_feedback_valid[1] == 0u) {
                if (g_arm_tool_debug.servo_online[1] == 0u) {
                    g_arm_tool_debug.gripper_state = ARM_GRIPPER_FAULT;
                    g_arm_tool_debug.gripper_fault_latched = 1u;
                    g_arm_tool_debug.error_code =
                        ARM_TOOL_ERROR_SERVO_FEEDBACK;
                }
            } else if (ArmToolGripperFeedbackArrived() != 0u) {
                g_arm_tool_debug.gripper_state =
                    g_arm_tool_debug.gripper_target_state;
                g_arm_tool_debug.gripper_fault_latched = 0u;
                g_arm_tool_debug.error_code = ARM_TOOL_ERROR_NONE;
            } else if ((uint32_t)(now_ms -
                           g_arm_tool_debug.gripper_action_start_tick) >
                       ARM_GRIPPER_MOVE_TIME_MS +
                           ARM_TOOL_ACTION_DEADLINE_MARGIN_MS) {
                g_arm_tool_debug.gripper_state = ARM_GRIPPER_FAULT;
                g_arm_tool_debug.gripper_fault_latched = 1u;
                g_arm_tool_debug.gripper_timeout_count++;
                g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_TIMEOUT;
            }
            break;

        case ARM_GRIPPER_UNKNOWN:
        case ARM_GRIPPER_READY:
        case ARM_GRIPPER_OPEN:
        case ARM_GRIPPER_CONTACT_SUSPECTED:
        case ARM_GRIPPER_HELD_CONTACT:
        case ARM_GRIPPER_CLOSED_EMPTY:
        case ARM_GRIPPER_FORCED_HELD:
        case ARM_GRIPPER_JAMMED:
        case ARM_GRIPPER_FAULT:
        default:
            break;
    }
}

static void ARM_TOOL_DISABLED_UNUSED ArmToolProcessInit(uint32_t now_ms)
{
    Arm_Command_Result_e pitch_result;
    Arm_Command_Result_e gripper_result;

    switch (g_arm_tool_debug.init_state) {
        case ARM_TOOL_INIT_WAIT_FEEDBACK:
            if ((uint32_t)(now_ms - g_arm_tool_debug.last_update_tick) <
                    ARM_TOOL_BOOT_START_DELAY_MS ||
                g_arm_tool_debug.servo_feedback_valid[0] == 0u ||
                g_arm_tool_debug.servo_feedback_valid[1] == 0u) {
                break;
            }
            g_arm_tool_debug.init_state = ARM_TOOL_INIT_COMMAND_BOOT;
            break;

        case ARM_TOOL_INIT_COMMAND_BOOT:
            pitch_result = ArmToolQueuePosition(
                ARM_TOOL_PITCH_SERVO_ID, ARM_TOOL_PITCH_NEUTRAL_POS,
                ARM_TOOL_BOOT_MOVE_TIME_MS, 1u);
            gripper_result = ArmToolQueuePosition(
                ARM_GRIPPER_SERVO_ID, ARM_GRIPPER_BOOT_POS,
                ARM_GRIPPER_BOOT_MOVE_TIME_MS, 1u);
            g_arm_tool_debug.init_result[0] = pitch_result;
            g_arm_tool_debug.init_result[1] = gripper_result;
            if (pitch_result == ARM_COMMAND_OK &&
                gripper_result == ARM_COMMAND_OK) {
                g_arm_tool_debug.gripper_command =
                    ARM_GRIPPER_COMMAND_HOLD;
                g_arm_tool_debug.gripper_state = ARM_GRIPPER_BOOTING;
                g_arm_tool_debug.gripper_target_state =
                    ARM_GRIPPER_BOOTING;
                g_arm_tool_debug.gripper_action_start_tick = now_ms;
                g_arm_tool_debug.gripper_close_start_pos =
                    g_arm_tool_debug.gripper_feedback_pos;
                g_arm_tool_debug.gripper_last_feedback_sequence =
                    g_arm_tool_debug.servo_feedback_sequence[1];
                ArmToolResetStallWindow(
                    now_ms, g_arm_tool_debug.gripper_feedback_pos);
                g_arm_tool_debug.init_state = ARM_TOOL_INIT_WAIT_BOOT;
            } else if (pitch_result != ARM_COMMAND_BUSY &&
                       gripper_result != ARM_COMMAND_BUSY) {
                g_arm_tool_debug.init_state = ARM_TOOL_INIT_ERROR;
            }
            break;

        case ARM_TOOL_INIT_WAIT_BOOT:
            if (g_arm_tool_debug.gripper_state == ARM_GRIPPER_RELIEVING ||
                g_arm_tool_debug.gripper_state == ARM_GRIPPER_FAULT) {
                break;
            }
            if (g_arm_tool_debug.servo_feedback_valid[0] == 0u ||
                g_arm_tool_debug.servo_feedback_valid[1] == 0u) {
                if (g_arm_tool_debug.servo_online[0] == 0u ||
                    g_arm_tool_debug.servo_online[1] == 0u) {
                    g_arm_tool_debug.init_state = ARM_TOOL_INIT_ERROR;
                    g_arm_tool_debug.error_code =
                        ARM_TOOL_ERROR_SERVO_FEEDBACK;
                }
                break;
            }
            if (g_arm_tool_debug.tx_pending[0] == 0u &&
                g_arm_tool_debug.tx_pending[1] == 0u &&
                g_arm_tool_debug.servo_arrived[0] != 0u &&
                g_arm_tool_debug.servo_arrived[1] != 0u) {
                g_arm_tool_debug.tool_ready = 1u;
                g_arm_tool_debug.tool_pitch_servo_pos =
                    ARM_TOOL_PITCH_NEUTRAL_POS;
                g_arm_tool_debug.gripper_target_pos =
                    ARM_GRIPPER_BOOT_POS;
                /* ID2初始化、等待抓取和释放统一使用默认位置550。 */
                g_arm_tool_debug.gripper_state = ARM_GRIPPER_UNKNOWN;
                g_arm_tool_debug.gripper_target_state =
                    ARM_GRIPPER_UNKNOWN;
                g_arm_tool_debug.init_state = ARM_TOOL_INIT_DONE;
                g_arm_tool_debug.error_code = ARM_TOOL_ERROR_NONE;
            } else if ((uint32_t)(now_ms -
                           g_arm_tool_debug.gripper_action_start_tick) >
                       ARM_GRIPPER_BOOT_DEADLINE_MS ||
                       g_arm_tool_debug.servo_motion_timeout[0] != 0u) {
                g_arm_tool_debug.init_state = ARM_TOOL_INIT_ERROR;
                g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_TIMEOUT;
            }
            break;

        case ARM_TOOL_INIT_RELIEVING:
        case ARM_TOOL_INIT_DONE:
        case ARM_TOOL_INIT_ERROR:
        case ARM_TOOL_INIT_DISABLED:
        default:
            break;
    }
}

void ArmToolInit(void)
{
    memset(&g_arm_servo_angle_debug, 0,
           sizeof(g_arm_servo_angle_debug));
    g_arm_servo_angle_debug.servo1_current_deg = NAN;
    g_arm_servo_angle_debug.servo1_target_deg = NAN;
    g_arm_servo_angle_debug.servo2_current_deg = NAN;
    g_arm_servo_angle_debug.servo2_target_deg = NAN;
    arm_tool_feedback_only_mode = 0u;
    arm_tool_feedback_unload_in_progress = 0u;
#if ARM_TOOL_ENABLE != 0u
    const uint8_t feedback_ids[2] = {
        ARM_TOOL_PITCH_SERVO_ID, ARM_GRIPPER_SERVO_ID
    };
    uint8_t initialized;

    memset(&g_arm_tool_debug, 0, sizeof(g_arm_tool_debug));
    memset(&g_arm_gripper_stall_debug, 0,
           sizeof(g_arm_gripper_stall_debug));
    g_arm_gripper_stall_debug.current_deg = NAN;
    memset(&arm_tool_tx_scheduler, 0, sizeof(arm_tool_tx_scheduler));
    arm_gripper_relief_outcome = ARM_GRIPPER_RELIEF_NONE;
    initialized = ArmToolSelfTest();
    if (initialized != 0u) {
        initialized = HuanerServoInit();
    }
    if (initialized != 0u &&
        HuanerServoConfigureFeedbackPolling(
            feedback_ids, 2u, HUANER_SERVO_DEFAULT_POLL_PERIOD_MS) == 0u) {
        initialized = 0u;
    }
    if (initialized != 0u &&
        HuanerServoConfigureBoardVoltagePolling(
            HUANER_SERVO_DEFAULT_VOLTAGE_POLL_PERIOD_MS) == 0u) {
        initialized = 0u;
    }

    g_arm_tool_debug.initialized = initialized;
    g_arm_tool_debug.init_state = initialized != 0u ?
        ARM_TOOL_INIT_WAIT_FEEDBACK : ARM_TOOL_INIT_ERROR;
    g_arm_tool_debug.gripper_state = ARM_GRIPPER_UNKNOWN;
    g_arm_tool_debug.gripper_target_state = ARM_GRIPPER_BOOTING;
    g_arm_tool_debug.gripper_target_pos = ARM_GRIPPER_BOOT_POS;
    g_arm_tool_debug.tool_pitch_servo_pos = ARM_TOOL_PITCH_NEUTRAL_POS;
    g_arm_tool_debug.servo_target_pos[0] = ARM_TOOL_PITCH_NEUTRAL_POS;
    g_arm_tool_debug.servo_target_pos[1] = ARM_GRIPPER_BOOT_POS;
    g_arm_servo_angle_debug.servo1_target_deg =
        ArmToolServoPositionToDeg(ARM_TOOL_PITCH_NEUTRAL_POS);
    g_arm_servo_angle_debug.servo2_target_deg =
        ArmToolServoPositionToDeg(ARM_GRIPPER_BOOT_POS);
    g_arm_tool_debug.small_link_pitch_deg = NAN;
    g_arm_tool_debug.tool_pitch_target_deg = NAN;
    g_arm_tool_debug.tool_pitch_feedback_deg = NAN;
    g_arm_tool_debug.last_update_tick = HAL_GetTick();
    if (initialized == 0u) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_TX;
    }
#else
    memset(&g_arm_tool_debug, 0, sizeof(g_arm_tool_debug));
    memset(&g_arm_gripper_stall_debug, 0,
           sizeof(g_arm_gripper_stall_debug));
    g_arm_gripper_stall_debug.current_deg = NAN;
    g_arm_tool_debug.init_state = ARM_TOOL_INIT_DISABLED;
#endif
}

void ArmToolInitFeedbackOnly(void)
{
#if ARM_TOOL_ENABLE != 0u
    uint8_t initialized;

    memset(&g_arm_servo_angle_debug, 0,
           sizeof(g_arm_servo_angle_debug));
    memset(&g_arm_tool_debug, 0, sizeof(g_arm_tool_debug));
    memset(&g_arm_gripper_stall_debug, 0,
           sizeof(g_arm_gripper_stall_debug));
    memset(&arm_tool_tx_scheduler, 0, sizeof(arm_tool_tx_scheduler));
    arm_tool_feedback_only_mode = 0u;
    arm_tool_feedback_unload_in_progress = 0u;
    g_arm_servo_angle_debug.servo1_current_deg = NAN;
    g_arm_servo_angle_debug.servo1_target_deg = NAN;
    g_arm_servo_angle_debug.servo2_current_deg = NAN;
    g_arm_servo_angle_debug.servo2_target_deg = NAN;
    g_arm_gripper_stall_debug.current_deg = NAN;
    arm_gripper_relief_outcome = ARM_GRIPPER_RELIEF_NONE;

    initialized = ArmToolSelfTest();
    if (initialized != 0u) {
        initialized = HuanerServoInit();
    }
    g_arm_tool_debug.initialized = initialized;
    g_arm_tool_debug.feedback_only_mode = initialized != 0u ? 1u : 0u;
    arm_tool_feedback_only_mode = g_arm_tool_debug.feedback_only_mode;
    /* DISABLED使现有工具状态机不产生BOOT/HOME目标，只保留反馈刷新。 */
    g_arm_tool_debug.init_state = initialized != 0u ?
        ARM_TOOL_INIT_DISABLED : ARM_TOOL_INIT_ERROR;
    g_arm_tool_debug.gripper_state = ARM_GRIPPER_UNKNOWN;
    g_arm_tool_debug.gripper_target_state = ARM_GRIPPER_UNKNOWN;
    g_arm_tool_debug.small_link_pitch_deg = NAN;
    g_arm_tool_debug.tool_pitch_target_deg = NAN;
    g_arm_tool_debug.tool_pitch_feedback_deg = NAN;
    g_arm_tool_debug.last_update_tick = HAL_GetTick();
    if (initialized == 0u) {
        g_arm_tool_debug.error_code = ARM_TOOL_ERROR_SERVO_TX;
    }
#else
    ArmToolInit();
#endif
}

/*
 * 无力打点模式先发送一次0x14卸载ID1/ID2，再开启0x15位置轮询。
 * 0x14没有应答，因此done只证明DMA发送和控制板发送间隔正常结束；
 * 舵机是否在线仍由后续位置反馈独立判断。
 */
static void ArmToolProcessFeedbackOnlyUnload(void)
{
    static const uint8_t feedback_ids[2] = {
        ARM_TOOL_PITCH_SERVO_ID, ARM_GRIPPER_SERVO_ID
    };
    HuanerServo_Result_e result;

    if (arm_tool_feedback_only_mode == 0u ||
        g_arm_tool_debug.servo_unload_done != 0u) {
        return;
    }

    if (arm_tool_feedback_unload_in_progress != 0u) {
        if (g_huaner_servo_driver_debug.busy != 0u) {
            return;
        }
        arm_tool_feedback_unload_in_progress = 0u;
        if (g_huaner_servo_driver_debug.last_command ==
                HUANER_SERVO_COMMAND_UNLOAD &&
            g_huaner_servo_driver_debug.last_result ==
                HUANER_SERVO_RESULT_OK) {
            if (HuanerServoConfigureFeedbackPolling(
                    feedback_ids, 2u,
                    HUANER_SERVO_DEFAULT_POLL_PERIOD_MS) != 0u) {
                g_arm_tool_debug.servo_unload_done = 1u;
                return;
            }
        }
        g_arm_tool_debug.servo_unload_fail_count++;
        return;
    }

    if (g_huaner_servo_driver_debug.busy != 0u ||
        g_huaner_servo_driver_debug.state != HUANER_SERVO_STATE_IDLE) {
        return;
    }
    result = HuanerServoUnload(feedback_ids, 2u);
    if (result == HUANER_SERVO_RESULT_OK) {
        arm_tool_feedback_unload_in_progress = 1u;
        g_arm_tool_debug.servo_unload_requested = 1u;
        g_arm_tool_debug.servo_unload_count++;
    } else if (result != HUANER_SERVO_RESULT_BUSY) {
        g_arm_tool_debug.servo_unload_fail_count++;
    }
}

void ArmToolTask(uint32_t now_ms)
{
#if ARM_TOOL_ENABLE != 0u
    HuanerServoTask(now_ms);
    ArmToolRefreshServoFeedback(now_ms);
    if (arm_tool_feedback_only_mode != 0u) {
        ArmToolProcessFeedbackOnlyUnload();
        return;
    }
    ArmToolProcessGripper(now_ms);
    ArmToolProcessInit(now_ms);
    ArmToolDispatchPending();
    ArmToolUpdateGripperStallDebug();
#else
    (void)now_ms;
#endif
}

void ArmToolClearPendingCommands(void)
{
    ArmToolClearPending(0u);
    ArmToolClearPending(1u);
}

uint8_t ArmToolTxIdle(void)
{
    return g_huaner_servo_driver_debug.busy == 0u &&
           arm_tool_tx_scheduler.valid[0] == 0u &&
           arm_tool_tx_scheduler.valid[1] == 0u;
}

uint8_t ArmToolGripperActionComplete(void)
{
    return g_arm_tool_debug.gripper_state == ARM_GRIPPER_READY ||
           g_arm_tool_debug.gripper_state == ARM_GRIPPER_OPEN ||
           g_arm_tool_debug.gripper_state == ARM_GRIPPER_HELD_CONTACT ||
           g_arm_tool_debug.gripper_state == ARM_GRIPPER_CLOSED_EMPTY ||
           g_arm_tool_debug.gripper_state == ARM_GRIPPER_FORCED_HELD;
}

uint8_t ArmToolGripperFaulted(void)
{
    return g_arm_tool_debug.gripper_fault_latched != 0u ||
           g_arm_tool_debug.gripper_state == ARM_GRIPPER_JAMMED ||
           g_arm_tool_debug.gripper_state == ARM_GRIPPER_FAULT;
}

uint8_t ArmToolGetCenterFromWrist(const Arm_Position_s *wrist,
                                  float base_yaw_deg,
                                  float tool_pitch_deg,
                                  Arm_Position_s *center)
{
    float yaw_rad;
    float pitch_rad;
    float radial_offset;

    if (wrist == NULL || center == NULL ||
        !isfinite(wrist->x_mm) || !isfinite(wrist->y_mm) ||
        !isfinite(wrist->z_mm) || !isfinite(base_yaw_deg) ||
        !isfinite(tool_pitch_deg)) {
        return 0u;
    }
    yaw_rad = base_yaw_deg * ARM_TOOL_DEG_TO_RAD;
    pitch_rad = tool_pitch_deg * ARM_TOOL_DEG_TO_RAD;
    radial_offset = ARM_TOOL_PITCH_AXIS_TO_CENTER_MM * cosf(pitch_rad);
    center->x_mm = wrist->x_mm + radial_offset * cosf(yaw_rad);
    center->y_mm = wrist->y_mm + radial_offset * sinf(yaw_rad);
    center->z_mm = wrist->z_mm +
        ARM_TOOL_PITCH_AXIS_TO_CENTER_MM * sinf(pitch_rad);
    g_arm_tool_debug.wrist_center_mm = *wrist;
    g_arm_tool_debug.tool_tip_mm = *center;
    return 1u;
}

uint8_t ArmToolGetWristFromCenter(const Arm_Position_s *center,
                                  float base_yaw_deg,
                                  float tool_pitch_deg,
                                  Arm_Position_s *wrist)
{
    float yaw_rad;
    float pitch_rad;
    float radial_offset;

    if (center == NULL || wrist == NULL ||
        !isfinite(center->x_mm) || !isfinite(center->y_mm) ||
        !isfinite(center->z_mm) || !isfinite(base_yaw_deg) ||
        !isfinite(tool_pitch_deg)) {
        return 0u;
    }
    yaw_rad = base_yaw_deg * ARM_TOOL_DEG_TO_RAD;
    pitch_rad = tool_pitch_deg * ARM_TOOL_DEG_TO_RAD;
    radial_offset = ARM_TOOL_PITCH_AXIS_TO_CENTER_MM * cosf(pitch_rad);
    wrist->x_mm = center->x_mm - radial_offset * cosf(yaw_rad);
    wrist->y_mm = center->y_mm - radial_offset * sinf(yaw_rad);
    wrist->z_mm = center->z_mm -
        ARM_TOOL_PITCH_AXIS_TO_CENTER_MM * sinf(pitch_rad);
    return isfinite(wrist->x_mm) && isfinite(wrist->y_mm) &&
           isfinite(wrist->z_mm) ? 1u : 0u;
}

const Arm_Tool_State_s *ArmToolGetState(void)
{
    return &g_arm_tool_debug;
}

void ArmToolStopServo1Tracking(void)
{
    ArmToolClearPending(0u);
}
