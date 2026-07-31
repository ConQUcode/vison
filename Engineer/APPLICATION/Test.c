#include "DJI_motor.h"
#include "dmmotor.h"
#include "Test.h"
#include "arm_config.h"
#include "arm_usb_bridge.h"
#include "buzzer.h"
#include "hsl_servo.h"
#include "protocol.h"
#include "math.h"
#include "string.h"

Arm_Host_Sim_Debug_s g_arm_host_sim_debug;
Feetech_Servo_Test_Debug_s g_feetech_servo_test_debug;

#define ARM_HOST_SIM_PI 3.14159265358979323846f

#define FEETECH_SERVO_SWEEP_TEST_ENABLE       0u
#define FEETECH_SERVO_SWEEP_TEST_ID           1u
#define FEETECH_SERVO_SWEEP_START_DELAY_MS    500u
#define FEETECH_SERVO_SWEEP_STEP_INTERVAL_MS  900u
#define FEETECH_SERVO_SWEEP_MOVE_TIME_MS      600u

static const uint16_t feetech_servo_sweep_pos[] = {
    333u, 417u, 375u
};

/*
 * 模拟上位机100Hz发送最新目标：
 * 1. 初始化完成后先用公开关节接口进入安全中心姿态；
 * 2. 用平滑关节正弦生成期望姿态，再通过FK转换成上位机应发送的XYZ；
 * 3. 每10ms调用一次ArmSubmitRealtimeCartesianTarget()。
 * 该测试不会直接访问电机、PID或CAN，和未来上位机走的是同一公开接口。
 */
static void ArmRealtimeHostSimulatorTask(void)
{
#if ARM_REALTIME_HOST_SIM_ENABLE != 0u
    const Arm_State_s *arm_state = ArmGetState();
    const Arm_Motion_Debug_s *motion = ArmGetMotionState();
    uint32_t now = HAL_GetTick();

    if (arm_state->mode != ARM_MODE_READY || !arm_state->kinematics_valid ||
        !arm_state->motor_online[0] || !arm_state->motor_online[1] ||
        !arm_state->motor_online[2]) {
        g_arm_host_sim_debug.state = ARM_HOST_SIM_WAIT_READY;
        g_arm_host_sim_debug.last_result = ARM_COMMAND_NOT_READY;
        return;
    }

    switch (g_arm_host_sim_debug.state) {
        case ARM_HOST_SIM_WAIT_READY:
        case ARM_HOST_SIM_MOVE_TO_CENTER:
        {
            Arm_Cartesian_Command_s command;

            memset(&command, 0, sizeof(command));
            command.command_id = ++g_arm_host_sim_debug.command_id;
            command.control_point = ARM_CONTROL_POINT_WRIST_CENTER;
            command.move_type = ARM_MOVE_LINEAR;
            ArmForwardKinematics3DOF(0.0f, 60.0f, -95.0f,
                                     &command.target_mm);
            command.max_speed_mm_s = ARM_LINEAR_DEFAULT_SPEED_MM_S;
            g_arm_host_sim_debug.last_result =
                ArmSubmitCartesianCommand(&command);
            if (g_arm_host_sim_debug.last_result == ARM_COMMAND_OK) {
                g_arm_host_sim_debug.state = ARM_HOST_SIM_WAIT_CENTER;
            }
            break;
        }

        case ARM_HOST_SIM_WAIT_CENTER:
            if (motion->motion_state == ARM_MOTION_HOLDING &&
                fabsf(arm_state->q_feedback_deg[0]) <= 2.0f &&
                fabsf(arm_state->q_feedback_deg[1] - 60.0f) <= 3.0f &&
                fabsf(arm_state->q_feedback_deg[2] + 95.0f) <= 3.0f) {
                g_arm_host_sim_debug.last_send_tick = now;
                g_arm_host_sim_debug.stream_start_tick = now;
                g_arm_host_sim_debug.state = ARM_HOST_SIM_STREAMING;
            }
            break;

        case ARM_HOST_SIM_STREAMING:
            if ((uint32_t)(now - g_arm_host_sim_debug.last_send_tick) >=
                ARM_REALTIME_HOST_SIM_PERIOD_MS) {
                Arm_Realtime_Cartesian_Target_s target;
                float phase = 2.0f * ARM_HOST_SIM_PI *
                    (float)((now - g_arm_host_sim_debug.stream_start_tick) %
                            ARM_REALTIME_HOST_SIM_CYCLE_MS) /
                    (float)ARM_REALTIME_HOST_SIM_CYCLE_MS;

                g_arm_host_sim_debug.last_send_tick = now;
                g_arm_host_sim_debug.phase_rad = phase;
#if ARM_REALTIME_HOST_SIM_FIXED_POINT_ENABLE != 0u
                g_arm_host_sim_debug.generated_q_deg[0] = 1.35f;
                g_arm_host_sim_debug.generated_q_deg[1] = 24.8f;
                g_arm_host_sim_debug.generated_q_deg[2] = -100.9f;
                g_arm_host_sim_debug.target_mm.x_mm =
                    ARM_REALTIME_HOST_SIM_FIXED_X_MM;
                g_arm_host_sim_debug.target_mm.y_mm =
                    ARM_REALTIME_HOST_SIM_FIXED_Y_MM;
                g_arm_host_sim_debug.target_mm.z_mm =
                    ARM_REALTIME_HOST_SIM_FIXED_Z_MM;
#else
                g_arm_host_sim_debug.generated_q_deg[0] =
                    ARM_REALTIME_HOST_SIM_CENTER_Q1_DEG +
                    ARM_REALTIME_HOST_SIM_AMPLITUDE_Q1_DEG * sinf(phase);
                g_arm_host_sim_debug.generated_q_deg[1] =
                    ARM_REALTIME_HOST_SIM_CENTER_Q2_DEG +
                    ARM_REALTIME_HOST_SIM_AMPLITUDE_Q2_DEG *
                        sinf(phase);
                g_arm_host_sim_debug.generated_q_deg[2] =
                    ARM_REALTIME_HOST_SIM_CENTER_Q3_DEG +
                    ARM_REALTIME_HOST_SIM_AMPLITUDE_Q3_DEG *
                        sinf(2.0f * phase);
                ArmForwardKinematics3DOF(
                    g_arm_host_sim_debug.generated_q_deg[0],
                    g_arm_host_sim_debug.generated_q_deg[1],
                    g_arm_host_sim_debug.generated_q_deg[2],
                    &g_arm_host_sim_debug.target_mm);
#endif

                memset(&target, 0, sizeof(target));
                target.command_id = ++g_arm_host_sim_debug.command_id;
                target.target_mm = g_arm_host_sim_debug.target_mm;
                target.max_speed_mm_s = ARM_REALTIME_HOST_SIM_SPEED_MM_S;
                target.max_acceleration_mm_s2 =
                    ARM_REALTIME_HOST_SIM_ACCEL_MM_S2;
                g_arm_host_sim_debug.last_result =
                    ArmSubmitRealtimeCartesianTarget(&target);
                g_arm_host_sim_debug.send_count++;
                if (g_arm_host_sim_debug.last_result != ARM_COMMAND_OK) {
                    g_arm_host_sim_debug.reject_count++;
                }
            }
            break;

        default:
            g_arm_host_sim_debug.state = ARM_HOST_SIM_WAIT_READY;
            break;
    }
#endif
}

static void FeetechServoSweepTestTask(void)
{
#if FEETECH_SERVO_SWEEP_TEST_ENABLE != 0u
    uint32_t now = HAL_GetTick();

    if (g_feetech_servo_test_debug.enabled == 0u) {
        return;
    }

    switch (g_feetech_servo_test_debug.state) {
        case FEETECH_TEST_STATE_WAIT_START:
            if ((uint32_t)(now - g_feetech_servo_test_debug.start_tick) >=
                FEETECH_SERVO_SWEEP_START_DELAY_MS) {
                g_feetech_servo_test_debug.state =
                    FEETECH_TEST_STATE_SEND;
            }
            break;

        case FEETECH_TEST_STATE_SEND:
            if (g_feetech_servo_test_debug.step >=
                (sizeof(feetech_servo_sweep_pos) /
                 sizeof(feetech_servo_sweep_pos[0]))) {
                g_feetech_servo_test_debug.state =
                    FEETECH_TEST_STATE_DONE;
                break;
            }
            g_feetech_servo_test_debug.target_position =
                feetech_servo_sweep_pos[g_feetech_servo_test_debug.step];
            g_feetech_servo_test_debug.target_deg =
                (uint16_t)(((uint32_t)
                    g_feetech_servo_test_debug.target_position * 240u +
                    500u) / 1000u);
            g_feetech_servo_test_debug.last_result =
                HSLServoMove(
                    g_feetech_servo_test_debug.id,
                    g_feetech_servo_test_debug.target_position,
                    FEETECH_SERVO_SWEEP_MOVE_TIME_MS);
            if (g_feetech_servo_test_debug.last_result ==
                HSL_SERVO_RESULT_OK) {
                g_feetech_servo_test_debug.send_count++;
                g_feetech_servo_test_debug.last_send_tick = now;
                g_feetech_servo_test_debug.step++;
                g_feetech_servo_test_debug.state =
                    FEETECH_TEST_STATE_WAIT_STEP;
            } else if (g_feetech_servo_test_debug.last_result ==
                       HSL_SERVO_RESULT_BUSY) {
                g_feetech_servo_test_debug.busy_count++;
            } else {
                g_feetech_servo_test_debug.error_count++;
                g_feetech_servo_test_debug.state =
                    FEETECH_TEST_STATE_ERROR;
            }
            break;

        case FEETECH_TEST_STATE_WAIT_STEP:
            if ((uint32_t)(now - g_feetech_servo_test_debug.last_send_tick) >=
                FEETECH_SERVO_SWEEP_STEP_INTERVAL_MS) {
                g_feetech_servo_test_debug.state =
                    FEETECH_TEST_STATE_SEND;
            }
            break;

        case FEETECH_TEST_STATE_DISABLED:
        case FEETECH_TEST_STATE_DONE:
        case FEETECH_TEST_STATE_ERROR:
        default:
            break;
    }
#endif
}

void all_init_Task(void)
{
	memset(&g_arm_host_sim_debug, 0, sizeof(g_arm_host_sim_debug));
	memset(&g_feetech_servo_test_debug, 0,
           sizeof(g_feetech_servo_test_debug));
	g_arm_host_sim_debug.enabled = ARM_REALTIME_HOST_SIM_ENABLE != 0u;
	protocol_init();
	ArmUsbBridgeInit();
	BuzzerInit();
	ArmInit();
	g_feetech_servo_test_debug.enabled =
        FEETECH_SERVO_SWEEP_TEST_ENABLE != 0u;
	g_feetech_servo_test_debug.id = FEETECH_SERVO_SWEEP_TEST_ID;
	g_feetech_servo_test_debug.start_tick = HAL_GetTick();
	g_feetech_servo_test_debug.state =
        g_feetech_servo_test_debug.enabled != 0u ?
        FEETECH_TEST_STATE_WAIT_START :
        FEETECH_TEST_STATE_DISABLED;
}

void all_cmd_Task(void)
{
	ArmTask();
	ArmRealtimeHostSimulatorTask();
	FeetechServoSweepTestTask();
	DMMotorControl(HAL_GetTick());
	DJIMotorControl();
}
