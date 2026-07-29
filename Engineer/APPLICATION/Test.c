#include "DJI_motor.h"
#include "dmmotor.h"
#include "Test.h"
#include "arm_config.h"
#include "math.h"
#include "string.h"

Arm_Api_Test_Debug_s g_arm_api_test_debug;
Arm_Host_Sim_Debug_s g_arm_host_sim_debug;

#define ARM_HOST_SIM_PI 3.14159265358979323846f

/*
 * 用公开坐标接口回放一次打点结果。
 * 只有机械臂完成初始化、三台电机在线且运动学有效后才提交；提交成功或被
 * 明确拒绝后不再重试，避免1ms任务重复发送同一条命令。
 */
static void ArmApiPointTestTask(void)
{
#if ARM_API_POINT_TEST_ENABLE != 0u
    const Arm_State_s *state = ArmGetState();
    Arm_Cartesian_Command_s command;

    if (g_arm_api_test_debug.submitted) {
        return;
    }
    if (state->mode != ARM_MODE_READY || !state->kinematics_valid ||
        !state->motor_online[0] || !state->motor_online[1] ||
        !state->motor_online[2]) {
        g_arm_api_test_debug.result = ARM_COMMAND_NOT_READY;
        return;
    }

    memset(&command, 0, sizeof(command));
    command.command_id = 1u;
    command.control_point = ARM_CONTROL_POINT_WRIST_CENTER;
    command.move_type = ARM_MOVE_LINEAR;
    command.target_mm = g_arm_api_test_debug.target_mm;
    command.max_speed_mm_s = ARM_API_POINT_TEST_SPEED_MM_S;
    command.tool_pitch_valid = 0u;

    g_arm_api_test_debug.result = ArmSubmitCartesianCommand(&command);
    g_arm_api_test_debug.submit_count++;
    if (g_arm_api_test_debug.result != ARM_COMMAND_NOT_READY &&
        g_arm_api_test_debug.result != ARM_COMMAND_BUSY) {
        g_arm_api_test_debug.submitted = 1u;
    }
#endif
}

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

void all_init_Task(void)
{
	memset(&g_arm_api_test_debug, 0, sizeof(g_arm_api_test_debug));
	memset(&g_arm_host_sim_debug, 0, sizeof(g_arm_host_sim_debug));
	g_arm_api_test_debug.enabled = ARM_API_POINT_TEST_ENABLE != 0u;
	g_arm_host_sim_debug.enabled = ARM_REALTIME_HOST_SIM_ENABLE != 0u;
	g_arm_api_test_debug.target_mm.x_mm = ARM_API_POINT_TEST_X_MM;
	g_arm_api_test_debug.target_mm.y_mm = ARM_API_POINT_TEST_Y_MM;
	g_arm_api_test_debug.target_mm.z_mm = ARM_API_POINT_TEST_Z_MM;
	ArmInit();
}

void all_cmd_Task(void)
{
	ArmTask();
	ArmApiPointTestTask();
	ArmRealtimeHostSimulatorTask();
	DMMotorControl(HAL_GetTick());
	DJIMotorControl();
}
