#include "DJI_motor.h"
#include "Test.h"
#include "arm_config.h"
#include "string.h"

Arm_Api_Test_Debug_s g_arm_api_test_debug;

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

void all_init_Task(void)
{
	memset(&g_arm_api_test_debug, 0, sizeof(g_arm_api_test_debug));
	g_arm_api_test_debug.enabled = ARM_API_POINT_TEST_ENABLE != 0u;
	g_arm_api_test_debug.target_mm.x_mm = ARM_API_POINT_TEST_X_MM;
	g_arm_api_test_debug.target_mm.y_mm = ARM_API_POINT_TEST_Y_MM;
	g_arm_api_test_debug.target_mm.z_mm = ARM_API_POINT_TEST_Z_MM;
	ArmInit();
}

void all_cmd_Task(void)
{
	ArmTask();
	ArmApiPointTestTask();
	DJIMotorControl();
}
