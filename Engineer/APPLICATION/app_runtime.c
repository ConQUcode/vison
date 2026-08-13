#include "Test.h"

#include "stm32f4xx_hal.h"

#if CHASSIS_ONE_METER_TEST_ONLY != 0u
#include "DJI_motor.h"
#include "buzzer.h"
#include "chassis.h"
#include "ins_task.h"
#include "protocol.h"
#include "usb.h"
#elif HUANER_SERVO_DUAL_FEEDBACK_TEST_ONLY != 0u
#include "hsl_servo.h"
#else
#include "DJI_motor.h"
#include "arm.h"
#include "buzzer.h"
#include "dmmotor.h"
#include "fruit_usb_bridge.h"
#include "protocol.h"
#include "usb.h"
#endif

#include <math.h>
#include <string.h>

#define HUANER_SERVO1_ID                     1u
#define HUANER_SERVO2_ID                     2u
#define HUANER_SERVO_QUERY_INTERVAL_MS       25u
#define HUANER_SERVO_POSITION_TO_DEG      0.24f

Huaner_Dual_Servo_Debug_s g_huaner_dual_servo_debug;

#if CHASSIS_ONE_METER_TEST_ONLY != 0u
/* INS_Init()返回的共享姿态快照，只由底盘读取，不在底盘任务内重复解算。 */
static attitude_t *chassis_test_imu;
#endif

#if HUANER_SERVO_DUAL_FEEDBACK_TEST_ONLY != 0u
static uint8_t huaner_next_query_id;
static uint32_t huaner_next_query_tick;

static void HuanerDualServoFeedbackTask(void)
{
    HSLServo_Status_s servo1_status;
    HSLServo_Status_s servo2_status;
    uint32_t now_ms = HAL_GetTick();

    memset(&servo1_status, 0, sizeof(servo1_status));
    memset(&servo2_status, 0, sizeof(servo2_status));
    HSLServoTask(now_ms);

    /*
     * 两个ID交替单独查询，避免一台断线时双ID合并应答校验失败，
     * 从而把另一台实际在线的舵机也显示为通信失败。
     */
    if ((int32_t)(now_ms - huaner_next_query_tick) >= 0 &&
        HSLServoRequestPosition(huaner_next_query_id) ==
            HSL_SERVO_RESULT_OK) {
        huaner_next_query_id =
            huaner_next_query_id == HUANER_SERVO1_ID ?
            HUANER_SERVO2_ID : HUANER_SERVO1_ID;
        huaner_next_query_tick = now_ms +
            HUANER_SERVO_QUERY_INTERVAL_MS;
    }

    (void)HSLServoGetStatus(HUANER_SERVO1_ID, &servo1_status);
    (void)HSLServoGetStatus(HUANER_SERVO2_ID, &servo2_status);

    g_huaner_dual_servo_debug.servo1_communication_ok =
        servo1_status.online != 0u &&
        servo1_status.feedback_valid != 0u;
    g_huaner_dual_servo_debug.servo2_communication_ok =
        servo2_status.online != 0u &&
        servo2_status.feedback_valid != 0u;

    if (g_huaner_dual_servo_debug.servo1_communication_ok != 0u) {
        g_huaner_dual_servo_debug.servo1_position =
            servo1_status.feedback_position;
        g_huaner_dual_servo_debug.servo1_angle_deg =
            (float)servo1_status.feedback_position *
            HUANER_SERVO_POSITION_TO_DEG;
    } else {
        g_huaner_dual_servo_debug.servo1_angle_deg = NAN;
    }
    if (g_huaner_dual_servo_debug.servo2_communication_ok != 0u) {
        g_huaner_dual_servo_debug.servo2_position =
            servo2_status.feedback_position;
        g_huaner_dual_servo_debug.servo2_angle_deg =
            (float)servo2_status.feedback_position *
            HUANER_SERVO_POSITION_TO_DEG;
    } else {
        g_huaner_dual_servo_debug.servo2_angle_deg = NAN;
    }
}
#endif

void all_init_Task(void)
{
    memset(&g_huaner_dual_servo_debug, 0,
           sizeof(g_huaner_dual_servo_debug));
    g_huaner_dual_servo_debug.servo1_angle_deg = NAN;
    g_huaner_dual_servo_debug.servo2_angle_deg = NAN;
#if HUANER_SERVO_DUAL_FEEDBACK_TEST_ONLY != 0u
    if (HSLServoInit() != 0u) {
        huaner_next_query_id = HUANER_SERVO1_ID;
        huaner_next_query_tick = HAL_GetTick();
        g_huaner_dual_servo_debug.driver_initialized = 1u;
    }
#elif CHASSIS_ONE_METER_TEST_ONLY != 0u
    /*
     * 底盘1 m测试初始化：不注册机械臂、达妙电机或USART6工具舵机。
     * INS初始化包含BMI088启动与初始姿态计算，必须在调度器启动前只调用一次。
     */
    USB_Init();
    protocol_init();
    BuzzerInit();
    chassis_test_imu = INS_Init();
    (void)ChassisInit(chassis_test_imu);
#else
    USB_Init();
    protocol_init();
    FruitUsbBridgeInit();
    BuzzerInit();
    ArmInit();
#endif
}

void all_cmd_Task(void)
{
#if HUANER_SERVO_DUAL_FEEDBACK_TEST_ONLY != 0u
    HuanerDualServoFeedbackTask();
#elif CHASSIS_ONE_METER_TEST_ONLY != 0u
    uint32_t now_ms = HAL_GetTick();

    /* ChassisTask内部按5 ms运行；DJI电机电流环保持1 kHz服务。 */
    ChassisTask(now_ms);
    DJIMotorControl();
#else
    ArmTask();
    DMMotorControl(HAL_GetTick());
    DJIMotorControl();
#endif
}
