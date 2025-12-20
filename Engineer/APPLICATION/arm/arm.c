#include "arm.h"
#include "dmmotor.h"
#include "DJI_motor.h"
#include "remote.h"
#include "bsp_dwt.h"
#include "robot_def.h"
#include "chassis.h"
#include "relay.h"

//static DJIMotor_Instance *lift, *yaw;
//static Relay_Instance *raw;

//void ArmInit(void)
//{
////	Relay_Init_Config_s raw_config = {
////    .gpio1     = GPIOE,         // C3
////		.gpio2     = GPIOE,         // C4
////    .gpio_pin1 = GPIO_PIN_9,    
////    .gpio_pin2 = GPIO_PIN_11,    
////    .state    = 0   // 2 up 1 down	0 off
////};
////	 
////		raw=RelayInit(&raw_config);
//	 

//		    Motor_Init_Config_s lift_config = {
//        .can_init_config = {
//            .can_handle = &hcan2,
//            .tx_id      = 5,
//        },
//        .controller_param_init_config = {
//            .speed_PID = {
//                .Kp = 15,  // 10
//                .Ki = 2, // 1
//                .Kd = 0,
//                // .CoefA         = 0.2,
//                // .CoefB         = 0.3,
//                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//                .IntegralLimit = 10000,
//                .MaxOut        = 15000,
//            },
//            .current_PID = {
//                .Kp            = 0.7, // 0.7
//                .Ki            = 0.1, // 0.1
//                .Kd            = 0,
//                .Improve       = PID_Integral_Limit,
//                .IntegralLimit = 10000,
//                .MaxOut        = 15000,
//                // .DeadBand      = 0.1,
//            },
//        },
//        .controller_setting_init_config = {
//            .speed_feedback_source = MOTOR_FEED,
//            .outer_loop_type       = SPEED_LOOP,
//            .close_loop_type       = CURRENT_LOOP | SPEED_LOOP,
//            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL, 
//            .feedforward_flag      = CURRENT_AND_SPEED_FEEDFORWARD,
//        },
//        .motor_type = M3508};
//    lift = DJIMotorInit(&lift_config);

//		    Motor_Init_Config_s yaw_config = {
//        .can_init_config = {
//            .can_handle = &hcan2,
//            .tx_id      = 4,
//        },
//        .controller_param_init_config = {
//            .speed_PID = {
//                .Kp            = 15,  // 10
//                .Ki            = 2, // 1
//                .Kd            = 1,
//                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//                .IntegralLimit = 10000,
//                .MaxOut        = 15000,
//            },
//            .current_PID = {
//                .Kp            = 1,  // 0.7
//                .Ki            = 0.02f, // 0.1
//                .Kd            = 0,
//                .Improve       = PID_Integral_Limit,
//                .IntegralLimit = 10000,
//                .MaxOut        = 15000,
//            },
//        },
//        .controller_setting_init_config = {
//            .speed_feedback_source = MOTOR_FEED,
//            .outer_loop_type    = SPEED_LOOP, 
//            .close_loop_type    = CURRENT_LOOP | SPEED_LOOP ,
//            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // ！！！ 只有在只对速度闭环时才能反向 ！！！
//        },
//        .motor_type = M2006};
//    yaw = DJIMotorInit(&yaw_config);
//				
//	DJIMotorEnable(lift);
//	DJIMotorEnable(yaw);
//	 	
//}

//	
//void ArmTask()
//{
////	if(rc_cmd->rc.switch_right ==1) RelayUp(raw);
////	if(rc_cmd->rc.switch_right ==2) RelayOff(raw);
////	if(rc_cmd->rc.switch_right ==3) RelayDown(raw);
//	DJIMotorSetRef(lift,((float)rc_cmd->rc.rocker_r1/660)*50000);
//  DJIMotorSetRef(yaw,((float)rc_cmd->rc.rocker_r1/660)*50000);

// }

