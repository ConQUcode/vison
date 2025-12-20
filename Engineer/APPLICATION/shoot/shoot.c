#include "shoot.h"
#include "dmmotor.h"
#include "DJI_motor.h"
#include "remote.h"
#include "bsp_dwt.h"
#include "robot_def.h"
#include "chassis.h"
#include "relay.h"
#include "cmsis_os.h"
#include "nac.h"


static DJIMotor_Instance *motor_3508_1, *motor_3508_2, *motor_3508_3,*motor_3508_4;
static float totalangle_1 = 0;
static float totalangle = 0;
static float set=0;
static int8_t is_init = 0;
static float angle = 100;
static PID_Instance gains_pid;
static float Ang,Vec,control = 0 ;
static Relay_Instance *push;
Nac_Recv_s	*nac_ctrl;

void ShootInit()
{
	
	
	Relay_Init_Config_s push_config = {
    .gpio1     = GPIOE,         // C1
		.gpio2     = GPIOE,         // C2
    .gpio_pin1 = GPIO_PIN_9,    
    .gpio_pin2 = GPIO_PIN_11,  
    .state    = 0   //  2 up 1 down	0 off
	};
	 push = RelayInit(&push_config);

	PID_Init_Config_s gains_pid_config ={
	      .Kp                = 15, // 6
        .Ki                = 2,
        .Kd                = 0.5, // 0.5
        .DeadBand          = 0.5,
        .CoefA             = 0.2,
        .CoefB             = 0.3,
        .Improve           = PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_DerivativeFilter | PID_Derivative_On_Measurement | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit     = 500, // 200
        .MaxOut            = 10000,
        .Derivative_LPF_RC = 0.01, // 0.01
	};
	PIDInit(&gains_pid,&gains_pid_config);

	
	Motor_Init_Config_s M3508_zhuan_config ={
	.can_init_config={  
	.can_handle =&hcan2,
	.tx_id=1,
	},
	.controller_param_init_config={
	 .speed_PID={
								.Kp=7,
								.Ki=0.7,
								.Kd=0.2,
								.Improve= PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
								.IntegralLimit = 1000,
								.MaxOut        = 15000,
						},
	.current_PID = {
                .Kp            = 0.7, // 0.7
                .Ki            = 0.1, // 0.1
                .Kd            = 0,
                .Improve       = PID_Integral_Limit,
                .IntegralLimit = 1000,
                .MaxOut        = 15000,
                // .DeadBand      = 0.1,
            },
        },
        .controller_setting_init_config = {
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type       = CURRENT_LOOP | SPEED_LOOP  ,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL, 
            .feedforward_flag      = CURRENT_AND_SPEED_FEEDFORWARD,
        },
        .motor_type = M3508};
	
    motor_3508_1 =DJIMotorInit(&M3508_zhuan_config);
				
    M3508_zhuan_config.can_init_config.tx_id=2;
		motor_3508_2 =DJIMotorInit(&M3508_zhuan_config);
    M3508_zhuan_config.can_init_config.tx_id=4;
		M3508_zhuan_config.controller_param_init_config.speed_PID.Kp=3;
				M3508_zhuan_config.controller_param_init_config.speed_PID.Ki=2.2;
				M3508_zhuan_config.controller_param_init_config.speed_PID.Kd=0.001;
    motor_3508_4 =DJIMotorInit(&M3508_zhuan_config);

		

Motor_Init_Config_s M3508_jiaodu_config = {
			.can_init_config = {
					.can_handle = &hcan2,
					.tx_id      = 3,
			},
			.controller_param_init_config = {
					.angle_PID = {
                .Kp                = 5,
                .Ki                = 0.05,
                .Kd                = 0.01,
                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ErrorHandle,
                .IntegralLimit     = 5000,
                .MaxOut            = 16000,
                .Derivative_LPF_RC = 0.01,
            },
					.speed_PID = {
							.Kp = 5,  // 10
							.Ki = 0.2, // 1
							.Kd = 0.02,
							// .CoefA         = 0.2,
							// .CoefB         = 0.3,
							.Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
							.IntegralLimit = 5000,
							.MaxOut        = 10000,
					},
					.current_PID = {
							.Kp            = 0.3, // 0.7
							.Ki            = 0.1, // 0.1
							.Kd            = 0,
							.Improve       = PID_Integral_Limit,
							.IntegralLimit = 1000,
							.MaxOut        = 6000,
							// .DeadBand      = 0.1,
					},
					.other_angle_feedback_ptr = &totalangle,
			},
					.controller_setting_init_config = {
					.angle_feedback_source = OTHER_FEED,
					.speed_feedback_source = MOTOR_FEED,
					.outer_loop_type       = ANGLE_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
					.close_loop_type       = CURRENT_LOOP | SPEED_LOOP |ANGLE_LOOP,
					.motor_reverse_flag    = MOTOR_DIRECTION_NORMAL, 
					.feedforward_flag      = FEEDFORWARD_NONE,
			},
			.motor_type = M3508};
				
		
 	
			motor_3508_3 =  DJIMotorInit(&M3508_jiaodu_config);
		DJIMotorOuterLoop(motor_3508_3, ANGLE_LOOP);
			
			
		}			
//  Motor_Init_Config_s M3508_angle_config ={
//		.can_init_config={  
//		.can_handle =&hcan2,
//		.tx_id=3,
//		},
//	.controller_param_init_config={
//	 .speed_PID={
//								.Kp=10,
//								.Ki=1,
//								.Kd=0.2,
//								.Improve= PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
//								.IntegralLimit = 1000,
//								.MaxOut        = 15000,
//							},
//	.current_PID = {
//                .Kp            = 0.5, // 0.7
//                .Ki            = 0.01, // 0.1
//                .Kd            = 0,
//                .Improve       = PID_Integral_Limit,
//                .IntegralLimit = 1000,
//                .MaxOut        = 15000,
//                // .DeadBand      = 0.1,
//								},
//	.angle_PID = {
//                .Kp            = 0.5, // 0.7
//                .Ki            = 0.01, // 0.1
//                .Kd            = 0,
//                .Improve       = PID_Integral_Limit,
//                .IntegralLimit = 1000,
//                .MaxOut        = 15000,
//                // .DeadBand      = 0.1,
//								},
//        },
//        .controller_setting_init_config = {
//            .speed_feedback_source = MOTOR_FEED,
//            .outer_loop_type       = ANGLE_LOOP, 
//            .close_loop_type       = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP ,
//            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL, 
//            .feedforward_flag      = CURRENT_AND_SPEED_FEEDFORWARD,
//        },
//        .motor_type = M3508};
//			motor_3508_3 =DJIMotorInit(&M3508_angle_config);
//}



void gains_pid_schedule(DJIMotor_Instance*motor){
if(motor->measure.real_current>3000||motor->measure.real_current<-3000){
	motor->motor_controller.speed_PID.Kp  = (motor->motor_controller.speed_PID.Kp < gains_pid.Kp)? motor->motor_controller.speed_PID.Kp*0.9+gains_pid.Kp*0.1 : gains_pid.Kp;
	motor->motor_controller.speed_PID.Ki =(motor->motor_controller.speed_PID.Ki < gains_pid.Ki)? motor->motor_controller.speed_PID.Ki*0.9+gains_pid.Ki*0.1:gains_pid.Ki;
	motor->motor_controller.speed_PID.Kd =(motor->motor_controller.speed_PID.Kd < gains_pid.Kd)?motor->motor_controller.speed_PID.Kd*0.9+gains_pid.Kd*0.1:gains_pid.Kd;
}
else {
		motor->motor_controller.speed_PID.Kp = (motor->motor_controller.speed_PID.Kp >4)     ? (motor->motor_controller.speed_PID.Kp  -  0.1* gains_pid.Kp)/0.9 : 4;
		motor->motor_controller.speed_PID.Ki = (motor->motor_controller.speed_PID.Ki >0.025) ? (motor->motor_controller.speed_PID.Ki  -  0.1* gains_pid.Ki)/0.9 : 0.025;
		motor->motor_controller.speed_PID.Kd = (motor->motor_controller.speed_PID.Kd >0.02)  ? (motor->motor_controller.speed_PID.Kd  -  0.1* gains_pid.Kd)/0.9 : 0.0;	
  }
}
/**
 * @brief 使舵电机角度最小旋转，取优弧，防止电机旋转不必要的行程
 *          例如：上次角度为0，目标角度为135度，
 *          电机会选择逆时针旋转至-45度，而不是顺时针旋转至135度，
 *          两个角度都会让轮电机处于同一平行线上
 *
 * @param angle 目标角度
 * @param last_angle 上次角度
 *
 */
static void MinmizeRotation(int *angle, uint16_t *last_angle, float *speed)
{
    float rotation = *angle - *last_angle;
    if (rotation > 3000) {
        *angle -= 8191;
    } else if (rotation < -3000) {
        *angle += 8191;
    }
}

void gains(){
		gains_pid_schedule(motor_3508_1);
		gains_pid_schedule(motor_3508_2);

}

static void ShootangleInit(){
//	if(!is_init){
//		
//		if (abs(motor_3508_3->measure.real_current) <= 4500 && is_init == 0) {
//        DJIMotorEnable(motor_3508_3);
//        DJIMotorOuterLoop(motor_3508_3, SPEED_LOOP);
//        DJIMotorSetRef(motor_3508_3, -1000);
//    } else if ((abs(motor_3508_3->measure.real_current) > 4500) && is_init == 0) {
//				HAL_Delay(5);
//			 if ((motor_3508_3->measure.real_current < -4500) && is_init == 0){	// Change the logic to reset using a microswitch
//					DJIMotorOuterLoop(motor_3508_3, ANGLE_LOOP);
//					DJIMotorReset(motor_3508_3);
//					DJIMotorStop(motor_3508_3);
//					is_init = 1;
//			 }
//    }

//	}
}

void Shoot_speed(){
}

void Shoot_angle(){
}


void Shoot_task(){
	DJIMotorEnable(motor_3508_1);
	DJIMotorEnable(motor_3508_2);
				gains();
				DJIMotorSetRef(motor_3508_1,-Vec);
				DJIMotorSetRef(motor_3508_2,-Vec);
//				DJIMotorSetRef(motor_3508_3,Ang);
//				DJIMotorControl();
}

void ShootTask(){
	
//    	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)==GPIO_PIN_SET){
//			     DJIMotorReset(motor_3508_3);
//			}
//			
			
			if(rc_cmd->rc.rocker_r_/660 >0.3 || rc_cmd->rc.rocker_r_/660 <-0.3 ){
			set+=(float)rc_cmd->rc.rocker_r_/660;
			}
			
			totalangle = motor_3508_3->measure.total_angle;
			DJIMotorSetRef(motor_3508_3,set);
			if(rc_cmd->rc.rocker_r1>=200){
				RelayUp(push);
			}
			else if(rc_cmd->rc.rocker_r1<=-200){
				RelayDown(push);
			}
			else if(rc_cmd->rc.rocker_r1>=-200 || rc_cmd->rc.rocker_r1<=200){
				RelayOff(push);
			}
			
			static int state2=0;
			state2 = rc_cmd->rc.switch_right;
				switch(state2){                                                                                                                
					  case 2 :	/*停止一切任务*/
							Vec=30000;
							Shoot_task();
						break;
					case 3:		/*投篮后的恢复*/
					  Vec=31500;
							Shoot_task();
						break;
					case 1:  /*投篮任务*/
						Vec=32500;
							Shoot_task();
						break;
					default:break;
				}			
				
			}
void ShootTask2(){
	Vec=0;
								if(rc_cmd->rc.rocker_r_>=400){
									Vec+=500;
								}
								else if(rc_cmd->rc.rocker_r_<=-400){
									Vec-=500;
								}
//    	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)==GPIO_PIN_SET){
//			     DJIMotorReset(motor_3508_3);
//			}
//			
//		  if(rc_cmd->rc.rocker_r_/660 >0.3 || rc_cmd->rc.rocker_r_/660 <-0.3 ){
//			set+=(float)rc_cmd->rc.rocker_r_/660;
//			}
//			totalangle = motor_3508_3->measure.total_angle;
//			DJIMotorSetRef(motor_3508_3,set);
			
			if(rc_cmd->rc.rocker_r1>=200){
				RelayUp(push);
			}
			else if(rc_cmd->rc.rocker_r1<=-200){
				RelayDown(push);
			}
			else if(rc_cmd->rc.rocker_r1>=-200 || rc_cmd->rc.rocker_r1<=200){
				RelayOff(push);
			}
//			uint16_t chi=rc_cmd->rc.rocker_r1/600*5000;
//			if(rc_cmd->rc.rocker_r1>=100){
//				DJIMotorEnable(motor_3508_4);
////				DJIMotorSetRef(motor_3508_4,chi);
//				DJIMotorSetRef(motor_3508_4,8000);
//			}
//			else if(rc_cmd->rc.rocker_r1<=-100){
//				DJIMotorEnable(motor_3508_4);
////				DJIMotorSetRef(motor_3508_4,chi);
//				DJIMotorSetRef(motor_3508_4,-8000);
//			}
//			else if(rc_cmd->rc.rocker_r1>=-100 || rc_cmd->rc.rocker_r1<=100){
//		   DJIMotorStop(motor_3508_4);
//			}
			static int state2=0;
			state2 = rc_cmd->rc.switch_right;
				switch(state2){                                                                                                                   
					  case 2 :	/*停止一切任务*/
							Vec+=40000;		
							Shoot_task();
						break;
					case 3:		/*投篮后的恢复*/
							Vec+=43500;
							Shoot_task();
						break;
					case 1:  /*投篮任务*/
							Vec+=47500;
							Shoot_task();
						break;
					default:break;
				}			
			}

			
void Shoot_Stop(){
					DJIMotorStop(motor_3508_1);
					DJIMotorStop(motor_3508_2);	
					DJIMotorSetRef(motor_3508_3,totalangle);
					RelayOff(push);	
					DJIMotorControl();
}
	

