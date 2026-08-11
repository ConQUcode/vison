#include "catch.h"
#include "daemon.h"
#include "remote.h"
#include "feite_motor.h"
#include "bsp_dwt.h"
#include "DJI_motor.h"
#include "tim.h"
#include "cmsis_os.h"
#include "hsl_servo.h"

FTMotor_instance *FT_1,*FT_2,*FT_3,*FT_4;
static DJIMotor_Instance *DJM2006,*DJM3508;
//    static uint8_t is_catching = 0;
//    static uint32_t catch_start_time = 0;
//    static uint8_t is_homed = 0; // ��������¼�Ƿ��Ѿ���ɵ��������������
extern RC_ctrl_t *rc_cmd;
int IR_sensor_level;
float control;
int a=0;
uint8_t ID[] = {1, 2};
int16_t Pos[] = {2000, 2000};    // 目标位置
uint16_t Speed[] = {1000, 1000}; // 运行速度
uint8_t ACC[] = {50, 50};       // 加速度
uint16_t Torque[] = {1000, 1000};// 最大输出扭矩

//void duoji_init(){

//	FTMotor_Init_Config_s ftmotor_config ={
//		.usart_init_config = {
//			.usart_handle = &huart1,
//		},
//		.motor_set = {
//			.ID = 1,
//			.MemAddr = SMS_STS_ACC,  // MemAddr will be set dynamically by WriteAccEx() and WritePosEx()
//      .Fun = INST_WRITE,
//		},
//		.motor_ref = {
//			.Position =  0,
//			.Speed =  2250,      // Increased from 2250 to 4000 (stronger force for heavy load)
//			.ACC = 50,          // Increased from 50 to 100 (faster acceleration)
//		},
//	};
//	FT_1 = FTMotorInit(&ftmotor_config);
//  
//	ftmotor_config.motor_set.ID = 2;
//  FT_2 = FTMotorInit(&ftmotor_config);

//	ftmotor_config.motor_set.ID = 3;
//  FT_3 = FTMotorInit(&ftmotor_config);
//  
//  ftmotor_config.motor_set.ID = 4;
//  FT_4 = FTMotorInit(&ftmotor_config);

//}



void dianji_init(){

			Motor_Init_Config_s dianji_config = {
			.can_init_config = {
					.can_handle = &hcan1,
					.tx_id      = 1,
			},
			.controller_param_init_config = {
				.angle_PID = {
                .Kp                = 3,
                .Ki                = 1.5,
                .Kd                = 0.1,
                .Improve           = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ErrorHandle),
                .IntegralLimit     = 60000,
                .MaxOut            = 40000,
                .Derivative_LPF_RC = 0.01,
            },
					.speed_PID = {
							.Kp = 3,  // 7
							.Ki = 0.005,// 0.01
							.Kd = 0.004,//0.008
							// .CoefA         = 0.2,
							// .CoefB         = 0.3,
							.Improve       = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
							.IntegralLimit = 10000,
							.MaxOut        = 15000,
					},
				
					.current_PID = {
							.Kp            = 0.2, // 0.4
							.Ki            = 0.0006, // 0.001
							.Kd            = 0,
							.Improve       = PID_Integral_Limit,
							.IntegralLimit = 10000,
							.MaxOut        = 15000,
							// .DeadBand      = 0.1,
					},
			},
			.controller_setting_init_config = {
					.speed_feedback_source = MOTOR_FEED,
					.outer_loop_type       = ANGLE_LOOP, 
					.close_loop_type       = (Closeloop_Type_e)(CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP),
					.motor_reverse_flag    = MOTOR_DIRECTION_NORMAL, 
					.feedforward_flag      = CURRENT_AND_SPEED_FEEDFORWARD,
			},
			.motor_type = M2006
			};
      DJM2006 = DJIMotorInit(&dianji_config);
	
			
//M3508
			Motor_Init_Config_s M3508_config = {
    .can_init_config = {
        .can_handle = &hcan1,
        .tx_id      = 2,
    },
    .controller_param_init_config = {
				.angle_PID = {
            .Kp                = 10	,
		  			.Ki                = 0.5,
            .Kd                = 0.1,
            .Improve           = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ErrorHandle),
            .IntegralLimit     = 50000,
            .MaxOut            = 50000,
            .Derivative_LPF_RC = 0.01,
            },
        .speed_PID = {
            .Kp = 4,
            .Ki = 0.025,
            .Kd = 0.02,
            .Improve       = (PID_Improvement_e)(PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement),
            .IntegralLimit = 10000,
            .MaxOut        = 15000,
        },
        .current_PID = {
            .Kp            = 0.5,
            .Ki            = 0.01,
            .Kd            = 0,
            .Improve       = PID_Integral_Limit,
            .IntegralLimit = 10000,
            .MaxOut        = 50000,
        },
    },
    .controller_setting_init_config = {
        .speed_feedback_source = MOTOR_FEED,
        .outer_loop_type       = ANGLE_LOOP, 
        .close_loop_type       = (Closeloop_Type_e)(CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP),
        .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL, 
        .feedforward_flag      = CURRENT_AND_SPEED_FEEDFORWARD,
    },
    .motor_type = M3508
};
			DJM3508 = DJIMotorInit(&M3508_config);
}


#if 0
static void LiftInit(void) {
	
    // ״̬ A����ʼ�� M2006 (IR ����������)
    if (!is_init_2006) {
			DJIMotorSetRef(DJM3508,15000);
        if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == 1) {
            DJIMotorEnable(DJM2006);
            DJIMotorOuterLoop(DJM2006, SPEED_LOOP);
            DJIMotorSetRef(DJM2006, -4000); // �ٶȲ�Ҫ̫��
        } else {
            osDelay(10); // ����
            if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == 0) {
                DJIMotorStop(DJM2006);
                DJIMotorReset(DJM2006);
                DJIMotorOuterLoop(DJM2006, ANGLE_LOOP);
                DJIMotorSetRef(DJM2006, 0);
                is_init_2006 = 1;
            }
        }
        return; // M2006 û�㶨ǰ���������
    }

    // ״̬ B����ʼ�� M3508 (������ת����)
    if (!is_init_3508) {
        //�����ջ�
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_RESET);
        // �л����ٶȻ�ȥײ��λ
        DJIMotorEnable(DJM3508);
        DJIMotorOuterLoop(DJM3508, SPEED_LOOP);
        DJIMotorSetRef(DJM3508, -3000); // �ý�С���ٶ�ȥײ

        // ������β�����������ֹ˲ʱ��������
        // M3508 ��ת����ͨ���� 8000~10000 ���ϣ�4500 �Ե�
        if (abs(DJM3508->measure.real_current) > 4200) {
              osDelay(5);
					if (abs(DJM3508->measure.real_current) > 4200) {
            DJIMotorStop(DJM3508);
            DJIMotorReset(DJM3508);
            DJIMotorOuterLoop(DJM3508, ANGLE_LOOP);
            DJIMotorSetRef(DJM3508, 0); // ��������λ
            is_init_3508 = 1;
					}
        }
    }
}
#endif

void catch_init(void){
	//DWT_Init(168);
//  duoji_init();

  dianji_init();
}

//צ�ӱպ�
void feite_catch(){

  // FT_1 ->motor_ref.Position = 800;
	// FT_2 ->motor_ref.Position = 800;
  // FT_3 ->motor_ref.Position = 400;
	// FTMotorControl();

	//id,位置，速度，加速度，力矩
WritePosEx2(1, 875, 500, 20, 1500);
WritePosEx2(2, 900, 500, 20, 1000);
WritePosEx2(3, 590, 500, 20, 1500);

}
//צ���ſ�(С�Ƕ�)
void feite_putdown(){
  FT_1 ->motor_ref.Position = 500;
	FT_2 ->motor_ref.Position = 500;
  FT_3 ->motor_ref.Position = 500;
//	FTMotorControl();

}

void feite_open(){
WritePosEx2(1, 1500, 500, 20, 1500);
WritePosEx2(2, 1500, 500, 20, 1000);
WritePosEx2(3, 1100, 500, 20, 1500);

//  FT_1 ->motor_ref.Position = 800;
//	FT_2 ->motor_ref.Position = 800;
//  FT_3 ->motor_ref.Position = 800;
//	FTMotorControl();

}



void catch_all(void){

static uint32_t catch_start_time = 0;
static uint8_t is_timing = 0; // 标志位：是否正在计时
if(rc_cmd->rc.switch_left == 2){
const uint8_t rocker_pressed = ((int16_t)rc_cmd->rc.rocker_r1 < -500);
    DJIMotorSetRef(DJM3508, 1000);
    if (DJM3508->measure.total_angle > 900) {
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,GPIO_PIN_SET);
        DJIMotorSetRef(DJM2006, 9000);
    }
if (rocker_pressed) {

    // 2006 到位后开始计时并夹取
    if (DJM2006->measure.total_angle > 8500) {
        if (is_timing == 0) {
            catch_start_time = HAL_GetTick();
            is_timing = 1;
            feite_catch(); // 动作开始
        }

        // 判断时间是否超过 1500ms
        if ((uint32_t)(HAL_GetTick() - catch_start_time) > 1500) {
            DJIMotorSetRef(DJM3508, 10000);
        }
    }
} else {
    // 摇杆松开时，重置标志位，以便下次重新计时
    is_timing = 0;
    DJIMotorSetRef(DJM3508, 1000); // 回到初始状态（根据需求决定）
    feite_open();
}
}

	}
