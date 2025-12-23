#include "chassis.h"
#include "DJI_motor.h"
#include "bsp_dwt.h"
#include "remote.h"
#include "ins_task.h"
#include "user_lib.h"
#include "robot_def.h"
#include "nac.h"
#include "usb.h" // 引入USB模块

static DJIMotor_Instance *chassis_lf, *chassis_lb, *chassis_rf, *chassis_rb;
RC_ctrl_t *rc_cmd;
static DJIMotor_Instance *motor_lf, *motor_rf, *motor_lb, *motor_rb;                                     // left right forward back
static DJIMotor_Instance *motor_steering_lf, *motor_steering_rf, *motor_steering_lb, *motor_steering_rb; // 6020电机 
static PID_Instance chassis_follow_pid;  // 底盘跟随PID
static float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出,待进行限幅
static float at_lf, at_rf, at_lb, at_rb; // 底盘的角度解算后的临时输出,待进行限幅

Chassis_Ctrl_Cmd_s chassis_ctrl_cmd;

void ChassisInit()
{
	    USB_Init(); // 初始化USB模块
		rc_cmd = RemoteControlInit(&huart3);
	// 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp            = 4, // 3
                .Ki            = 0.2, // 0.5
                .Kd            = 0.005,   // 0
                .IntegralLimit = 3000,//5000
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 10000,
            },
            .current_PID = {
                .Kp            = 1, // 1
                .Ki            = 0.01,   // 0
                .Kd            = 0,
                .IntegralLimit = 3000,//3000
                .Improve       = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut        = 10000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = SPEED_LOOP,
            .close_loop_type       = CURRENT_LOOP | SPEED_LOOP,
        },
        .motor_type = M3508,
    };
    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    chassis_motor_config.can_init_config.tx_id                             = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb                                                               = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id                             = 2;
		chassis_motor_config.controller_param_init_config.speed_PID.Kp         =2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb                                                               = DJIMotorInit(&chassis_motor_config);

    // 6020电机初始化
    Motor_Init_Config_s chassis_motor_steering_config = {
        .can_init_config.can_handle   = &hcan1,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp                = 12,
                .Ki                = 0.2,
                .Kd                = 0,
                .CoefA             = 5,
                .CoefB             = 0.1,
                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ChangingIntegrationRate,
                .IntegralLimit     = 1000,
                .MaxOut            = 16000,
                .Derivative_LPF_RC = 0.001,
                .DeadBand          = 0.5,
            },
            .speed_PID = {
                .Kp            = 40,
                .Ki            = 3,
                .Kd            = 0,
                .Improve       = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate | PID_OutputFilter,
                .IntegralLimit = 4000,
                .MaxOut        = 20000,
                .Output_LPF_RC = 0.03,
            },
//						.angle_PID = {
//                .Kp                = 15,
//                .Ki                = 0.5,
//                .Kd                = 0.1,
//                .CoefA             = 5,
//                .CoefB             = 0.1,
//                .Improve           = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter | PID_ChangingIntegrationRate,
//                .IntegralLimit     = 1000,
//                .MaxOut            = 16000,
//                .Derivative_LPF_RC = 0.001,
//                .DeadBand          = 0.5,
//            },
//            .speed_PID = {
//                .Kp            = 30,
//                .Ki            = 0.5,
//                .Kd            = 0.001,
//                .Improve       = PID_Integral_Limit | PID_Derivative_On_Measurement | PID_ChangingIntegrationRate | PID_OutputFilter,
//                .IntegralLimit = 4000,
//                .MaxOut        = 16000,
//                .Output_LPF_RC = 0.03,
//            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type       = ANGLE_LOOP,
            .close_loop_type       = SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag    = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = GM6020,
    };
    chassis_motor_steering_config.can_init_config.tx_id = 4;
    motor_steering_lf                                   = DJIMotorInit(&chassis_motor_steering_config);
    chassis_motor_steering_config.can_init_config.tx_id = 1;
    motor_steering_rf                                   = DJIMotorInit(&chassis_motor_steering_config);
    chassis_motor_steering_config.can_init_config.tx_id = 3;
    motor_steering_lb                                   = DJIMotorInit(&chassis_motor_steering_config);
    chassis_motor_steering_config.can_init_config.tx_id = 2;
    motor_steering_rb                                   = DJIMotorInit(&chassis_motor_steering_config);

		PID_Init_Config_s chassis_follow_pid_conf = {
        .Kp                = 100, // 6
        .Ki                = 0.1f,
        .Kd                = 17, // 0.5
        .DeadBand          = 0.5,
        .CoefA             = 0.2,
        .CoefB             = 0.3,
        .Improve           = PID_Trapezoid_Intergral | PID_DerivativeFilter | PID_DerivativeFilter | PID_Derivative_On_Measurement | PID_Integral_Limit | PID_Derivative_On_Measurement,
        .IntegralLimit     = 500, // 200
        .MaxOut            = 20000,
        .Derivative_LPF_RC = 0.01, // 0.01
    };
    PIDInit(&chassis_follow_pid, &chassis_follow_pid_conf);
		nac_ctrl = NacInit(&huart1);
       chassis_ctrl_cmd.Chassis_IMU_data = INS_Init();
        chassis_ctrl_cmd.correct_mode =  IMU_CORRECT_HYBRID;
        chassis_ctrl_cmd.imu_enable = 1;                       // 使能IMU校准
        chassis_ctrl_cmd.target_yaw = 0;
        chassis_ctrl_cmd.offset_w = 0;
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
static void MinmizeRotation(float *angle, const float *last_angle, float *speed)
{
    float rotation = *angle - *last_angle;

    if (rotation > 90) {
        *angle -= 180;
        *speed = -(*speed);
    } else if (rotation < -90) {
        *angle += 180;
        *speed = -(*speed);
    }
}
/**
 * @brief 统一的IMU角度校准函数，支持多种模式
 * @param target_vw 目标角速度（来自指令）
 * @return 校准后的offset_w
 */
static float UpdateIMUCorrection(float target_vw)
{
    if(!chassis_ctrl_cmd.imu_enable) {
        return 0;  // IMU未使能，不校准
    }
    
    float current_yaw = chassis_ctrl_cmd.Chassis_IMU_data->Yaw;
    float offset = 0;
    
    switch(chassis_ctrl_cmd.correct_mode)
    {
        case IMU_CORRECT_STRAIGHT:
            // 直线模式：只在无转速指令时校准
            if(fabsf(target_vw) < 100.0f) {  // 死区判断
                float yaw_error = current_yaw - chassis_ctrl_cmd.last_yaw;
                // 处理角度跳变
                if(yaw_error > 180.0f) yaw_error -= 360.0f;
                else if(yaw_error < -180.0f) yaw_error += 360.0f;
                offset = PIDCalculate(&chassis_follow_pid, yaw_error, 0);
            } else {
                // 有转速指令时，更新参考角度，不校准
                chassis_ctrl_cmd.last_yaw = current_yaw;
                offset = 0;
            }
            break;
            
        case IMU_CORRECT_ROTATION:
            // 转弯模式：跟踪目标角度
            if(fabsf(target_vw) > 100.0f) {
                // 有转速指令时，更新目标角度
                chassis_ctrl_cmd.target_yaw += target_vw * 0.001f; // 积分计算期望角度
                // 归一化到-180~180
                while(chassis_ctrl_cmd.target_yaw > 180.0f) chassis_ctrl_cmd.target_yaw -= 360.0f;
                while(chassis_ctrl_cmd.target_yaw < -180.0f) chassis_ctrl_cmd.target_yaw += 360.0f;
            }
            // 计算与目标角度的误差
            float target_error = current_yaw - chassis_ctrl_cmd.target_yaw;
            if(target_error > 180.0f) target_error -= 360.0f;
            else if(target_error < -180.0f) target_error += 360.0f;
            offset = PIDCalculate(&chassis_follow_pid, target_error, 0);
            break;
            
        case IMU_CORRECT_HYBRID:
        {
            // 混合模式：根据转速大小动态调整
            float yaw_error = current_yaw - chassis_ctrl_cmd.last_yaw;
            // 处理角度跳变
            if(yaw_error > 180.0f) yaw_error -= 360.0f;
            else if(yaw_error < -180.0f) yaw_error += 360.0f;
            
            if(fabsf(target_vw) < 100.0f) {
                // 小转速或直线：保持角度
                offset = PIDCalculate(&chassis_follow_pid, yaw_error, 0);
            } else {
                // 大转速：辅助控制，减小PID增益
                offset = PIDCalculate(&chassis_follow_pid, yaw_error, 0) * 0.5f;
                // 更新参考角度，避免误差累积
                chassis_ctrl_cmd.last_yaw = current_yaw;
            }
            break;
        }
            
        default:
            offset = 0;
            break;
    }
    
    return offset;
}

void GetCmd(){
	
	//解算似乎有点问题，左走变直线，直线变左走，因此这直接改两个轴
	chassis_ctrl_cmd.vy = ((float)rc_cmd->rc.rocker_l_/660)*40000;
	chassis_ctrl_cmd.vx= ((float)rc_cmd->rc.rocker_l1/660)*40000;
	
	chassis_ctrl_cmd.vw = ((float)rc_cmd->rc.dial/660)*10000;

}


/**
 * @brief 舵轮电机角度解算
 *
 */
static void SteeringWheelCalculate()
{
    float offset_lf, offset_rf, offset_lb, offset_rb;     // 用于计算舵轮的角度
    float at_lf_last, at_rf_last, at_lb_last, at_rb_last; // 上次的角度
		float chassis_vx = 0;
		float chassis_vy = 0;
		float chassis_vw = 0;
    at_lb_last = motor_steering_lb->measure.total_angle;
    at_lf_last = motor_steering_lf->measure.total_angle;
    at_rf_last = motor_steering_rf->measure.total_angle;
    at_rb_last = motor_steering_rb->measure.total_angle;
	
	// 判断是否有速度指令（先判断,后赋值）
	if(chassis_ctrl_cmd.vx != 0 || chassis_ctrl_cmd.vy != 0 || chassis_ctrl_cmd.vw != 0) {
		// 有速度指令时,正常运动学解算并启用IMU校准
		chassis_vx = chassis_ctrl_cmd.vx;
		chassis_vy = chassis_ctrl_cmd.vy;
		// 首次运行时初始化last_yaw
		static uint8_t first_run = 1;
		if(first_run) {
			chassis_ctrl_cmd.last_yaw = chassis_ctrl_cmd.Chassis_IMU_data->Yaw;
			first_run = 0;
		}
		chassis_ctrl_cmd.offset_w = UpdateIMUCorrection(chassis_ctrl_cmd.vw);
		chassis_vw = chassis_ctrl_cmd.vw + chassis_ctrl_cmd.offset_w;
	} else {
		// 完全静止时,不调用IMU校准
		chassis_vx = 0;
		chassis_vy = 0;
		chassis_vw = 0;  
	}
    
        // 生成预计算变量，减少计算量，空间换时间
        // chassis_vx = chassis_vx * 1.5;chassis_vy = chassis_vy * 1.5;
//        float w      = chassis_cmd_recv.wz * CHASSIS_WHEEL_OFFSET * SQRT2;
		float w = chassis_vw;
        float temp_x = chassis_vx - w, temp_y = chassis_vy - w;
        arm_sqrt_f32(temp_x * temp_x + temp_y * temp_y, &vt_lf); // lf：y- , x-
        temp_y = chassis_vy + w;                                 // 重复利用变量,temp_x = chassis_vy - w;与上次相同因此注释
        arm_sqrt_f32(temp_x * temp_x + temp_y * temp_y, &vt_lb); // lb: y+ , x-
        temp_x = chassis_vx + w;                                 // temp_y = chassis_vx + w;与上次相同因此注释
        arm_sqrt_f32(temp_x * temp_x + temp_y * temp_y, &vt_rb); // rb: y+ , x+
        temp_y = chassis_vy - w;                                 // temp_x = chassis_vy + w;与上次相同因此注释
        arm_sqrt_f32(temp_x * temp_x + temp_y * temp_y, &vt_rf); // rf: y- , x+
    
        // 计算角度偏移
        offset_lf = atan2f(chassis_vy - w, chassis_vx - w) * RAD_2_DEGREE; // lf:  y- , x-
        offset_rf = atan2f(chassis_vy - w, chassis_vx + w) * RAD_2_DEGREE; // rf:  y- , x+
        offset_lb = atan2f(chassis_vy + w, chassis_vx - w) * RAD_2_DEGREE; // lb:  y+ , x-
        offset_rb = atan2f(chassis_vy + w, chassis_vx + w) * RAD_2_DEGREE; // rb:  y+ , x+
  
        at_lf = STEERING_CHASSIS_ALIGN_ANGLE_LF + offset_lf; 
        at_rf = STEERING_CHASSIS_ALIGN_ANGLE_RF + offset_rf;
        at_lb = STEERING_CHASSIS_ALIGN_ANGLE_LB + offset_lb;
        at_rb = STEERING_CHASSIS_ALIGN_ANGLE_RB + offset_rb;


				
				
		ANGLE_LIMIT_360_TO_180_ABS(at_lf);
        ANGLE_LIMIT_360_TO_180_ABS(at_rf);
        ANGLE_LIMIT_360_TO_180_ABS(at_lb);
        ANGLE_LIMIT_360_TO_180_ABS(at_rb);

        MinmizeRotation(&at_lf, &at_lf_last, &vt_lf);
        MinmizeRotation(&at_rf, &at_rf_last, &vt_rf);
        MinmizeRotation(&at_lb, &at_lb_last, &vt_lb);
        MinmizeRotation(&at_rb, &at_rb_last, &vt_rb);
    
//if(w==0){

//    DJIMotorSetRef(motor_steering_lf, at_lf);
//    DJIMotorSetRef(motor_steering_rf, at_rf);
//    DJIMotorSetRef(motor_steering_lb, at_lb);
//    DJIMotorSetRef(motor_steering_rb, at_rb);
//		
//		DJIMotorSetRef(motor_lf, vt_lf*2.5 );
//    DJIMotorSetRef(motor_rf, vt_rf*2.5 );
//    DJIMotorSetRef(motor_lb, vt_lb*2.5 );
//    DJIMotorSetRef(motor_rb, vt_rb*2.5 );
//}

//else{
    

    DJIMotorSetRef(motor_steering_lf, at_lf);
    DJIMotorSetRef(motor_steering_rf, at_rf);
    DJIMotorSetRef(motor_steering_lb, at_lb);//+90
    DJIMotorSetRef(motor_steering_rb, at_rb);
		if (w==6000){
			DJIMotorSetRef(motor_lf, 0 );
			DJIMotorSetRef(motor_rf, 0 );
			DJIMotorSetRef(motor_lb, 0 );
			DJIMotorSetRef(motor_rb, 0 );
		}
		else{
		DJIMotorSetRef(motor_lf, vt_lf );
    DJIMotorSetRef(motor_rf, vt_rf );
    DJIMotorSetRef(motor_lb, vt_lb );
    DJIMotorSetRef(motor_rb, vt_rb );
		}
//}
	}
/**
 * @brief 舵轮运动学解算(旧版本修改-速度控制+航向锁定)
 * @param vx 前进速度   
 * @param vw 角速度(旋转速度)
 * @note  内部强制 vy=0，且在 vw=0 时自动锁定航向
 */
void SteeringWheelKinematics_old(float vx, float vw)
{
    float vy = 0; // 强制横移为0
    float offset_lf, offset_rf, offset_lb, offset_rb;     // 用于计算舵轮的角度
    float at_lf_last, at_rf_last, at_lb_last, at_rb_last; // 上次的角度
    float chassis_vx = 0;
    float chassis_vy = 0;
    float chassis_vw = 0;
    
    // 获取上次角度
    at_lb_last = motor_steering_lb->measure.total_angle;
    at_lf_last = motor_steering_lf->measure.total_angle;
    at_rf_last = motor_steering_rf->measure.total_angle;
    at_rb_last = motor_steering_rb->measure.total_angle;
   
    // 赋值速度
    chassis_vx = vx;
    chassis_vy = vy;

    // --- IMU 航向锁定逻辑 ---
    static float target_lock_yaw = 0;
    static uint8_t first_run = 1;
    float current_yaw = chassis_ctrl_cmd.Chassis_IMU_data->Yaw;

    // 初始化
    if(first_run) {
        target_lock_yaw = current_yaw;
        first_run = 0;
    }

    if (fabsf(vw) > 10.0f) { // 设定一个死区，避免摇杆漂移导致无法锁定
        // 1. 有旋转指令：执行指令，并跟随当前角度
        chassis_vw = vw;
        target_lock_yaw = current_yaw; 
    } else {
        // 2. 无旋转指令：执行锁头 (PID修正)
        float yaw_error = target_lock_yaw - current_yaw;
        
        // 角度过零处理 (-180 ~ 180)
        if(yaw_error > 180.0f) yaw_error -= 360.0f;
        else if(yaw_error < -180.0f) yaw_error += 360.0f;

        // 使用与新版本相同的 PID 对象进行修正
        // 注意：这里假设 PID 输出方向与 vw 定义一致，参考新版本取负号
        chassis_vw = -PIDCalculate(&chassis_follow_pid, yaw_error, 0);
    }

    // 核心解算
    // 使用真实的物理尺寸进行解算
    // LF: x+, y+; RF: x+, y-; LB: x-, y+; RB: x-, y-
    // v_wheel_x = vx - w * y
    // v_wheel_y = vy + w * x
    
    float half_base = CHASSIS_HALF_BASE;   // L/2
    float half_track = CHASSIS_HALF_TRACK; // W/2

    // LF (Left Front)
    float v_lf_x = chassis_vx - chassis_vw * half_track;
    float v_lf_y = chassis_vy + chassis_vw * half_base;
    arm_sqrt_f32(v_lf_x * v_lf_x + v_lf_y * v_lf_y, &vt_lf);
    offset_lf = atan2f(v_lf_y, v_lf_x) * RAD_2_DEGREE;

    // RF (Right Front)
    float v_rf_x = chassis_vx + chassis_vw * half_track;
    float v_rf_y = chassis_vy + chassis_vw * half_base;
    arm_sqrt_f32(v_rf_x * v_rf_x + v_rf_y * v_rf_y, &vt_rf);
    offset_rf = atan2f(v_rf_y, v_rf_x) * RAD_2_DEGREE;

    // LB (Left Back)
    float v_lb_x = chassis_vx - chassis_vw * half_track;
    float v_lb_y = chassis_vy - chassis_vw * half_base;
    arm_sqrt_f32(v_lb_x * v_lb_x + v_lb_y * v_lb_y, &vt_lb);
    offset_lb = atan2f(v_lb_y, v_lb_x) * RAD_2_DEGREE;

    // RB (Right Back)
    float v_rb_x = chassis_vx + chassis_vw * half_track;
    float v_rb_y = chassis_vy - chassis_vw * half_base;
    arm_sqrt_f32(v_rb_x * v_rb_x + v_rb_y * v_rb_y, &vt_rb);
    offset_rb = atan2f(v_rb_y, v_rb_x) * RAD_2_DEGREE;
    
    // 计算角度偏移
    // offset_lf = atan2f(chassis_vy - w, chassis_vx - w) * RAD_2_DEGREE;
    // offset_rf = atan2f(chassis_vy - w, chassis_vx + w) * RAD_2_DEGREE;
    // offset_lb = atan2f(chassis_vy + w, chassis_vx - w) * RAD_2_DEGREE;
    // offset_rb = atan2f(chassis_vy + w, chassis_vx + w) * RAD_2_DEGREE;
  
    at_lf = STEERING_CHASSIS_ALIGN_ANGLE_LF + offset_lf; 
    at_rf = STEERING_CHASSIS_ALIGN_ANGLE_RF + offset_rf;
    at_lb = STEERING_CHASSIS_ALIGN_ANGLE_LB + offset_lb;
    at_rb = STEERING_CHASSIS_ALIGN_ANGLE_RB + offset_rb;

    ANGLE_LIMIT_360_TO_180_ABS(at_lf);
    ANGLE_LIMIT_360_TO_180_ABS(at_rf);
    ANGLE_LIMIT_360_TO_180_ABS(at_lb);
    ANGLE_LIMIT_360_TO_180_ABS(at_rb);

    MinmizeRotation(&at_lf, &at_lf_last, &vt_lf);
    MinmizeRotation(&at_rf, &at_rf_last, &vt_rf);
    MinmizeRotation(&at_lb, &at_lb_last, &vt_lb);
    MinmizeRotation(&at_rb, &at_rb_last, &vt_rb);

    DJIMotorSetRef(motor_steering_lf, at_lf);
    DJIMotorSetRef(motor_steering_rf, at_rf);
    DJIMotorSetRef(motor_steering_lb, at_lb);
    DJIMotorSetRef(motor_steering_rb, at_rb);

    // 停车逻辑：指令全0 且 PID修正量也很小(说明已对准)
    if (vx == 0 && vw == 0 && fabsf(chassis_vw) < 100.0f){
        DJIMotorSetRef(motor_lf, 0 );
        DJIMotorSetRef(motor_rf, 0 );
        DJIMotorSetRef(motor_lb, 0 );
        DJIMotorSetRef(motor_rb, 0 );
    }
    else{
        DJIMotorSetRef(motor_lf, vt_lf );
        DJIMotorSetRef(motor_rf, vt_rf );
        DJIMotorSetRef(motor_lb, vt_lb );
        DJIMotorSetRef(motor_rb, vt_rb );
    }
}

/**
 * @brief 舵轮运动学解算(角度控制版本-IMU闭环)
 * @param vx 前进速度   
 * @param vy 横移速度    
 * @param target_angle 目标绝对角度(度),以IMU初始yaw=0为基准,正值逆时针,负值顺时针
 *                     传入0时保持当前角度不变
 */
void SteeringWheelKinematics(float vx, float vy, float target_angle)
{
    float offset_lf, offset_rf, offset_lb, offset_rb;
    float at_lf_last, at_rf_last, at_lb_last, at_rb_last;
    float chassis_vx = 0;
    float chassis_vy = 0;
    float chassis_vw = 0;
    
    static float last_target_angle = 0;  // 记录上次的目标角度
    static uint8_t first_run_kinematics = 1;
    
    // 获取上次角度
    at_lb_last = motor_steering_lb->measure.total_angle;
    at_lf_last = motor_steering_lf->measure.total_angle;
    at_rf_last = motor_steering_rf->measure.total_angle;
    at_rb_last = motor_steering_rb->measure.total_angle;
    
    // 首次运行时初始化last_yaw
    if(first_run_kinematics) {
        chassis_ctrl_cmd.last_yaw = chassis_ctrl_cmd.Chassis_IMU_data->Yaw;
        first_run_kinematics = 0;
    }
    
    // 处理角度控制逻辑
    float current_yaw = chassis_ctrl_cmd.Chassis_IMU_data->Yaw;
    
    // 检测目标角度是否改变
    if(fabsf(target_angle - last_target_angle) > 0.1f) {
        // 目标角度改变,更新目标
        chassis_ctrl_cmd.target_yaw = target_angle;
        // 归一化到-180~180
        while(chassis_ctrl_cmd.target_yaw > 180.0f) chassis_ctrl_cmd.target_yaw -= 360.0f;
        while(chassis_ctrl_cmd.target_yaw < -180.0f) chassis_ctrl_cmd.target_yaw += 360.0f;
        last_target_angle = target_angle;
    }
    
    // 计算角度误差
    float yaw_error = chassis_ctrl_cmd.target_yaw - current_yaw;
    if(yaw_error > 180.0f) yaw_error -= 360.0f;
    else if(yaw_error < -180.0f) yaw_error += 360.0f;
    
    // 判断是否到达目标
    if(fabsf(yaw_error) < 2.0f) {  // 到达目标,精度2度
        chassis_vw = 0;
        chassis_ctrl_cmd.last_yaw = current_yaw;  // 更新参考角度
    } else {
        // 使用PID计算角速度,实现闭环控制
        chassis_vw = -PIDCalculate(&chassis_follow_pid, yaw_error, 0);
    }
    
    // 设置线速度
    chassis_vx = vx;
    chassis_vy = vy;
    
    // 生成预计算变量
    float w = chassis_vw;
    float temp_x = chassis_vx - w, temp_y = chassis_vy - w;
    arm_sqrt_f32(temp_x * temp_x + temp_y * temp_y, &vt_lf);
    temp_y = chassis_vy + w;
    arm_sqrt_f32(temp_x * temp_x + temp_y * temp_y, &vt_lb);
    temp_x = chassis_vx + w;
    arm_sqrt_f32(temp_x * temp_x + temp_y * temp_y, &vt_rb);
    temp_y = chassis_vy - w;
    arm_sqrt_f32(temp_x * temp_x + temp_y * temp_y, &vt_rf);
    
    // 计算角度偏移
    offset_lf = atan2f(chassis_vy - w, chassis_vx - w) * RAD_2_DEGREE;
    offset_rf = atan2f(chassis_vy - w, chassis_vx + w) * RAD_2_DEGREE;
    offset_lb = atan2f(chassis_vy + w, chassis_vx - w) * RAD_2_DEGREE;
    offset_rb = atan2f(chassis_vy + w, chassis_vx + w) * RAD_2_DEGREE;
    
    at_lf = STEERING_CHASSIS_ALIGN_ANGLE_LF + offset_lf;
    at_rf = STEERING_CHASSIS_ALIGN_ANGLE_RF + offset_rf;
    at_lb = STEERING_CHASSIS_ALIGN_ANGLE_LB + offset_lb;
    at_rb = STEERING_CHASSIS_ALIGN_ANGLE_RB + offset_rb;
    
    ANGLE_LIMIT_360_TO_180_ABS(at_lf);
    ANGLE_LIMIT_360_TO_180_ABS(at_rf);
    ANGLE_LIMIT_360_TO_180_ABS(at_lb);
    ANGLE_LIMIT_360_TO_180_ABS(at_rb);
    
    MinmizeRotation(&at_lf, &at_lf_last, &vt_lf);
    MinmizeRotation(&at_rf, &at_rf_last, &vt_rf);
    MinmizeRotation(&at_lb, &at_lb_last, &vt_lb);
    MinmizeRotation(&at_rb, &at_rb_last, &vt_rb);
    
    DJIMotorSetRef(motor_steering_lf, at_lf);
    DJIMotorSetRef(motor_steering_rf, at_rf);
    DJIMotorSetRef(motor_steering_lb, at_lb);
    DJIMotorSetRef(motor_steering_rb, at_rb);
    
    if(w == 0 && vx == 0 && vy == 0) {
        DJIMotorSetRef(motor_lf, 0);
        DJIMotorSetRef(motor_rf, 0);
        DJIMotorSetRef(motor_lb, 0);
        DJIMotorSetRef(motor_rb, 0);
    } else {
        DJIMotorSetRef(motor_lf, vt_lf);
        DJIMotorSetRef(motor_rf, vt_rf);
        DJIMotorSetRef(motor_lb, vt_lb);
        DJIMotorSetRef(motor_rb, vt_rb);
    }
}

void ChassisTest_OldVersion(){
    // 使用遥控器模拟上位机指令进行测试
    // 左摇杆上下(rocker_l1) -> 控制前进速度 vx
    // 拨轮(dial) -> 控制旋转速度 vw
    
    float test_vx = ((float)rc_cmd->rc.rocker_l1 / 660.0f) * 8000.0f; // 速度系数，根据实际情况调整
    float test_vw = ((float)rc_cmd->rc.dial / 660.0f) * 5000.0f;      // 角速度系数
    
    // 简单的死区处理，防止误触
    if(fabsf(test_vx) < 200.0f) test_vx = 0;
    if(fabsf(test_vw) < 100.0f) test_vw = 0;
    
    // 调用修改后的旧版本解算函数
    SteeringWheelKinematics_old(test_vx, test_vw);
}

void ChassisTask()
{
    // 检查USB数据超时 (500ms)
    // 如果上位机停止发送，底盘应该停止，防止失控
    if (HAL_GetTick() - usb_last_recv_time < 500) {
        // 未超时，使用USB指令
        
        // 单位转换: 
        // 上位机单位: 线速度 m/s, 角速度 rad/s
        // 转换系数: LINEAR_VELOCITY_TO_MOTOR_RPM (m/s -> RPM)
        
        // 线速度转换
        float cmd_vx = usb_chassis_cmd.linear_x * LINEAR_VELOCITY_TO_MOTOR_RPM; 
        
        // 角速度转换: 
        //传入 (rad/s * 转换系数), 在解算函数内部再乘以具体的半径(half_track/half_base)
        float cmd_vw = usb_chassis_cmd.angular_z * LINEAR_VELOCITY_TO_MOTOR_RPM; 
        
        SteeringWheelKinematics_old(cmd_vx, cmd_vw);
    } else {
        // 超时，停车
        SteeringWheelKinematics_old(0, 0);
    }
}
