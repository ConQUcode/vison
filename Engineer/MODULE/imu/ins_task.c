/**
 ******************************************************************************
 * @file    ins_task.c
 * @author  Wang Hongxi
 * @author  annotation and modificaiton by neozng
 * @version V2.0.0
 * @date    2022/2/23
 * @brief   惯性导航系统(INS)任务实现文件
 * @details 该模块实现完整的惯性导航解算系统，主要功能：
 *          1. BMI088 IMU传感器数据读取和处理
 *          2. 基于扩展卡尔曼滤波(EKF)的姿态解算
 *          3. 四元数和欧拉角转换
 *          4. 坐标系变换（机体系↔惯性系）
 *          5. 运动加速度解算和滤波
 *          6. IMU温度控制和校准
 ******************************************************************************
 * @attention
 *          - INS任务应以1kHz频率运行以保证解算精度
 *          - 初始化不要放入实时系统，应在应用层调用
 *          - 温度控制以500Hz频率运行，确保IMU恒温工作
 ******************************************************************************
 */
#include "ins_task.h"
#include "controller.h"
#include "QuaternionEKF.h"
#include "spi.h"
#include "tim.h"
#include "user_lib.h"
#include "general_def.h"
// #include "robot_def.h"

// ======================== 全局变量定义 ========================
static INS_t INS;                    // INS主结构体，存储所有惯导相关数据
static IMU_Param_t IMU_Param;        // IMU参数结构体，用于安装误差校正
static PID_Instance TempCtrl = {0};  // IMU温度控制PID实例

// 机体坐标系标准基向量定义
const float xb[3] = {1, 0, 0};  // 机体系X轴单位向量（前向）
const float yb[3] = {0, 1, 0};  // 机体系Y轴单位向量（右向）
const float zb[3] = {0, 0, 1};  // 机体系Z轴单位向量（下向）

// ======================== 时间和温控相关变量 ========================
static uint32_t INS_DWT_Count = 0;  // DWT计数器，用于精确时间测量
static float dt = 0, t = 0;         // 时间间隔和累计时间
static float RefTemp = 40;          // IMU恒温控制目标温度（℃）

// ======================== 私有函数声明 ========================
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);

/**
 * @brief 设置IMU加热器PWM占空比
 * @param pwm PWM占空比值 (0-65535)
 * @details 通过TIM10的通道1输出PWM信号控制IMU加热器
 *          用于维持IMU在恒定温度下工作，提高测量精度
 */
static void IMUPWMSet(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}

/**
 * @brief IMU温度闭环控制函数
 * @details 通过PID控制器调节IMU加热器功率，维持IMU在设定温度
 *          恒温工作可以显著提高陀螺仪和加速度计的测量精度和稳定性
 * @note 该函数应以500Hz频率调用
 */
static void IMU_Temperature_Ctrl(void)
{
    // 使用PID控制器计算温度控制输出
    PIDCalculate(&TempCtrl, BMI088.Temperature, RefTemp);
    
    // 限幅并设置PWM输出，控制加热器功率
    IMUPWMSet(float_constrain(float_rounding(TempCtrl.Output), 0, UINT32_MAX));
}

/**
 * @brief 使用加速度计数据初始化四元数，设定初始姿态
 * @param init_q4 输出的初始四元数[q0, q1, q2, q3]
 * @details 该函数通过加速度计测量的重力方向来估算初始的Roll和Pitch角
 *          Yaw角设为0，避免初始姿态估计误差影响后续解算
 *          初始化流程：
 *          1. 多次采样加速度计数据并取平均值
 *          2. 计算重力矢量与标准重力的夹角和旋转轴
 *          3. 使用轴角公式计算初始四元数
 * @note 初始化时机器人应保持静止状态
 */
static void InitQuaternion(float *init_q4)
{
    float acc_init[3] = {0};
    float gravity_norm[3] = {0, 0, 1}; // 导航系标准重力矢量(归一化)
    float axis_rot[3] = {0};           // 旋转轴矢量
    // 步骤1: 采集100次加速度计数据取平均值，消除随机噪声
    for (uint8_t i = 0; i < 100; ++i)
    {
        BMI088_Read(&BMI088);          // 读取BMI088传感器数据
        acc_init[X] += BMI088.Accel[X]; // 累加X轴加速度
        acc_init[Y] += BMI088.Accel[Y]; // 累加Y轴加速度
        acc_init[Z] += BMI088.Accel[Z]; // 累加Z轴加速度
        DWT_Delay(0.001);              // 延时1ms，避免采样过快
    }
    
    // 步骤2: 计算平均值并归一化
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= 100;            // 取平均值
    Norm3d(acc_init);                  // 归一化为单位向量
    
    // 步骤3: 计算从当前重力向量到标准重力向量的旋转
    float angle = acosf(Dot3d(acc_init, gravity_norm));  // 计算两向量夹角
    Cross3d(acc_init, gravity_norm, axis_rot);           // 计算旋转轴（叉积）
    Norm3d(axis_rot);                                    // 旋转轴归一化
    
    // 步骤4: 使用轴角公式计算四元数 q = [cos(θ/2), sin(θ/2)*axis]
    init_q4[0] = cosf(angle / 2.0f);                     // 四元数标量部分
    for (uint8_t i = 0; i < 2; ++i)
        init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // 四元数向量部分（只有xy分量）
    // 注意：init_q4[3] = 0，因为初始化时Yaw=0，没有z轴旋转分量
}

/**
 * @brief 惯性导航系统初始化函数
 * @return attitude_t* 返回姿态数据结构指针，供外部访问姿态信息
 * @details 完整的INS初始化流程：
 *          1. 检查是否已初始化，避免重复初始化
 *          2. 启动IMU温度控制PWM
 *          3. 初始化BMI088传感器
 *          4. 配置IMU校准参数
 *          5. 初始化四元数EKF滤波器
 *          6. 配置温度PID控制器
 *          7. 设置加速度低通滤波参数
 * @note 该函数执行时间较长，不应在实时任务中调用
 */
attitude_t *INS_Init(void)
{
    // 检查初始化标志，防止重复初始化
    if (!INS.init)
        INS.init = 1;
    else
        return (attitude_t *)&INS.Gyro;

    // 启动TIM10通道1的PWM输出，用于IMU温度控制
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

    // 初始化BMI088传感器，循环等待直到初始化成功
    while (BMI088Init(&hspi1, 1) != BMI088_NO_ERROR)
        ;
        
    // 初始化IMU校准参数（安装误差校正）
    IMU_Param.scale[X] = 1;  // X轴标度因子
    IMU_Param.scale[Y] = 1;  // Y轴标度因子
    IMU_Param.scale[Z] = 1;  // Z轴标度因子
    IMU_Param.Yaw = 0;       // Yaw轴安装偏移角
    IMU_Param.Pitch = 0;     // Pitch轴安装偏移角
    IMU_Param.Roll = 0;      // Roll轴安装偏移角
    IMU_Param.flag = 1;      // 参数更新标志

    // 初始化四元数EKF滤波器
    float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);  // 基于重力方向计算初始四元数
    // EKF参数: 初始四元数，过程噪声方差，观测噪声方差，初始协方差，陀螺仪噪声，加速度计噪声
    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);
    
    // 配置IMU温度控制PID参数
    PID_Init_Config_s config = {
        .MaxOut = 2000,          // 最大输出限幅（PWM占空比）
        .IntegralLimit = 300,    // 积分限幅，防止积分饱和
        .DeadBand = 0,           // 死区设为0
        .Kp = 1000,              // 比例增益，快速响应温度偏差
        .Ki = 20,                // 积分增益，消除稳态误差
        .Kd = 0,                 // 微分增益，温控系统一般不需要
        .Improve = PID_Integral_Limit  // 启用积分限幅功能
    };
    PIDInit(&TempCtrl, &config);

    // 设置加速度低通滤波系数，滤除高频噪声
    // 加速度计噪声相对较大且频率较高，使用LPF平滑数据
    INS.AccelLPF = 0.0085;
    
    // 初始化时间测量
    DWT_GetDeltaT(&INS_DWT_Count);
    
    return (attitude_t *)&INS.Gyro; // @todo: 这里偷懒了,不要这样做! 修改INS_t结构体可能会导致异常,待修复.
}

/**
 * @brief 惯性导航系统主任务函数
 * @details 该函数实现完整的INS解算流程，包括：
 *          - 1000Hz: IMU数据读取和姿态解算
 *          - 500Hz:  温度控制
 *          - 1Hz:    系统监控（可扩展）
 * @note 必须以1kHz频率运行此任务，确保姿态解算精度
 *       建议在FreeRTOS任务中调用：osDelay(1)
 * @warning 任务频率不稳定会影响EKF滤波器性能和姿态解算精度
 */
void INS_Task(void)
{
    static uint32_t count = 0;           // 任务计数器，用于频率分频
    const float gravity[3] = {0, 0, 9.81f}; // 标准重力加速度矢量(m/s²)

    // 获取精确的时间间隔，用于积分运算
    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;  // 累计运行时间

    // ==================== 1000Hz: IMU数据处理和姿态解算 ====================
    if ((count % 1) == 0)
    {
        // 读取BMI088 IMU传感器数据
        BMI088_Read(&BMI088);

        // 将传感器数据拷贝到INS结构体中
        INS.Accel[X] = BMI088.Accel[X];  // X轴加速度 (m/s²)
        INS.Accel[Y] = BMI088.Accel[Y];  // Y轴加速度 (m/s²)
        INS.Accel[Z] = BMI088.Accel[Z];  // Z轴加速度 (m/s²)
        INS.Gyro[X] = BMI088.Gyro[X];    // X轴角速度 (rad/s)
        INS.Gyro[Y] = BMI088.Gyro[Y];    // Y轴角速度 (rad/s)
        INS.Gyro[Z] = BMI088.Gyro[Z];    // Z轴角速度 (rad/s)

        // IMU安装误差校正（可选功能，当前demo未启用）
        IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

        // 预留功能：计算倾斜角度，可用于姿态粗估计或异常检测
        // INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI; // X-Z平面倾斜角
        // INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;   // Y-Z平面倾斜角

        // ★ 核心算法：扩展卡尔曼滤波器更新四元数
        // 融合陀螺仪和加速度计数据，估计最优姿态
        IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], 
                                 INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

        // 获取EKF滤波后的四元数估计值
        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // 计算机体坐标系基向量在导航坐标系(惯性系)中的表示
        // 这些向量可用于后续的坐标变换和姿态可视化
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);  // 机体X轴在导航系中的方向
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);  // 机体Y轴在导航系中的方向
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);  // 机体Z轴在导航系中的方向

        // ==================== 运动加速度解算 ====================
        // 物理原理：加速度计测量值 = 运动加速度 + 重力加速度
        // 因此：运动加速度 = 加速度计测量值 - 重力分量
        
        // 步骤1：将标准重力向量从导航系转换到当前机体系
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        
        // 步骤2：计算机体系下的运动加速度并进行低通滤波
        // 滤波公式：y(k) = (1-α)×x(k) + α×y(k-1)，其中α = τ/(τ+dt)
        for (uint8_t i = 0; i < 3; ++i)
        {
            // 去除重力分量，得到纯运动加速度，并通过一阶低通滤波器平滑
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + 
                                   INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        
        // 步骤3：将机体系运动加速度转换到导航系，便于导航解算
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q);

        // 从EKF滤波器中提取欧拉角结果
        INS.Yaw = QEKF_INS.Yaw;                    // 偏航角 (°)
        INS.Pitch = QEKF_INS.Pitch;                // 俯仰角 (°) 
        INS.Roll = QEKF_INS.Roll;                  // 横滚角 (°)
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle; // 偏航角累计值，支持多圈计数
        
        // 向视觉系统发送姿态信息（条件编译）
#if (defined(ONE_BOARD) || defined(GIMBAL_BOARD))
        VisionSetAltitude(INS.Yaw, INS.Pitch, INS.Roll);
#endif
    }

    // ==================== 500Hz: IMU温度控制 ====================
    if ((count % 2) == 0)
    {
        // 以500Hz频率进行温度控制，确保IMU恒温工作
        // 频率选择原因：温度变化相对缓慢，500Hz足够且不会过度消耗CPU
        IMU_Temperature_Ctrl();
    }

    // ==================== 1Hz: 系统监控和诊断 ====================
    if ((count++ % 1000) == 0)
    {
        // 1Hz频率执行系统级监控任务
        // 可扩展功能：IMU健康检查、数据有效性验证、离线检测等
        // 例如：检查IMU数据更新率、温度是否异常、传感器故障诊断
    }
}

/**
 * @brief 将三维向量从机体坐标系转换到地球坐标系(导航系)
 * @param vecBF 输入：机体坐标系下的三维向量 [x_b, y_b, z_b]
 * @param vecEF 输出：地球坐标系下的三维向量 [x_e, y_e, z_e]  
 * @param q     四元数 [q0, q1, q2, q3]，表示从机体系到地球系的旋转
 * @details 数学原理：使用四元数旋转公式 v_e = q ⊗ v_b ⊗ q*
 *          其中⊗表示四元数乘法，q*表示四元数共轭
 *          展开后的矩阵形式避免了四元数乘法的复杂计算
 * @note 常用场景：
 *       - 将机体系下的加速度转换为导航系加速度
 *       - 将机体系下的速度转换为地理坐标系速度
 *       - 姿态可视化和控制算法中的坐标统一
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief 将三维向量从地球坐标系(导航系)转换到机体坐标系
 * @param vecEF 输入：地球坐标系下的三维向量 [x_e, y_e, z_e]
 * @param vecBF 输出：机体坐标系下的三维向量 [x_b, y_b, z_b]
 * @param q     四元数 [q0, q1, q2, q3]，表示从机体系到地球系的旋转
 * @details 数学原理：使用四元数共轭旋转 v_b = q* ⊗ v_e ⊗ q
 *          这是BodyFrameToEarthFrame的逆变换
 * @note 常用场景：
 *       - 将重力向量从导航系转换到机体系（用于运动加速度解算）
 *       - 将导航指令从地理系转换到机体系（用于控制算法）
 *       - 传感器数据的坐标系对齐
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief IMU安装误差和标度因数校正函数
 * @param param IMU校正参数结构体，包含安装角度和标度因子
 * @param gyro  输入/输出：三轴角速度数据 (rad/s)
 * @param accel 输入/输出：三轴加速度数据 (m/s²)
 * @details 校正内容：
 *          1. 标度因数校正：补偿传感器灵敏度误差
 *          2. 安装误差校正：补偿IMU与机体坐标系的安装偏差
 *          3. 使用ZYX欧拉角序列的旋转矩阵进行坐标变换
 * @note 校正原理：
 *       - 标度校正：data_corrected = scale × data_raw  
 *       - 安装校正：通过旋转矩阵R(yaw,pitch,roll)对齐坐标系
 *       - 旋转顺序：先绕Z轴(Yaw) → 再绕Y轴(Pitch) → 最后绕X轴(Roll)
 * @warning 该功能为高级校准功能，需要专业设备标定后才能使用
 *          当前demo中参数全部为默认值，实际未进行校正
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    // 检查校正参数是否发生变化，避免重复计算旋转矩阵
    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        // 角度转弧度 (度 × π/180 ≈ 度 / 57.295779513)
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 计算ZYX欧拉角旋转矩阵元素
        // R = R_z(α) × R_y(β) × R_x(γ)
        // 旋转顺序：1.Yaw(α) 2.Pitch(β) 3.Roll(γ)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;    // R[0][0]
        c_12 = cosPitch * sinYaw;                                 // R[0][1]
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;    // R[0][2]
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;    // R[1][0]
        c_22 = cosYaw * cosPitch;                                 // R[1][1]
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;   // R[1][2]
        c_31 = -cosPitch * sinRoll;                               // R[2][0]
        c_32 = sinPitch;                                          // R[2][1]
        c_33 = cosPitch * cosRoll;                                // R[2][2]
        param->flag = 0;  // 清除参数更新标志
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

// ========================================================================================
// ===================  以下函数为教学和扩展用途，当前demo中未使用  ===================
// ========================================================================================
// 这些函数提供了基础的四元数和欧拉角操作，可用于:
// 1. 学习四元数数学原理和实现方法
// 2. 开发自定义的姿态解算算法  
// 3. 替代或补充现有的EKF滤波器
// 4. 进行算法验证和对比分析
// ========================================================================================

/**
 * @brief 四元数更新函数，实现微分方程 dq/dt = 0.5Ωq
 * @param q  输入/输出：四元数 [q0, q1, q2, q3]
 * @param gx X轴角速度 (rad/s)
 * @param gy Y轴角速度 (rad/s) 
 * @param gz Z轴角速度 (rad/s)
 * @param dt 时间间隔 (s)
 * @details 数学原理：
 *          四元数微分方程：dq/dt = 0.5 * Ω(ω) * q
 *          其中Ω(ω)是角速度的反对称矩阵
 *          使用欧拉前差分方法进行数值积分
 * @note 使用场景：
 *       - 简单的姿态解算(不含加速度计修正)
 *       - 学习四元数积分原理
 *       - 作为EKF的预测步骤基础
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief 四元数转换为ZYX欧拉角（航空序列）
 * @param q    输入：四元数 [q0, q1, q2, q3]
 * @param Yaw  输出：偏航角(°) ∈ [-180, 180]
 * @param Pitch 输出：俯仰角(°) ∈ [-180, 180]
 * @param Roll  输出：横滚角(°) ∈ [-90, 90]
 * @details 数学公式：
 *          Yaw   = atan2(2(q0*q3 + q1*q2), 2(q0² + q1²) - 1)
 *          Pitch = atan2(2(q0*q1 + q2*q3), 2(q0² + q3²) - 1) 
 *          Roll  = asin(2(q0*q2 - q1*q3))
 * @warning 欧拉角存在万向节锁问题：
 *          当Pitch = ±90°时，Yaw和Roll不再独立
 *          建议直接使用四元数进行控制算法设计
 * @note 角度输出为度数，乘以57.295779513 = 180/π
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief ZYX欧拉角转换为四元数
 * @param Yaw   输入：偏航角(°)
 * @param Pitch 输入：俯仰角(°)
 * @param Roll  输入：横滚角(°)
 * @param q     输出：四元数 [q0, q1, q2, q3]
 * @details 数学公式（使用半角公式）：
 *          q0 = cos(φ/2)*cos(θ/2)*cos(ψ/2) + sin(φ/2)*sin(θ/2)*sin(ψ/2)
 *          q1 = sin(φ/2)*cos(θ/2)*cos(ψ/2) - cos(φ/2)*sin(θ/2)*sin(ψ/2)
 *          q2 = sin(φ/2)*cos(θ/2)*sin(ψ/2) + cos(φ/2)*sin(θ/2)*cos(ψ/2)
 *          q3 = cos(φ/2)*cos(θ/2)*sin(ψ/2) - sin(φ/2)*sin(θ/2)*cos(ψ/2)
 *          其中φ=Roll, θ=Pitch, ψ=Yaw
 * @note 应用场景：
 *       - 设定初始姿态或目标姿态
 *       - 从控制指令生成目标四元数
 *       - 姿态轨迹规划中的中间点生成
 * @warning 输入角度为度数，函数内部会转换为弧度
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}
