/**
 * @file nac.c
 * @brief NAC通信模块实现文件
 * @details 该模块负责与NAC设备（Navigation and Control）进行串口通信
 *          主要功能：
 *          1. 发送底盘电机转速和航向角信息给NAC设备
 *          2. 接收NAC设备的控制指令（x, y, z轴控制量）
 *          3. 通信协议解析和数据包封装
 *          4. 通信状态监控和离线保护
 * @author [Your Name]
 * @date 2024-11-28
 */

#include "nac.h"
#include "string.h"
#include "robot_def.h"
#include "daemon.h"

// ======================== 私有变量定义 ========================
static Nac_Instance *nac_instance;          // NAC通信实例指针
static uint8_t *vis_recv_buff __attribute__((unused)); // 接收缓冲区（暂未使用）
static Daemon_Instance *nac_daemon_instance; // 守护进程实例，用于检测通信状态





// ======================== 公共函数实现 ========================

/**
 * @brief 设置要发送给NAC设备的底盘状态信息
 * @param motor_lf_rpm 左前轮电机转速 (RPM)
 * @param motor_rf_rpm 右前轮电机转速 (RPM) 
 * @param motor_lb_rpm 左后轮电机转速 (RPM)
 * @param motor_rb_rpm 右后轮电机转速 (RPM)
 * @param chassis_yaw 底盘航向角 (度)
 * @note 该函数将底盘状态数据存储到发送缓冲区，等待发送
 */
void NacSetAltitude(int16_t motor_lf_rpm,int16_t motor_rf_rpm,int16_t motor_lb_rpm,int16_t motor_rb_rpm,float chassis_yaw)
{
    nac_instance->send_data->motor_lf_rpm = motor_lf_rpm;  // 左前轮转速
    nac_instance->send_data->motor_rf_rpm = motor_rf_rpm;  // 右前轮转速
    nac_instance->send_data->motor_lb_rpm = motor_lb_rpm;  // 左后轮转速
    nac_instance->send_data->motor_rb_rpm = motor_rb_rpm;  // 右后轮转速
    nac_instance->send_data->chassis_yaw  = chassis_yaw;   // 底盘航向角
}



// ======================== 私有函数实现 ========================

extern uint16_t CRC_INIT;  // CRC校验初始值（外部定义）

/**
 * @brief 处理NAC设备接收到的数据包
 * @param recv 接收数据结构体指针
 * @param rx_buff 原始接收数据缓冲区
 * @details 从原始字节流中解析出结构化数据
 *          数据格式：小端序，每个数据占用2字节
 *          字节1-2: x轴控制量
 *          字节3-4: y轴控制量  
 *          字节5-6: z轴控制量
 *          字节7-8: 零点/校准值
 */
static void RecvProcess(Nac_Recv_s *recv, uint8_t *rx_buff)
{
    recv->x    = (rx_buff[2]<<8) | rx_buff[1];  // 解析x轴数据（小端序）
    recv->y    = (rx_buff[4]<<8) | rx_buff[3];  // 解析y轴数据（小端序）
    recv->z    = (rx_buff[6]<<8) | rx_buff[5];  // 解析z轴数据（小端序）
    recv->zero = (rx_buff[8]<<8) | rx_buff[7];  // 解析零点数据（小端序）
}

/**
 * @brief NAC数据解码函数，串口接收回调函数
 * @details 该函数在串口接收到完整数据包时被调用
 *          功能：
 *          1. 重载守护进程计数器，表示通信正常
 *          2. 验证数据包的帧头和帧尾
 *          3. 调用数据解析函数处理有效数据
 */
static void DecodeNac()
{
    DaemonReload(nac_daemon_instance); // 重载守护进程，表示通信活跃

    // 验证数据包格式：检查帧头(0xAA)和帧尾(0x55)
    if (nac_instance->usart->recv_buff[0] == nac_instance->recv_data->header1 && 
        nac_instance->usart->recv_buff[9] == nac_instance->recv_data->tail) {
        // 数据包格式正确，进行数据解析
        RecvProcess(nac_instance->recv_data, nac_instance->usart->recv_buff);
    }
}


/**
 * @brief NAC设备离线回调函数，由daemon.c的守护任务调用
 * @param id NAC实例ID，此处暂未使用
 * @attention 当NAC设备通信超时时，该函数会被调用
 *            这是由于HAL库的问题，当DMA传输出错时可能导致__HAL_LOCK()锁死
 *            通过重新初始化串口服务来解决此问题
 * @details 离线处理策略：重新初始化串口服务，尝试恢复通信
 */
static void NacOfflineCallback(void *id)
{
    // 重新初始化串口服务，尝试恢复NAC通信
    USARTServiceInit(nac_instance->usart);
}



/**
 * @brief 处理要发送给NAC设备的数据包封装
 * @param send 发送数据结构体指针
 * @param tx_buff 发送缓冲区指针
 * @details 将结构化数据封装成字节流格式
 *          数据包格式（12字节）：
 *          字节0: 帧头 0x98
 *          字节1-2: 左前轮转速（大端序）
 *          字节3-4: 右前轮转速（大端序）
 *          字节5-6: 左后轮转速（大端序）
 *          字节7-8: 右后轮转速（大端序）
 *          字节9-10: 航向角*100（大端序，扩大100倍精度）
 *          字节11: 帧尾 0x34
 */
static void SendProcess(Nac_Send_s *send, uint8_t *tx_buff)
{
    tx_buff[0] = send->header;                        // 帧头：0x98
    
    // 左前轮转速数据（大端序发送）
    tx_buff[1] = send->motor_lf_rpm >> 8;            // 高字节
    tx_buff[2] = send->motor_lf_rpm & 0xFF;          // 低字节
    
    // 右前轮转速数据（大端序发送）
    tx_buff[3] = send->motor_rf_rpm >> 8;            // 高字节
    tx_buff[4] = send->motor_rf_rpm & 0xFF;          // 低字节
    
    // 左后轮转速数据（大端序发送）
    tx_buff[5] = send->motor_lb_rpm >> 8;            // 高字节
    tx_buff[6] = send->motor_lb_rpm & 0xFF;          // 低字节
    
    // 右后轮转速数据（大端序发送）
    tx_buff[7] = send->motor_rb_rpm >> 8;            // 高字节
    tx_buff[8] = send->motor_rb_rpm & 0xFF;          // 低字节
    
    // 航向角数据处理：扩大100倍以保持小数精度
    int16_t scaled = (int16_t)(send->chassis_yaw * 100);
    tx_buff[9]  = scaled >> 8;                       // 高字节
    tx_buff[10] = scaled & 0xFF;                     // 低字节
    
    tx_buff[11] = send->tail;                        // 帧尾：0x34
}

/**
 * @brief 向NAC设备发送数据包
 * @details 将当前存储的发送数据打包并通过串口发送给NAC设备
 *          使用阻塞模式发送，确保数据发送完成
 * @note 该函数应当在主循环或定时任务中周期性调用
 */
void NacSend()
{
    static uint8_t send_buff[VISION_SEND_SIZE];  // 静态发送缓冲区
    
    // 将结构化数据打包成字节流
    SendProcess(nac_instance->send_data, send_buff);
    
    // 通过串口发送数据包（阻塞模式）
    USARTSend(nac_instance->usart, send_buff, VISION_SEND_SIZE, USART_TRANSFER_BLOCKING);
}

// ======================== 注册和初始化函数 ========================

/**
 * @brief 注册NAC接收数据结构，动态分配内存并初始化
 * @param recv_config 接收配置结构体指针
 * @return Nac_Recv_s* 返回分配的接收数据结构指针
 * @details 该函数创建并初始化接收数据结构
 *          包括设置帧头、帧尾等通信协议参数
 */
Nac_Recv_s *NacRecvRegister(Nac_Recv_Init_Config_s *recv_config)
{
    // 动态分配接收数据结构内存
    Nac_Recv_s *recv_data = (Nac_Recv_s *)malloc(sizeof(Nac_Recv_s));
    memset(recv_data, 0, sizeof(Nac_Recv_s));  // 清零初始化

    // 设置通信协议参数
    recv_data->header1 = recv_config->header1;  // 帧头1：0xAA
    recv_data->tail    = recv_config->tail;     // 帧尾：0x55

    return recv_data;
}

/**
 * @brief 注册NAC发送数据结构，动态分配内存并初始化  
 * @param send_config 发送配置结构体指针
 * @return Nac_Send_s* 返回分配的发送数据结构指针
 * @details 该函数创建并初始化发送数据结构
 *          包括设置帧头、帧尾等通信协议参数
 */
Nac_Send_s *NacSendRegister(Nac_Send_Init_Config_s *send_config)
{
    // 动态分配发送数据结构内存
    Nac_Send_s *send_data = (Nac_Send_s *)malloc(sizeof(Nac_Send_s));
    memset(send_data, 0, sizeof(Nac_Send_s));  // 清零初始化

    // 设置通信协议参数
    send_data->header = send_config->header;   // 帧头：0x98
    send_data->tail   = send_config->tail;     // 帧尾：0x34
    
    return send_data;
}

/**
 * @brief NAC通信模块初始化函数
 * @param nac_usart_handle UART句柄指针，指定使用的串口外设
 * @return Nac_Recv_s* 返回NAC接收数据结构指针，供外部访问接收到的数据
 * @details 完整的NAC模块初始化流程：
 *          1. 创建NAC实例并分配内存
 *          2. 配置串口服务（接收回调、缓冲区大小等）
 *          3. 初始化接收数据结构（设置帧头帧尾）
 *          4. 初始化发送数据结构（设置帧头帧尾）
 *          5. 注册守护进程用于通信监控
 */
Nac_Recv_s *NacInit(UART_HandleTypeDef *nac_usart_handle)
{
    // 1. 创建NAC实例
    nac_instance = (Nac_Instance *)malloc(sizeof(Nac_Instance));
    memset(nac_instance, 0, sizeof(Nac_Instance));  // 清零初始化
    
    // 2. 配置串口服务
    USART_Init_Config_s conf;
    conf.module_callback = DecodeNac;            // 设置接收回调函数
    conf.recv_buff_size  = VISION_RECV_SIZE;     // 设置接收缓冲区大小：24字节
    conf.usart_handle    = nac_usart_handle;     // 绑定UART句柄

    nac_instance->usart = USARTRegister(&conf);  // 注册串口服务

    // 3. 初始化接收数据结构
    Nac_Recv_Init_Config_s recv_config = {
        .header1 = NAC_RECV_HEADER1,  // 帧头1：0xAA
        .tail    = NAC_RECV_TAIL,     // 帧尾：0x55
    };
    nac_instance->recv_data = NacRecvRegister(&recv_config);

    // 4. 初始化发送数据结构  
    Nac_Send_Init_Config_s send_config = {
        .header = NAC_SEND_HEADER,    // 帧头：0x98
        .tail   = NAC_SEND_TAIL,      // 帧尾：0x34
    };
    nac_instance->send_data = NacSendRegister(&send_config);

    // 5. 注册守护进程，监控通信状态
    Daemon_Init_Config_s daemon_conf = {
        .callback     = NacOfflineCallback,  // 离线回调函数
        .owner_id     = NULL,                // 所有者ID（暂不使用）
        .reload_count = 5,                   // 重载计数：50ms超时检测
    };
    nac_daemon_instance = DaemonRegister(&daemon_conf);

    return nac_instance->recv_data;  // 返回接收数据指针供外部使用
}
