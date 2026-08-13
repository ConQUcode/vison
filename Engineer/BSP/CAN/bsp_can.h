/**
 * @file bsp_can.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief CAN1/CAN2 统一注册、过滤器、回调派发和发送接口。
 * @version 0.1
 * @date 2024-01-07
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include <stdint.h>
#include "can.h"

// 最多能够支持的CAN设备数
#define CAN_MX_REGISTER_CNT 16       // 这个数量取决于CAN总线的负载
#define MX_CAN_FILTER_CNT   (2 * 14) // 最多可以使用的CAN过滤器数量,目前远不会用到这么多
#define DEVICE_CAN_CNT      2        // 根据板子设定,F407IG有CAN1,CAN2,因此为2;F334只有一个,则设为1
// 如果只有1个CAN,还需要把bsp_can.c中所有的hcan2变量改为hcan1(别担心,主要是总线和FIFO的负载均衡,不影响功能)

#pragma pack(1) // 1字节对齐

/* CAN的实例结构体 */
typedef struct can_instance {
    CAN_HandleTypeDef *can_handle; // can句柄
    CAN_TxHeaderTypeDef txconf;    // CAN报文发送配置
    uint32_t tx_id;                // CAN报文发送ID
    uint32_t tx_mailbox;           // CAN报文发送邮箱
    uint8_t tx_buff[8];            // 发送缓存,发送消息长度可以通过CANSetDLC()设定,最大为8
    uint8_t rx_buff[8];            // 接收缓存,最大消息长度为8
    uint32_t rx_id;                // 接收id
    uint8_t rx_len;                // 接收长度,可能为0-8

    // 接收的回调函数,用于解析接收到的数据
    void (*can_module_callback)(struct can_instance *); // callback needs an instance to tell among registered ones
    void *id;                                           // 使用can外设的模块指针(即id指向的模块拥有此can实例,是父子关系)

} CAN_Instance;
#pragma pack()

/* CAN初始化实例结构体 */
typedef struct
{
    CAN_HandleTypeDef *can_handle;               // can句柄
    uint32_t tx_id;                              // 发送id
    uint32_t rx_id;                              // 接收id
    void (*can_module_callback)(CAN_Instance *); // 处理接收数据的回调函数
    void *id;                                    // 拥有can实例的模块地址,用于区分不同的模块(如果有需要的话),如果不需要可以不传入
} CAN_Init_Config_s;

/**
 * @brief 注册一个 CAN 实例；第一次注册时启动 CAN1/CAN2 和接收中断。
 * @param config 收发 ID、句柄、回调和模块所有者配置。
 * @return 成功返回实例指针；容量不足或参数错误返回 NULL。
 */
CAN_Instance *CANRegister(CAN_Init_Config_s *config);

/**
 * @brief transmit mesg through CAN device,通过can实例发送消息
 *        发送前需要向CAN实例的tx_buff写入发送数据
 *
 * @attention 超时时间不应该超过调用此函数的任务的周期,否则会导致任务阻塞
 *
 * @param timeout HAL 发送等待上限，单位 ms，必须小于调用任务周期。
 * @param _instance* can instance owned by module
 */
uint8_t CANTransmit(CAN_Instance *_instance, float timeout);

/**
 * @brief 修改CAN发送报文的数据帧长度;注意最大长度为8,在没有进行修改的时候,默认长度为8
 *
 * @param _instance 要修改长度的can实例
 * @param length    设定长度
 */
void CANSetDLC(CAN_Instance *_instance, uint8_t length);
#endif // BSP_CAN_H
