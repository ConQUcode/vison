/**
 * @file usb.h
 * @author your name
 * @brief USB虚拟串口通信模块
 * @version 0.1
 * @date 2025-12-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef USB_H
#define USB_H

#include "stdint.h"
#include "usbd_cdc_if.h"

/* 协议常量定义 */
#define USB_FRAME_HEADER    0xAA
#define USB_FRAME_FOOTER    0x55
#define USB_FUNC_CONTROL    0x02
#define USB_DATA_LEN        0x08
#define USB_FRAME_LEN       13

/* 接收缓冲区大小 */
#define USB_RX_BUFFER_SIZE 512

/* USB接收数据回调函数类型 */
typedef void (*usb_rx_callback_t)(uint8_t *data, uint32_t len);

/* 底盘控制指令结构体 */
#pragma pack(1)
typedef struct {
    float linear_x;   // 线速度 m/s
    float linear_y;   // 横移速度 m/s
    float angular_z;  // 角速度 rad/s
} USB_Chassis_Cmd_s;
#pragma pack()

/* 外部变量声明 */
extern USB_Chassis_Cmd_s usb_chassis_cmd;
extern uint32_t usb_last_recv_time;

/**
 * @brief USB模块初始化
 * 
 */
void USB_Init(void);

/**
 * @brief 注册USB接收数据回调函数
 * 
 * @param callback 回调函数指针
 */
void USB_RegisterRxCallback(usb_rx_callback_t callback);

/**
 * @brief 通过USB发送数据
 * 
 * @param data 要发送的数据指针
 * @param len 数据长度
 * @return uint8_t 发送状态 (USBD_OK, USBD_BUSY, USBD_FAIL)
 */
uint8_t USB_Transmit(uint8_t *data, uint16_t len);

/**
 * @brief 通过USB发送字符串
 * 
 * @param str 字符串指针
 * @return uint8_t 发送状态
 */
uint8_t USB_TransmitString(const char *str);

/**
 * @brief USB接收数据处理(内部函数,由usbd_cdc_if.c调用)
 * 
 * @param buf 接收到的数据缓冲区
 * @param len 数据长度
 */
void USB_RxHandler(uint8_t *buf, uint32_t len);

/**
 * @brief USB数据解析任务(建议在主循环或任务中调用)
 */
void USB_ProcessTask(void);

#endif // USB_H
