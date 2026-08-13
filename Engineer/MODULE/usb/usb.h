/**
 * @file usb.h
 * @author your name
 * @brief USB CDC 接收环形缓冲区和双优先级拷贝发送队列。
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

/* 接收缓冲区大小 */
#define USB_RX_BUFFER_SIZE 512

/* USB接收数据回调函数类型 */
typedef void (*usb_rx_callback_t)(uint8_t *data, uint32_t len);

extern volatile uint32_t g_usb_rx_overflow_count;
extern volatile uint32_t g_usb_tx_fail_count;

typedef struct {
    uint8_t busy;
    uint8_t active_high_priority;
    uint8_t high_queue_count;
    uint8_t normal_queue_count;
    uint8_t device_state;
    uint8_t cdc_tx_state;
    uint32_t tx_start_tick;
    uint32_t enqueue_count;
    uint32_t complete_count;
    uint32_t timeout_count;
    uint32_t reset_count;
    uint32_t dropped_count;
    uint32_t stale_complete_count;
} USB_Tx_Debug_s;

extern USB_Tx_Debug_s g_usb_tx_debug;

/**
 * @brief 初始化 USB 软件缓冲和发送队列；不调用 MX_USB_DEVICE_Init。
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
uint8_t USB_TransmitCopy(const uint8_t *data, uint16_t len);
uint8_t USB_TransmitCopyHighPriority(const uint8_t *data, uint16_t len);
void USB_TxTask(void);
void USB_TxCompleteHandler(uint8_t *data);
void USB_ConnectionResetHandler(void);

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
 * @brief 在 UsbTask 中把接收环形缓冲逐字节交给协议 Runtime。
 */
void USB_ProcessTask(void);

/** 协议端口使用的拷贝发送接口，high_priority 非零时进入系统队列。 */
uint8_t USB_ProtocolWrite(const uint8_t *data,
                          uint16_t len,
                          uint8_t high_priority);

#endif // USB_H
