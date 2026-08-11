/**
 * @file usb.c
 * @author your name
 * @brief USB虚拟串口通信模块实现
 * @version 0.1
 * @date 2025-12-18
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "usb.h"
#include "string.h"
#include "stdio.h"
#include "usb_device.h"
#include "main.h" // 确保可以使用 HAL_GetTick()
#include "protocol.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

/* 全局变量 */
USB_Chassis_Cmd_s usb_chassis_cmd;
uint32_t usb_last_recv_time = 0;
volatile uint32_t g_usb_rx_overflow_count = 0;
volatile uint32_t g_usb_tx_fail_count = 0;
USB_Tx_Debug_s g_usb_tx_debug;

/* 私有变量 */
// 环形缓冲区定义
#define RING_BUFFER_SIZE 1024
#define USB_TX_NORMAL_QUEUE_DEPTH 8u
#define USB_TX_HIGH_QUEUE_DEPTH 4u
#define USB_TX_ITEM_MAX_LEN 64u
#define USB_TX_TIMEOUT_MS 100u
#define USB_TX_RECOVERY_GUARD_MS 20u
static uint8_t ring_buffer[RING_BUFFER_SIZE];
static volatile uint32_t rb_head = 0; // 写入位置
static volatile uint32_t rb_tail = 0; // 读取位置
typedef struct {
    uint8_t data[USB_TX_ITEM_MAX_LEN];
    uint16_t len;
} USB_Tx_Item_s;

static USB_Tx_Item_s tx_normal_queue[USB_TX_NORMAL_QUEUE_DEPTH];
static USB_Tx_Item_s tx_high_queue[USB_TX_HIGH_QUEUE_DEPTH];
static volatile uint8_t tx_normal_head = 0u;
static volatile uint8_t tx_normal_count = 0u;
static volatile uint8_t tx_high_head = 0u;
static volatile uint8_t tx_high_count = 0u;
static volatile uint8_t tx_busy = 0u;
static volatile uint8_t tx_active_high_priority = 0u;
static volatile uint8_t connection_reset_pending = 0u;
static uint32_t tx_start_tick = 0u;
static uint32_t tx_recovery_until_tick = 0u;

static usb_rx_callback_t rx_callback = NULL;       // 接收回调函数
static uint8_t usb_initialized = 0;                // 初始化标志

/**
 * @brief USB模块初始化
 */
void USB_Init(void)
{
    if (!usb_initialized)
    {
        // USB外设初始化在main.c中已经调用MX_USB_DEVICE_Init()完成
        // 这里只需要初始化模块内部变量
        memset(ring_buffer, 0, RING_BUFFER_SIZE);
        rb_head = 0;
        rb_tail = 0;
        tx_normal_head = 0u;
        tx_normal_count = 0u;
        tx_high_head = 0u;
        tx_high_count = 0u;
        tx_busy = 0u;
        tx_active_high_priority = 0u;
        connection_reset_pending = 0u;
        tx_start_tick = 0u;
        tx_recovery_until_tick = 0u;
        g_usb_rx_overflow_count = 0u;
        g_usb_tx_fail_count = 0u;
        memset(&g_usb_tx_debug, 0, sizeof(g_usb_tx_debug));
        rx_callback = NULL;
        usb_initialized = 1;
        
        // 初始化指令数据
        memset(&usb_chassis_cmd, 0, sizeof(usb_chassis_cmd));
        usb_last_recv_time = 0;
    }
}

/**
 * @brief 注册USB接收数据回调函数
 */
void USB_RegisterRxCallback(usb_rx_callback_t callback)
{
    rx_callback = callback;
}

/**
 * @brief 通过USB发送数据
 */
uint8_t USB_Transmit(uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
        return USBD_FAIL;
    
    return CDC_Transmit_FS(data, len);
}

static uint8_t USB_TransmitCopyToQueue(const uint8_t *data, uint16_t len,
                                      uint8_t high_priority)
{
    uint32_t primask;
    uint8_t tail;

    if (data == NULL || len == 0u || len > USB_TX_ITEM_MAX_LEN) {
        g_usb_tx_fail_count++;
        return USBD_FAIL;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (high_priority != 0u) {
        if (tx_high_count >= USB_TX_HIGH_QUEUE_DEPTH) {
            if (primask == 0u) {
                __enable_irq();
            }
            g_usb_tx_fail_count++;
            return USBD_BUSY;
        }
        tail = (uint8_t)((tx_high_head + tx_high_count) %
                         USB_TX_HIGH_QUEUE_DEPTH);
        memcpy(tx_high_queue[tail].data, data, len);
        tx_high_queue[tail].len = len;
        tx_high_count++;
    } else {
        if (tx_normal_count >= USB_TX_NORMAL_QUEUE_DEPTH) {
            if (primask == 0u) {
                __enable_irq();
            }
            g_usb_tx_fail_count++;
            return USBD_BUSY;
        }
        tail = (uint8_t)((tx_normal_head + tx_normal_count) %
                         USB_TX_NORMAL_QUEUE_DEPTH);
        memcpy(tx_normal_queue[tail].data, data, len);
        tx_normal_queue[tail].len = len;
        tx_normal_count++;
    }
    g_usb_tx_debug.enqueue_count++;
    if (primask == 0u) {
        __enable_irq();
    }
    return USBD_OK;
}

uint8_t USB_TransmitCopy(const uint8_t *data, uint16_t len)
{
    return USB_TransmitCopyToQueue(data, len, 0u);
}

uint8_t USB_TransmitCopyHighPriority(const uint8_t *data, uint16_t len)
{
    return USB_TransmitCopyToQueue(data, len, 1u);
}

void USB_TxCompleteHandler(uint8_t *data)
{
    uint8_t *expected_data = NULL;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (tx_busy != 0u && tx_active_high_priority != 0u &&
        tx_high_count != 0u) {
        expected_data = tx_high_queue[tx_high_head].data;
    } else if (tx_busy != 0u && tx_active_high_priority == 0u &&
               tx_normal_count != 0u) {
        expected_data = tx_normal_queue[tx_normal_head].data;
    }
    if (tx_busy == 0u || expected_data == NULL || data != expected_data) {
        g_usb_tx_debug.stale_complete_count++;
        if (primask == 0u) {
            __enable_irq();
        }
        return;
    }
    if (tx_active_high_priority != 0u) {
        tx_high_head = (uint8_t)((tx_high_head + 1u) %
                                 USB_TX_HIGH_QUEUE_DEPTH);
        tx_high_count--;
    } else {
        tx_normal_head = (uint8_t)((tx_normal_head + 1u) %
                                   USB_TX_NORMAL_QUEUE_DEPTH);
        tx_normal_count--;
    }
    g_usb_tx_debug.complete_count++;
    tx_busy = 0u;
    tx_active_high_priority = 0u;
    tx_start_tick = 0u;
    if (primask == 0u) {
        __enable_irq();
    }
}

void USB_TxTask(void)
{
    USBD_CDC_HandleTypeDef *hcdc;
    USB_Tx_Item_s *item;
    uint32_t now_ms = HAL_GetTick();
    uint32_t primask;
    uint8_t result;

    hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    g_usb_tx_debug.busy = tx_busy;
    g_usb_tx_debug.active_high_priority = tx_active_high_priority;
    g_usb_tx_debug.high_queue_count = tx_high_count;
    g_usb_tx_debug.normal_queue_count = tx_normal_count;
    g_usb_tx_debug.device_state = hUsbDeviceFS.dev_state;
    g_usb_tx_debug.cdc_tx_state =
        hcdc != NULL && hcdc->TxState != 0u ? 1u : 0u;
    g_usb_tx_debug.tx_start_tick = tx_start_tick;

    if (tx_busy != 0u) {
        if ((uint32_t)(now_ms - tx_start_tick) < USB_TX_TIMEOUT_MS) {
            return;
        }

        /*
         * 主机断开或完成回调丢失时，不能让应用层永久停在busy。
         * 丢弃这一份在途副本；可靠状态会由protocol重试，主机命令
         * 则会使用相同ack_seq重发。
         */
        primask = __get_PRIMASK();
        __disable_irq();
        if (tx_active_high_priority != 0u && tx_high_count != 0u) {
            tx_high_head = (uint8_t)((tx_high_head + 1u) %
                                     USB_TX_HIGH_QUEUE_DEPTH);
            tx_high_count--;
        } else if (tx_active_high_priority == 0u &&
                   tx_normal_count != 0u) {
            tx_normal_head = (uint8_t)((tx_normal_head + 1u) %
                                       USB_TX_NORMAL_QUEUE_DEPTH);
            tx_normal_count--;
        }
        tx_busy = 0u;
        tx_active_high_priority = 0u;
        tx_start_tick = 0u;
        tx_recovery_until_tick = now_ms + USB_TX_RECOVERY_GUARD_MS;
        g_usb_tx_debug.timeout_count++;
        g_usb_tx_debug.dropped_count++;
        if (hcdc != NULL) {
            hcdc->TxState = 0u;
        }
        if (primask == 0u) {
            __enable_irq();
        }
        if (hUsbDeviceFS.pData != NULL) {
            (void)HAL_PCD_EP_Flush((PCD_HandleTypeDef *)hUsbDeviceFS.pData,
                                   CDC_IN_EP);
        }
        return;
    }

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || hcdc == NULL) {
        return;
    }
    if ((int32_t)(now_ms - tx_recovery_until_tick) < 0) {
        return;
    }
    if (tx_high_count != 0u) {
        item = &tx_high_queue[tx_high_head];
        tx_active_high_priority = 1u;
    } else if (tx_normal_count != 0u) {
        item = &tx_normal_queue[tx_normal_head];
        tx_active_high_priority = 0u;
    } else {
        return;
    }
    /* 先标记在途，避免极短USB传输的完成中断早于任务侧置busy。 */
    tx_busy = 1u;
    tx_start_tick = now_ms;
    result = CDC_Transmit_FS(item->data, item->len);
    if (result != USBD_OK) {
        tx_busy = 0u;
        tx_active_high_priority = 0u;
        tx_start_tick = 0u;
        if (result == USBD_FAIL) {
            g_usb_tx_fail_count++;
        }
    }
}

void USB_ConnectionResetHandler(void)
{
    USBD_CDC_HandleTypeDef *hcdc;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    g_usb_tx_debug.dropped_count +=
        (uint32_t)tx_high_count + (uint32_t)tx_normal_count;
    tx_high_head = 0u;
    tx_high_count = 0u;
    tx_normal_head = 0u;
    tx_normal_count = 0u;
    tx_busy = 0u;
    tx_active_high_priority = 0u;
    tx_start_tick = 0u;
    tx_recovery_until_tick = HAL_GetTick() + USB_TX_RECOVERY_GUARD_MS;
    g_usb_tx_debug.reset_count++;
    connection_reset_pending = 1u;
    hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (hcdc != NULL) {
        hcdc->TxState = 0u;
    }
    if (primask == 0u) {
        __enable_irq();
    }
}

/**
 * @brief 通过USB发送字符串
 */
uint8_t USB_TransmitString(const char *str)
{
    if (str == NULL)
        return USBD_FAIL;
    
    uint16_t len = strlen(str);
    return CDC_Transmit_FS((uint8_t*)str, len);
}

/**
 * @brief USB接收数据处理(由usbd_cdc_if.c的CDC_Receive_FS调用)
 * @note 仅负责将数据存入环形缓冲区，不做解析
 */
void USB_RxHandler(uint8_t *buf, uint32_t len)
{
    if (len > 0)
    {
        // 将数据存入环形缓冲区
        for (uint32_t i = 0; i < len; i++)
        {
            uint32_t next_head = (rb_head + 1) % RING_BUFFER_SIZE;
            if (next_head != rb_tail) // 缓冲区未满
            {
                ring_buffer[rb_head] = buf[i];
                rb_head = next_head;
            }
            else
            {
                // 缓冲区溢出，丢弃剩余数据
                g_usb_rx_overflow_count += (len - i);
                break;
            }
        }
        
        // 调用用户注册的回调函数 (仅通知有新数据，不建议在此做耗时操作)
        if (rx_callback != NULL)
        {
            rx_callback(buf, len);
        }
    }
}

/**
 * @brief 从环形缓冲区读取一个字节
 * @param data 输出指针
 * @return 1:成功, 0:缓冲区空
 */
static uint8_t RingBuffer_Read(uint8_t *data)
{
    if (rb_head == rb_tail)
    {
        return 0;
    }
    
    *data = ring_buffer[rb_tail];
    rb_tail = (rb_tail + 1) % RING_BUFFER_SIZE;
    return 1;
}

/**
 * @brief USB数据解析任务
 * @note 建议在主循环或任务中周期性调用
 */
void USB_ProcessTask(void)
{
    uint8_t byte;
    uint32_t primask;

    if (connection_reset_pending != 0u) {
        primask = __get_PRIMASK();
        __disable_irq();
        connection_reset_pending = 0u;
        if (primask == 0u) {
            __enable_irq();
        }
        /* 协议状态只在USB任务上下文复位，避免CDC中断与解析并发。 */
        protocol_reset_connection();
    }
    
    // 循环处理缓冲区中的数据
    while (RingBuffer_Read(&byte))
    {
        protocol_fsm_feed(byte);
    }
}

uint8_t serial_write(const uint8_t *data, uint16_t len)
{
    uint8_t high_priority = 0u;

    if (data != NULL && len >= 3u) {
        /* ACK独占高优先队列，不让心跳或状态重试挤占命令确认空间。 */
        high_priority = data[2] == PACKET_ID_ACK;
    }
    return (high_priority != 0u ?
            USB_TransmitCopyHighPriority(data, len) :
            USB_TransmitCopy(data, len)) == USBD_OK ? 1u : 0u;
}
