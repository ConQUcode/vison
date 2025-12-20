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

/* 全局变量 */
USB_Chassis_Cmd_s usb_chassis_cmd;
uint32_t usb_last_recv_time = 0;

/* 私有变量 */
// 环形缓冲区定义
#define RING_BUFFER_SIZE 1024
static uint8_t ring_buffer[RING_BUFFER_SIZE];
static volatile uint32_t rb_head = 0; // 写入位置
static volatile uint32_t rb_tail = 0; // 读取位置

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
 * @brief 预览环形缓冲区中的数据（不移动tail指针）
 * @param offset 偏移量
 * @param data 输出指针
 * @return 1:成功, 0:越界
 */
static uint8_t RingBuffer_Peek(uint32_t offset, uint8_t *data)
{
    uint32_t count = (rb_head >= rb_tail) ? (rb_head - rb_tail) : (RING_BUFFER_SIZE - rb_tail + rb_head);
    
    if (offset >= count)
    {
        return 0;
    }
    
    uint32_t index = (rb_tail + offset) % RING_BUFFER_SIZE;
    *data = ring_buffer[index];
    return 1;
}

/**
 * @brief USB数据解析任务
 * @note 建议在主循环或任务中周期性调用
 */
void USB_ProcessTask(void)
{
    uint8_t byte;
    
    // 循环处理缓冲区中的数据
    while (1)
    {
        // 1. 寻找帧头
        if (!RingBuffer_Peek(0, &byte)) break; // 缓冲区空
        
        if (byte != USB_FRAME_HEADER)
        {
            // 不是帧头，丢弃该字节，继续寻找
            RingBuffer_Read(&byte);
            continue;
        }
        
        // 2. 检查缓冲区是否有足够的数据构成一帧
        uint32_t count = (rb_head >= rb_tail) ? (rb_head - rb_tail) : (RING_BUFFER_SIZE - rb_tail + rb_head);
        if (count < USB_FRAME_LEN)
        {
            // 数据不够，等待下次处理
            break;
        }
        
        // 3. 预读取完整帧进行校验
        uint8_t frame[USB_FRAME_LEN];
        for (uint32_t i = 0; i < USB_FRAME_LEN; i++)
        {
            RingBuffer_Peek(i, &frame[i]);
        }
        
        // 4. 校验帧格式
        if (frame[1] == USB_FUNC_CONTROL && 
            frame[2] == USB_DATA_LEN && 
            frame[12] == USB_FRAME_FOOTER)
        {
            // 计算校验和
            uint8_t checksum = 0;
            for (int j = 1; j <= 10; j++) {
                checksum += frame[j];
            }
            
            if (checksum == frame[11])
            {
                // 校验通过，提取数据
                memcpy(&usb_chassis_cmd.linear_x, &frame[3], 4);
                memcpy(&usb_chassis_cmd.angular_z, &frame[7], 4);
                
                // 更新时间戳
                usb_last_recv_time = HAL_GetTick();

                // [DEBUG] 回显功能：将接收到的原始数据帧原样发回
                // TODO: 调试完成后请删除此行
                USB_Transmit(frame, USB_FRAME_LEN);
                
                // 从缓冲区中移除该帧
                for (uint32_t i = 0; i < USB_FRAME_LEN; i++)
                {
                    RingBuffer_Read(&byte);
                }
                
                // 继续处理下一帧
                continue;
            }
        }
        
        // 校验失败或格式错误，丢弃帧头，尝试下一字节
        RingBuffer_Read(&byte);
    }
}

