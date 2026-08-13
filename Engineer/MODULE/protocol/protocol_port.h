/**
 * @file protocol_port.h
 * @brief 将上位机生成协议的 serial_write 接口适配到 USB CDC 发送队列。
 */

#ifndef PROTOCOL_PORT_H
#define PROTOCOL_PORT_H

#include <stdint.h>

/** 返回最近一次协议帧是否成功进入 USB 发送队列。 */
uint8_t ProtocolPortLastWriteOk(void);

#endif
