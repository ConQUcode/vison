/**
 * @file protocol_port.c
 * @brief 协议生成代码与 STM32 USB CDC 传输层之间的稳定适配层。
 */

#include "protocol_port.h"

#include "protocol.h"
#include "usb.h"

static uint8_t protocol_port_last_write_ok;

void serial_write(const uint8_t *data, uint16_t len)
{
    uint8_t high_priority = 0u;

    if (data != NULL && len >= 3u) {
        high_priority = data[2] == PACKET_ID_ACK ||
            data[2] == PACKET_ID_HEARTBEAT ||
            data[2] == PACKET_ID_HANDSHAKE;
    }
    protocol_port_last_write_ok = USB_ProtocolWrite(
        data, len, high_priority);
}

uint8_t ProtocolPortLastWriteOk(void)
{
    return protocol_port_last_write_ok;
}
