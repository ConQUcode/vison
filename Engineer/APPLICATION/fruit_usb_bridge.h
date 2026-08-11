#ifndef FRUIT_USB_BRIDGE_H
#define FRUIT_USB_BRIDGE_H

#include <stdint.h>

#include "protocol.h"

/* 供上层读取的最新水果识别快照；valid=0时不得使用旧结果执行业务。 */
typedef struct {
    uint8_t fruit_id;
    uint8_t status;
    uint8_t valid;
    uint32_t update_seq;
    uint32_t rx_tick_ms;
} Fruit_Detection_s;

/* Watch只保留连接、最新结果和必要错误计数，不包含机械臂控制字段。 */
typedef struct {
    uint8_t initialized;
    uint8_t handshake_ok;
    uint8_t link_online;
    uint8_t result_valid;
    uint8_t fruit_id;
    uint8_t status;
    uint32_t update_seq;
    uint32_t rx_tick_ms;
    uint32_t valid_count;
    uint32_t no_target_count;
    uint32_t invalid_count;
    uint32_t handshake_mismatch_count;
    uint32_t heartbeat_timeout_count;
} Fruit_Usb_Debug_s;

extern Fruit_Usb_Debug_s g_fruit_usb_debug;

void FruitUsbBridgeInit(void);
void FruitUsbBridgeTask(uint32_t now_ms);
uint8_t FruitUsbBridgeGetLatest(Fruit_Detection_s *result);
void FruitUsbBridgeOnDetection(const Packet_FruitDetection *packet);

#endif
