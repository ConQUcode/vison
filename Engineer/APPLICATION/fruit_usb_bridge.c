/**
 * @file fruit_usb_bridge.c
 * @brief 保存上位机水果识别快照；当前只供 Watch 观察，不控制执行机构。
 */

#include "fruit_usb_bridge.h"

#include "protocol_runtime.h"

#include <string.h>

Fruit_Usb_Debug_s g_fruit_usb_debug;

static Fruit_Detection_s latest_detection;
static uint32_t last_protocol_session;

static void FruitUsbBridgeInvalidate(void)
{
    latest_detection.valid = 0u;
    g_fruit_usb_debug.result_valid = 0u;
}

static void FruitUsbBridgeSyncSession(
    const Protocol_Runtime_Debug_s *protocol)
{
    if (protocol == NULL || protocol->session_count == last_protocol_session) {
        return;
    }
    last_protocol_session = protocol->session_count;
    FruitUsbBridgeInvalidate();
}

void FruitUsbBridgeInit(void)
{
    memset(&latest_detection, 0, sizeof(latest_detection));
    memset(&g_fruit_usb_debug, 0, sizeof(g_fruit_usb_debug));
    last_protocol_session = 0u;
    g_fruit_usb_debug.initialized = 1u;
}

void FruitUsbBridgeTask(uint32_t now_ms)
{
    const Protocol_Runtime_Debug_s *protocol = ProtocolRuntimeGetDebug();

    (void)now_ms;
    if (protocol == NULL) {
        FruitUsbBridgeInvalidate();
        return;
    }

    FruitUsbBridgeSyncSession(protocol);
    g_fruit_usb_debug.handshake_ok = protocol->connection_ready;
    g_fruit_usb_debug.link_online = protocol->link_online;
    g_fruit_usb_debug.handshake_mismatch_count =
        protocol->handshake_mismatch_count;
    g_fruit_usb_debug.heartbeat_timeout_count =
        protocol->heartbeat_timeout_count;

    if (protocol->connection_ready == 0u || protocol->link_online == 0u) {
        /* 断线只使上位机数据失效，不控制主臂、夹爪或独立HOME流程。 */
        FruitUsbBridgeInvalidate();
    }
}

uint8_t FruitUsbBridgeGetLatest(Fruit_Detection_s *result)
{
    uint8_t valid;

    if (result == NULL) {
        return 0u;
    }
    *result = latest_detection;
    valid = latest_detection.valid != 0u &&
        ProtocolRuntimeConnectionReady() != 0u &&
        ProtocolRuntimeLinkOnline() != 0u;
    if (valid == 0u) {
        result->valid = 0u;
    }
    return valid;
}

void FruitUsbBridgeOnDetection(const Packet_FruitDetection *packet)
{
    const Protocol_Runtime_Debug_s *protocol = ProtocolRuntimeGetDebug();
    uint32_t now_ms = ProtocolRuntimeNowMs();

    if (packet == NULL) {
        g_fruit_usb_debug.invalid_count++;
        FruitUsbBridgeInvalidate();
        return;
    }
    ProtocolRuntimeNotifyApplicationRx();
    if (ProtocolRuntimeConnectionReady() == 0u ||
        ProtocolRuntimeLinkOnline() == 0u) {
        g_fruit_usb_debug.invalid_count++;
        FruitUsbBridgeInvalidate();
        return;
    }

    FruitUsbBridgeSyncSession(protocol);
    latest_detection.fruit_id = packet->fruit_id;
    latest_detection.status = packet->status;
    latest_detection.rx_tick_ms = now_ms;
    latest_detection.update_seq++;

    g_fruit_usb_debug.fruit_id = packet->fruit_id;
    g_fruit_usb_debug.status = packet->status;
    g_fruit_usb_debug.rx_tick_ms = now_ms;
    g_fruit_usb_debug.update_seq = latest_detection.update_seq;

    if (packet->fruit_id == 0u && packet->status == 0u) {
        g_fruit_usb_debug.no_target_count++;
        FruitUsbBridgeInvalidate();
        return;
    }
    if (packet->fruit_id < 1u || packet->fruit_id > 6u ||
        packet->status > 1u) {
        g_fruit_usb_debug.invalid_count++;
        FruitUsbBridgeInvalidate();
        return;
    }

    latest_detection.valid = 1u;
    g_fruit_usb_debug.result_valid = 1u;
    g_fruit_usb_debug.valid_count++;
}

void on_receive_FruitDetection(const Packet_FruitDetection *packet)
{
    FruitUsbBridgeOnDetection(packet);
}
