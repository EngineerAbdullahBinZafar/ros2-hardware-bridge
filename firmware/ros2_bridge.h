/*
 * ros2_bridge.h - Lightweight Serial Protocol for ROS2-to-Embedded Communication
 * Author: Engineer Abdullah Bin Zafar
 */

#ifndef ROS2_BRIDGE_H
#define ROS2_BRIDGE_H

#include <stdint.h>
#include <string.h>

#define BRIDGE_START_BYTE1 0xAA
#define BRIDGE_START_BYTE2 0x55

typedef struct {
    uint8_t msg_id;
    uint8_t len;
    uint8_t payload[64];
    uint8_t checksum;
} BridgePacket;

// Simple Checksum
static uint8_t calculate_checksum(uint8_t id, uint8_t len, uint8_t* data) {
    uint8_t sum = id + len;
    for (uint8_t i = 0; i < len; i++) sum += data[i];
    return sum;
}

// Prepare a packet for transmission
static int bridge_serialize(uint8_t id, uint8_t* data, uint8_t len, uint8_t* out_buf) {
    out_buf[0] = BRIDGE_START_BYTE1;
    out_buf[1] = BRIDGE_START_BYTE2;
    out_buf[2] = id;
    out_buf[3] = len;
    memcpy(&out_buf[4], data, len);
    out_buf[4 + len] = calculate_checksum(id, len, data);
    return 5 + len;
}

#endif // ROS2_BRIDGE_H
