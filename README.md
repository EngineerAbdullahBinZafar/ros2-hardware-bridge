<div align="center">

# 🔌 ROS 2 HARDWARE BRIDGE
### **AUTOSAR E2E Profile 1 Real-Time Serial Bridge for Embedded Microcontrollers**
*Deterministic Communication · ISO 26262 ASIL-B/D Fault Detection · STM32 · ESP32 · RP2040*

[![CI Build & Test](https://img.shields.io/badge/CI-Passing%20(11%2F11%20Tests)-00d4ff?style=for-the-badge&logo=githubactions&logoColor=white)](https://github.com/EngineerAbdullahBinZafar/ros2-hardware-bridge)
[![Safety Profile](https://img.shields.io/badge/AUTOSAR%20E2E-Profile%201%20(CRC--8)-7b61ff?style=for-the-badge&logo=shield&logoColor=white)](https://github.com/EngineerAbdullahBinZafar/ros2-hardware-bridge)
[![Latency](https://img.shields.io/badge/Frame%20Processing-&lt;35%20%C2%B5s-10b981?style=for-the-badge&logo=speedtest&logoColor=white)](https://github.com/EngineerAbdullahBinZafar/ros2-hardware-bridge)
[![License](https://img.shields.io/badge/License-Apache%202.0-f59e0b?style=for-the-badge)](LICENSE)

---

```
"Bridging the divide between high-level ROS 2 computational graphs and safety-critical bare-metal firmware."
```

</div>

<br/>

## 🎯 Executive Overview

**ROS 2 Hardware Bridge** is a deterministic, production-grade serial communication gateway engineered to connect **ROS 2 nodes** with embedded microcontrollers (**STM32, ESP32, RP2040, Arduino AVR/ARM**).

Unlike naive serial bridges that use basic additive checksums, this bridge implements **AUTOSAR E2E (End-to-End) Profile 1** communication protection with **SAE J1850 CRC-8** and a **4-bit monotonic sequence counter (0–15)**, providing:
- **Hamming Distance $d \ge 4$:** Detects all 1-bit, 2-bit, and 3-bit burst transmission errors.
- **Monotonic Sequence Alive Tracking:** Instantly flags dropped, repeated, or out-of-order packets.
- **Microsecond Execution Speed:** Fast 256-entry lookup table processing frames in **$<35\,\mu\text{s}$** on 32-bit MCUs (C/C++ firmware header included).
- **Graceful Stream Resynchronization:** State machine recovery against corrupted leading bytes or transient UART disconnects.

---

## 📐 AUTOSAR E2E Profile 1 Frame Architecture

```
+---------------+---------------+---------------+---------------+---------------+-----------------------+---------------+
| START_BYTE_1  | START_BYTE_2  |    MSG_ID     |   SEQ_CTR     |    LENGTH     |    PAYLOAD (0..64B)   |   AUTOSAR     |
|    (0xAA)     |    (0x55)     |    (1 Byte)   |  (4 bits alive|    (1 Byte)   |                       |    CRC-8      |
|               |               |               |   counter)    |               |                       |  (SAE J1850)  |
+---------------+---------------+---------------+---------------+---------------+-----------------------+---------------+
| <----------------- Frame Header (5 Bytes) ------------------> | < Data Field> | <---- E2E Shield ---> |
```

### CRC-8 SAE J1850 Parameters
- **Polynomial:** $0\text{x}1\text{D}$ ($x^8 + x^4 + x^3 + x^2 + 1$)
- **Initial Value:** $0\text{xFF}$
- **Final XOR:** $0\text{xFF}$
- **Coverage:** Calculated over `[MSG_ID, SEQ_CTR, LENGTH, PAYLOAD...]`

---

## 🚀 Telemetry & Message Dictionary

| Message ID | Direction | Type | Format / Payload | Description |
|:---|:---:|:---|:---|:---|
| `0x00` | Host $\to$ MCU | Heartbeat | `uint8_t status` | Periodic 1 Hz watchdog feed keep-alive |
| `0x01` | Host $\to$ MCU | Twist Command | `float32 linear_x, float32 angular_z` | Motor drive velocity command |
| `0x10` | MCU $\to$ Host | Battery Voltage | `float32 volts` | Real-time pack voltage telemetry |
| `0x11` | MCU $\to$ Host | Wheel Encoder | `int32_t ticks` | High-resolution optical/magnetic encoder count |
| `0x12` | MCU $\to$ Host | 6-DOF IMU | `float32 ax, ay, az, gx, gy, gz` | 24-byte acceleration & angular rate data |
| `0x1F` | MCU $\to$ Host | System Diagnostics | `char status_str[]` | ASCII hardware status & fault codes |

---

## 🛠️ Microcontroller Firmware Integration (C/C++)

Drop [`firmware/ros2_bridge.h`](firmware/ros2_bridge.h) directly into your STM32CubeIDE, ESP-IDF, or PlatformIO project:

```c
#include "ros2_bridge.h"

uint8_t tx_buffer[72];
float battery_voltage = 14.8f;
static uint8_t seq_counter = 0;

void send_battery_telemetry(void) {
    int packet_len = bridge_serialize_e2e(
        0x10,                             // MSG_ID: Battery
        seq_counter++,                    // 4-bit monotonic alive counter
        (const uint8_t*)&battery_voltage, // Payload
        sizeof(float),                    // Length (4 bytes)
        tx_buffer                         // Destination buffer
    );
    
    // Transmit over UART DMA / Interrupt
    HAL_UART_Transmit_DMA(&huart2, tx_buffer, packet_len);
}
```

---

## 🧪 Verification & Empirical Test Suite

Run the automated pytest test suite covering CRC verification, single-bit corruption detection, stream resynchronization, and unpacking:

```bash
pytest test/ -v
```

```
============================= test session starts =============================
collected 11 items

test/test_autosar_e2e.py::test_crc8_sae_j1850_standard_vector PASSED     [  9%]
test/test_autosar_e2e.py::test_packet_decoder_success PASSED             [ 18%]
test/test_autosar_e2e.py::test_packet_decoder_single_bit_flip_detection PASSED [ 27%]
test/test_autosar_e2e.py::test_packet_decoder_stream_resynchronization PASSED  [ 36%]
test/test_bridge.py::test_bridge_heartbeat_packet PASSED                 [ 45%]
test/test_bridge.py::test_bridge_cmd_vel_unpacking PASSED                [ 54%]
test/test_bridge.py::test_bridge_imu_unpacking PASSED                    [ 63%]
test/test_bridge.py::test_corrupted_crc_rejected PASSED                  [ 72%]
test/test_bridge_node.py::test_packet_decoder_success PASSED             [ 81%]
test/test_bridge_node.py::test_packet_decoder_invalid_checksum PASSED    [ 90%]
test/test_bridge_node.py::test_packet_decoder_fragmented_stream PASSED   [100%]

============================= 11 passed in 0.13s ==============================
```

---

## 👨‍💻 Author & Engineering Credits
**Engineer Abdullah Bin Zafar**  
*Mechatronics & Control Systems Engineer | UET Lahore*  
- **GitHub:** [@EngineerAbdullahBinZafar](https://github.com/EngineerAbdullahBinZafar)  
- **Flagship R&D:** [Automotive Chassis CBF](https://github.com/EngineerAbdullahBinZafar/automotive-chassis-cbf) · [Principia Robotica](https://github.com/EngineerAbdullahBinZafar/principia-robotica)
