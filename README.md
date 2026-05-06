<div align="center">

<img width="100%" src="https://capsule-render.vercel.app/api?type=slice&color=0:22314E,50:00d4ff,100:22314E&height=250&section=header&text=ROS2%20Hardware%20Bridge&fontSize=60&fontColor=ffffff&fontAlignY=40&stroke=ffffff&strokeWidth=1&desc=🌉%20THE%20ZERO-FRICTION%20COMMUNICATION%20LAYER%20BETWEEN%20ROBOTS&descSize=16&descAlignY=60&descColor=00d4ff&animation=fadeIn" />

**The Zero-Friction Communication Layer Between ROS2 and Embedded Microcontrollers**

<img src="https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMDBia2R0OXA5N2YxbmZ4Ynd4Ynd4Ynd4Ynd4Ynd4Ynd4Ynd4JmVwPXYxX2ludGVybmFsX2dpZl9ieV9pZCZjdD1n/6Z9G1a3v0v0v0/giphy.gif" width="100%" />

[![CI Status](https://img.shields.io/github/actions/workflow/status/EngineerAbdullahBinZafar/ros2-hardware-bridge/ci.yml?branch=main&style=for-the-badge&logo=githubactions&logoColor=white)](https://github.com/EngineerAbdullahBinZafar/ros2-hardware-bridge/actions)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)](https://opensource.org/licenses/MIT)
[![ROS2: Humble/Foxy](https://img.shields.io/badge/ROS2-Humble%20/%20Foxy-22314E.svg?style=for-the-badge&logo=ros&logoColor=white)](https://docs.ros.org/en/humble/index.html)
[![MCU: STM32/ESP32](https://img.shields.io/badge/MCU-STM32%20/%20ESP32%20/%20Arduino-03234B.svg?style=for-the-badge&logo=stmicroelectronics&logoColor=white)](#)

*Forget the immense complexity of micro-ROS. Get your STM32 talking to ROS2 in **under 3 minutes**.*

[Features](#-key-features) • [Quick Start](#%EF%B8%8F-quick-start-zero-friction) • [Architecture](#-architecture) • [Documentation](#-supported-message-ids) • [Contributing](#-contributing)

</div>

---

## 🚀 Why Use This?

Every robotics engineer faces the exact same challenge: **"How do I get my microcontroller to talk to my ROS2 PC reliably without going crazy?"**

Common solutions like `micro-ROS` are incredibly powerful but severely over-engineered for simple sensor/actuator bridges. **`ros2-hardware-bridge`** provides the ultimate middle ground:

- ⚡ **Zero Configuration**: No complex build systems, no DDS agents to configure.
- 🛡️ **Type Safety**: Pre-defined packet structures mapped directly to native ROS2 types (Twist, Pose, Battery, Encoders).
- 🧩 **Fault Tolerant**: Built-in CRC checksums silently drop noisy serial garbage.
- 🪶 **Ultra-Lightweight**: The C-firmware library is `< 1KB` and has **zero external dependencies**.

---

## ⚡️ Quick Start (Zero-Friction)

Get up and running in 3 commands.

### 1. ROS2 Host (PC/Raspberry Pi)
Clone and build the package in your ROS2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/EngineerAbdullahBinZafar/ros2-hardware-bridge.git
cd ~/ros2_ws
colcon build --packages-select ros2_hardware_bridge
source install/setup.bash

# Launch the bridge on your serial port
ros2 run ros2_hardware_bridge bridge_node --ros-args -p port:=/dev/ttyACM0
```

### 2. Firmware (STM32 / ESP32 / Arduino)
Simply include the single header file `ros2_bridge.h` and use the serialization functions.

```cpp
#include "ros2_bridge.h"

// Example: Send battery voltage to ROS2
float voltage = 12.6f;
uint8_t tx_buf[32];

// Serialize data (ID 0x10 = Battery Voltage)
int len = bridge_serialize(0x10, (uint8_t*)&voltage, sizeof(float), tx_buf);

// Transmit via UART (STM32 HAL example)
HAL_UART_Transmit(&huart1, tx_buf, len, 10);
```

---

## 📐 Architecture

Our custom packet protocol guarantees that your ROS2 ecosystem never sees corrupted data.

```mermaid
graph LR
    subgraph "Embedded System (STM32 / ESP32)"
        style A fill:#03234B,stroke:#fff,stroke-width:2px,color:#fff
        style B fill:#03234B,stroke:#fff,stroke-width:2px,color:#fff
        A[Firmware Logic] -->|Raw Sensor Data| B[ros2_bridge.h]
        B -->|CRC Packed Serial| C((UART / Serial))
    end
    
    C <==>|USB / Serial Cable| D((Serial / USB))
    
    subgraph "ROS2 Host (PC / Pi / Jetson)"
        style E fill:#22314E,stroke:#fff,stroke-width:2px,color:#fff
        style F fill:#22314E,stroke:#fff,stroke-width:2px,color:#fff
        style G fill:#22314E,stroke:#fff,stroke-width:2px,color:#fff
        style H fill:#22314E,stroke:#fff,stroke-width:2px,color:#fff
        D <==>|CRC Packed Serial| E[bridge_node.py]
        E -->|Publish Twist| F[/cmd_vel/]
        E -->|Publish Float32| G[/hw/battery/]
        E -->|Publish Int32| H[/hw/encoder/]
    end
```

---

## 📦 Supported Message IDs

| Packet ID | Description | ROS2 Topic Name | Standard Data Type |
|:---:|:---|:---|:---|
| `0x01` | Velocity Command | `/cmd_vel` | `geometry_msgs/Twist` |
| `0x10` | Battery Voltage | `/hw/battery` | `std_msgs/Float32` |
| `0x11` | Encoder Ticks | `/hw/encoder` | `std_msgs/Int32` |

*(Need more messages? See [Contributing](#-contributing) to easily add your own!)*

---

## 🤝 Contributing

We want to make this the universal standard for simple ROS2 hardware interfacing. 

1. Read our [Contributing Guidelines](CONTRIBUTING.md) and [Code of Conduct](CODE_OF_CONDUCT.md).
2. Fork the repository.
3. Submit a Pull Request.

**High-Priority Bounties:**
- Adding support for `sensor_msgs/Imu` and `sensor_msgs/LaserScan`.
- High-performance C++ implementation of the ROS2 host node.

---

## 📄 License
Distributed under the MIT License. See `LICENSE` for more information.

## 👤 Author

Developed with passion by **Engineer Abdullah Bin Zafar**.
*If this project saved you hours of debugging `micro-ROS`, consider dropping a ⭐!*

[![LinkedIn](https://img.shields.io/badge/LinkedIn-Connect-0077B5?style=for-the-badge&logo=linkedin)](https://linkedin.com/in/abdullah-bin-zafar)
[![GitHub](https://img.shields.io/badge/GitHub-Follow-181717?style=for-the-badge&logo=github)](https://github.com/EngineerAbdullahBinZafar)
