/*
 * serial_e2e_bridge.hpp
 * Modern C++17 Deterministic Serialization & Frame Codec for ROS 2 Hosts
 *
 * Wraps bare-metal AUTOSAR E2E Profile 1 C primitives in type-safe C++17 RAII
 * structures with zero runtime heap allocation on hot paths.
 *
 * Author: Engineer Abdullah Bin Zafar
 * License: Apache-2.0
 */

#ifndef ROS2_HARDWARE_BRIDGE_SERIAL_E2E_BRIDGE_HPP_
#define ROS2_HARDWARE_BRIDGE_SERIAL_E2E_BRIDGE_HPP_

#include <array>
#include <cstdint>
#include <cstring>
#include <optional>
#include <vector>

extern "C" {
#include "../../firmware/ros2_bridge.h"
#include "../../firmware/include/ros2_bridge_fsm.h"
}

namespace ros2_hardware_bridge {

struct TwistCommand {
    float linear_x;
    float angular_z;
};

struct Imu6DofSample {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
};

class SerialE2ECodec {
public:
    explicit SerialE2ECodec(uint8_t max_delta_counter = 2U)
        : tx_seq_(0U)
    {
        bridge_parser_init(&parser_ctx_, max_delta_counter);
    }

    /**
     * @brief Serialize an 8-byte Twist command into an AUTOSAR E2E Profile 1 frame.
     */
    std::array<uint8_t, 14> encode_twist(const TwistCommand& cmd) noexcept
    {
        std::array<uint8_t, 8> payload{};
        std::memcpy(payload.data(), &cmd.linear_x, sizeof(float));
        std::memcpy(payload.data() + sizeof(float), &cmd.angular_z, sizeof(float));

        std::array<uint8_t, 14> wire_frame{};
        bridge_serialize_e2e(0x01U, tx_seq_, payload.data(), 8U, wire_frame.data());
        tx_seq_ = static_cast<uint8_t>((tx_seq_ + 1U) & 0x0FU);
        return wire_frame;
    }

    /**
     * @brief Serialize a 24-byte 6-DOF IMU packet for HIL injection / loopback verification.
     */
    std::array<uint8_t, 30> encode_imu(const Imu6DofSample& sample) noexcept
    {
        std::array<uint8_t, 24> payload{};
        std::memcpy(payload.data() + 0U, &sample.ax, sizeof(float));
        std::memcpy(payload.data() + 4U, &sample.ay, sizeof(float));
        std::memcpy(payload.data() + 8U, &sample.az, sizeof(float));
        std::memcpy(payload.data() + 12U, &sample.gx, sizeof(float));
        std::memcpy(payload.data() + 16U, &sample.gy, sizeof(float));
        std::memcpy(payload.data() + 20U, &sample.gz, sizeof(float));

        std::array<uint8_t, 30> wire_frame{};
        bridge_serialize_e2e(0x12U, tx_seq_, payload.data(), 24U, wire_frame.data());
        tx_seq_ = static_cast<uint8_t>((tx_seq_ + 1U) & 0x0FU);
        return wire_frame;
    }

    /**
     * @brief Feed an incoming byte span from UART/USB-CDC into the FSM parser.
     */
    std::vector<bridge_decoded_frame_t> feed_bytes(const uint8_t* data, std::size_t length)
    {
        std::vector<bridge_decoded_frame_t> results;
        for (std::size_t i = 0U; i < length; ++i) {
            bridge_decoded_frame_t frame{};
            if (bridge_parser_feed_byte(&parser_ctx_, data[i], &frame)) {
                results.push_back(frame);
            }
        }
        return results;
    }

    static std::optional<TwistCommand> decode_twist(const bridge_decoded_frame_t& frame) noexcept
    {
        if (frame.msg_id != 0x01U || frame.len != 8U ||
            (frame.status != E2E_STATUS_OK && frame.status != E2E_STATUS_INITIAL)) {
            return std::nullopt;
        }
        TwistCommand cmd{};
        std::memcpy(&cmd.linear_x, frame.payload, sizeof(float));
        std::memcpy(&cmd.angular_z, frame.payload + sizeof(float), sizeof(float));
        return cmd;
    }

    static std::optional<Imu6DofSample> decode_imu(const bridge_decoded_frame_t& frame) noexcept
    {
        if (frame.msg_id != 0x12U || frame.len != 24U ||
            (frame.status != E2E_STATUS_OK && frame.status != E2E_STATUS_INITIAL)) {
            return std::nullopt;
        }
        Imu6DofSample imu{};
        std::memcpy(&imu.ax, frame.payload + 0U, sizeof(float));
        std::memcpy(&imu.ay, frame.payload + 4U, sizeof(float));
        std::memcpy(&imu.az, frame.payload + 8U, sizeof(float));
        std::memcpy(&imu.gx, frame.payload + 12U, sizeof(float));
        std::memcpy(&imu.gy, frame.payload + 16U, sizeof(float));
        std::memcpy(&imu.gz, frame.payload + 20U, sizeof(float));
        return imu;
    }

    const bridge_diag_counters_t& diagnostics() const noexcept { return parser_ctx_.diag; }

private:
    bridge_parser_ctx_t parser_ctx_{};
    uint8_t tx_seq_;
};

}  // namespace ros2_hardware_bridge

#endif  // ROS2_HARDWARE_BRIDGE_SERIAL_E2E_BRIDGE_HPP_
