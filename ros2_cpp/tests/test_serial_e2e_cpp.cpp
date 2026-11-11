/*
 * test_serial_e2e_cpp.cpp
 * C++17 Unit Test Suite for ROS 2 SerialE2ECodec
 * Compiled with -std=c++17 -Wall -Wextra -Werror -fsanitize=address,undefined
 */

#include "ros2_hardware_bridge/serial_e2e_bridge.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

using namespace ros2_hardware_bridge;

static void test_twist_encode_decode_roundtrip()
{
    SerialE2ECodec tx_codec;
    SerialE2ECodec rx_codec;

    TwistCommand cmd{1.25f, -0.45f};
    auto frame_bytes = tx_codec.encode_twist(cmd);

    auto decoded = rx_codec.feed_bytes(frame_bytes.data(), frame_bytes.size());
    assert(decoded.size() == 1U);
    auto recovered = SerialE2ECodec::decode_twist(decoded[0]);
    assert(recovered.has_value());
    assert(std::fabs(recovered->linear_x - 1.25f) < 1e-6f);
    assert(std::fabs(recovered->angular_z - (-0.45f)) < 1e-6f);
    std::cout << "  [PASS] test_twist_encode_decode_roundtrip\n";
}

static void test_imu_burst_noise_rejection()
{
    SerialE2ECodec tx_codec;
    SerialE2ECodec rx_codec;

    Imu6DofSample imu{0.1f, -0.2f, 9.81f, 0.01f, -0.02f, 0.03f};
    auto wire = tx_codec.encode_imu(imu);

    /* Corrupt byte 10 in payload */
    wire[10] ^= 0x04U;
    auto decoded = rx_codec.feed_bytes(wire.data(), wire.size());
    assert(decoded.size() == 1U);
    assert(decoded[0].status == E2E_STATUS_CRC_ERROR);
    assert(!SerialE2ECodec::decode_imu(decoded[0]).has_value());
    assert(rx_codec.diagnostics().frames_crc_err == 1ULL);
    std::cout << "  [PASS] test_imu_burst_noise_rejection\n";
}

int main()
{
    std::cout << "Running ROS 2 Hardware Bridge C++17 Codec Tests...\n";
    test_twist_encode_decode_roundtrip();
    test_imu_burst_noise_rejection();
    std::cout << "ALL C++17 CODEC TESTS PASSED.\n";
    return 0;
}
