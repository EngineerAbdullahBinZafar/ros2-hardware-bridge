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

static void test_cobs_twist_and_imu_roundtrip()
{
    SerialE2ECodec tx_codec(2U, static_cast<uint16_t>(0x0110U));
    SerialE2ECodec rx_codec(2U, static_cast<uint16_t>(0x0110U));
    SerialE2ECodec rx_mismatch(2U, static_cast<uint16_t>(0x0120U));

    TwistCommand cmd{0.0f, -1.25f};
    auto cobs_twist = tx_codec.encode_twist_cobs(cmd);
    assert(cobs_twist.back() == 0x00U);
    for (std::size_t i = 0U; i + 1U < cobs_twist.size(); ++i) {
        assert(cobs_twist[i] != 0x00U);
    }

    auto decoded = rx_codec.feed_cobs_bytes(cobs_twist.data(), cobs_twist.size());
    assert(decoded.size() == 1U);
    auto recovered = SerialE2ECodec::decode_twist(decoded[0]);
    assert(recovered.has_value());
    assert(std::fabs(recovered->linear_x - 0.0f) < 1e-6f);
    assert(std::fabs(recovered->angular_z - (-1.25f)) < 1e-6f);

    /* Data ID mismatch must fail CRC-8 */
    auto bad = rx_mismatch.feed_cobs_bytes(cobs_twist.data(), cobs_twist.size());
    assert(bad.size() == 1U);
    assert(bad[0].status == E2E_STATUS_CRC_ERROR);
    assert(!SerialE2ECodec::decode_twist(bad[0]).has_value());

    /* IMU COBS roundtrip + DESYNC enforcement */
    Imu6DofSample imu{0.0f, 0.0f, 9.81f, 0.0f, -0.5f, 0.25f};
    auto cobs_imu = tx_codec.encode_imu_cobs(imu);
    auto dec_imu = rx_codec.feed_cobs_bytes(cobs_imu.data(), cobs_imu.size());
    assert(dec_imu.size() == 1U);
    assert(dec_imu[0].status == E2E_STATUS_OK);
    auto rec_imu = SerialE2ECodec::decode_imu(dec_imu[0]);
    assert(rec_imu.has_value());
    assert(std::fabs(rec_imu->az - 9.81f) < 1e-5f);

    /* Skip 5 frames on tx_codec -> exceeds max_delta_counter=2 -> E2E_STATUS_DESYNC */
    for (int i = 0; i < 5; ++i) {
        (void)tx_codec.encode_twist(cmd);
    }
    auto jumped = tx_codec.encode_twist_cobs(cmd);
    auto dec_jump = rx_codec.feed_cobs_bytes(jumped.data(), jumped.size());
    assert(dec_jump.size() == 1U);
    assert(dec_jump[0].status == E2E_STATUS_DESYNC);
    assert(!SerialE2ECodec::decode_twist(dec_jump[0]).has_value());
    std::cout << "  [PASS] test_cobs_twist_and_imu_roundtrip\n";
}

int main()
{
    std::cout << "Running ROS 2 Hardware Bridge C++17 Codec Tests...\n";
    test_twist_encode_decode_roundtrip();
    test_imu_burst_noise_rejection();
    test_cobs_twist_and_imu_roundtrip();
    std::cout << "ALL C++17 CODEC TESTS PASSED.\n";
    return 0;
}
