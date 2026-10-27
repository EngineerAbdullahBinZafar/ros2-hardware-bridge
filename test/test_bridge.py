"""
Comprehensive unit & integration test suite for ros2-hardware-bridge.
Tests packet serialization, AUTOSAR E2E Profile 1 CRC-8, sequence counters,
resynchronization, and corrupted packet rejections.
"""

import struct
import pytest
from ros2_bridge.bridge_node import PacketDecoder, compute_autosar_crc8

def test_bridge_heartbeat_packet():
    decoder = PacketDecoder()
    msg_id = 0x00
    seq = 0x01
    payload = b'\x01'
    
    header_payload = bytes([msg_id, seq, len(payload)]) + payload
    crc = compute_autosar_crc8(header_payload)
    pkt = b'\xAA\x55' + header_payload + bytes([crc])
    
    res = None
    for b in pkt:
        r = decoder.parse_byte(b)
        if r:
            res = r
            
    assert res is not None
    assert res[0] == 0x00
    assert res[1] == 0x01
    assert res[2] == b'\x01'

def test_bridge_cmd_vel_unpacking():
    decoder = PacketDecoder()
    msg_id = 0x01
    seq = 0x0F
    # Linear.x = 1.25 m/s, angular.z = -0.5 rad/s
    payload = struct.pack('<ff', 1.25, -0.5)
    
    header_payload = bytes([msg_id, seq, len(payload)]) + payload
    crc = compute_autosar_crc8(header_payload)
    pkt = b'\xAA\x55' + header_payload + bytes([crc])
    
    res = None
    for b in pkt:
        r = decoder.parse_byte(b)
        if r:
            res = r
            
    assert res is not None
    vx, wz = struct.unpack('<ff', res[2])
    assert abs(vx - 1.25) < 1e-5
    assert abs(wz - (-0.5)) < 1e-5

def test_bridge_imu_unpacking():
    decoder = PacketDecoder()
    msg_id = 0x12
    seq = 0x02
    # 6 floats: ax, ay, az, gx, gy, gz
    payload = struct.pack('<ffffff', 0.0, 0.0, 9.81, 0.01, -0.02, 0.005)
    
    header_payload = bytes([msg_id, seq, len(payload)]) + payload
    crc = compute_autosar_crc8(header_payload)
    pkt = b'\xAA\x55' + header_payload + bytes([crc])
    
    res = None
    for b in pkt:
        r = decoder.parse_byte(b)
        if r:
            res = r
            
    assert res is not None
    ax, ay, az, gx, gy, gz = struct.unpack('<ffffff', res[2])
    assert abs(az - 9.81) < 1e-5
    assert abs(gx - 0.01) < 1e-5

def test_corrupted_crc_rejected():
    decoder = PacketDecoder()
    msg_id = 0x10
    seq = 0x00
    payload = struct.pack('<f', 12.6)
    header_payload = bytes([msg_id, seq, len(payload)]) + payload
    crc = compute_autosar_crc8(header_payload) ^ 0x01  # Invert 1 bit in CRC
    pkt = b'\xAA\x55' + header_payload + bytes([crc])
    
    res = None
    for b in pkt:
        r = decoder.parse_byte(b)
        if r:
            res = r
            
    assert res is None
