"""
test_autosar_e2e.py - Rigorous Unit & Fault Injection Test Suite for AUTOSAR E2E Profile 1
Author: Engineer Abdullah Bin Zafar
"""

import struct
import pytest
from ros2_bridge.bridge_node import PacketDecoder, compute_autosar_crc8, CRC8_SAE_J1850_TABLE

def test_crc8_sae_j1850_standard_vector():
    """Verify known test vector for SAE J1850 polynomial 0x1D, init 0xFF, xorout 0xFF."""
    data = bytes([0x01, 0x02, 0x03])
    crc = compute_autosar_crc8(data)
    assert crc == 0xC1, f"Expected 0xC1, got {hex(crc)}"

def test_packet_decoder_success():
    """Verify decoding of a valid AUTOSAR E2E frame."""
    decoder = PacketDecoder()
    msg_id = 0x10  # Battery telemetry
    seq_ctr = 0x03
    payload = struct.pack('<f', 14.8)  # 14.8 Volts
    length = len(payload)

    header_payload = bytes([msg_id, seq_ctr, length]) + payload
    crc = compute_autosar_crc8(header_payload)

    frame = b'\xAA\x55' + header_payload + bytes([crc])

    result = None
    for b in frame:
        res = decoder.parse_byte(b)
        if res:
            result = res

    assert result is not None
    parsed_id, parsed_seq, parsed_payload = result
    assert parsed_id == 0x10
    assert parsed_seq == 0x03
    val = struct.unpack('<f', parsed_payload)[0]
    assert abs(val - 14.8) < 1e-4

def test_packet_decoder_single_bit_flip_detection():
    """Verify that every single 1-bit corruption across the frame is rejected (Hamming Distance >= 4)."""
    decoder = PacketDecoder()
    msg_id = 0x11
    seq_ctr = 0x0A
    payload = struct.pack('<i', 65536)
    length = len(payload)

    header_payload = bytes([msg_id, seq_ctr, length]) + payload
    crc = compute_autosar_crc8(header_payload)
    frame = bytearray(b'\xAA\x55' + header_payload + bytes([crc]))

    # Inject bit flips across all payload and header bytes
    for byte_idx in range(2, len(frame)):
        for bit_idx in range(8):
            corrupted = bytearray(frame)
            corrupted[byte_idx] ^= (1 << bit_idx)
            
            dec = PacketDecoder()
            result = None
            for b in corrupted:
                r = dec.parse_byte(b)
                if r:
                    result = r
            assert result is None, f"Undetected bit corruption at byte {byte_idx}, bit {bit_idx}!"

def test_packet_decoder_stream_resynchronization():
    """Verify state machine gracefully recovers from garbage bytes before valid frame."""
    decoder = PacketDecoder()
    garbage = b'\x00\xFF\xAA\x12\x34\x56\xAA\x54\xAA'
    
    msg_id = 0x12
    seq_ctr = 0x05
    payload = b'STABLE_TELEMETRY'
    header_payload = bytes([msg_id, seq_ctr, len(payload)]) + payload
    crc = compute_autosar_crc8(header_payload)
    valid_frame = b'\xAA\x55' + header_payload + bytes([crc])

    stream = garbage + valid_frame
    result = None
    for b in stream:
        r = decoder.parse_byte(b)
        if r:
            result = r

    assert result is not None
    assert result[0] == 0x12
    assert result[1] == 0x05
    assert result[2] == payload
