"""
test_autosar_e2e.py - Rigorous Unit & Fault Injection Test Suite for AUTOSAR E2E Profile 1
Author: Engineer Abdullah Bin Zafar
"""

import struct
import pytest
from ros2_bridge.bridge_node import (
    PacketDecoder,
    cobs_decode,
    cobs_encode,
    compute_autosar_crc8,
    encode_e2e_frame,
)

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


def test_data_id_authentication_and_mismatch_rejection():
    """Verify 16-bit Data ID (low-byte, high-byte) authentication in AUTOSAR E2E Profile 1 CRC-8."""
    payload = b"\x11\x22\x33\x44"
    wire = encode_e2e_frame(0x01, 0x02, payload, data_id=0x1A2B)

    dec_ok = PacketDecoder(data_id=0x1A2B)
    res_ok = None
    for b in wire:
        r = dec_ok.parse_byte(b)
        if r is not None:
            res_ok = r
    assert res_ok == (0x01, 0x02, payload)
    assert dec_ok.last_status == "INITIAL"

    # Wrong Data ID must fail CRC-8
    dec_bad = PacketDecoder(data_id=0x9999)
    res_bad = None
    for b in wire:
        r = dec_bad.parse_byte(b)
        if r is not None:
            res_bad = r
    assert res_bad is None
    assert dec_bad.last_status == "CRC_ERROR"


def test_max_delta_counter_sequence_enforcement():
    """Verify repeated, in-window loss, and desync (> max_delta_counter) outcomes."""
    dec = PacketDecoder(max_delta_counter=2)
    p = b"\xDE\xAD\xBE\xEF"

    def feed(seq):
        out = None
        for b in encode_e2e_frame(0x10, seq, p):
            r = dec.parse_byte(b)
            if r is not None:
                out = r
        return out

    assert feed(0) == (0x10, 0, p)
    assert dec.last_status == "INITIAL"
    assert feed(1) == (0x10, 1, p)
    assert dec.last_status == "OK"
    assert feed(1) is None
    assert dec.last_status == "REPEATED"
    assert feed(3) == (0x10, 3, p)  # lost 1 (<= max_delta_counter=2)
    assert dec.last_status == "WRONG_SEQUENCE"
    assert dec.lost_frames == 1
    assert feed(7) is None           # lost 3 (> max_delta_counter=2 => DESYNC)
    assert dec.last_status == "DESYNC"
    assert dec.lost_frames == 3
    assert feed(8) == (0x10, 8, p)  # consecutive after desync recovers
    assert dec.last_status == "OK"


def test_cobs_e2e_roundtrip_and_zero_free_wire():
    """Verify COBS framing eliminates 0x00 bytes on wire and integrates with PacketDecoder."""
    raw = b"\x00\xAA\x55\x00\x12\x00\x00\x34"
    enc = cobs_encode(raw)
    assert 0x00 not in enc
    assert cobs_decode(enc) == raw

    payload = b"\x00\x01\x00\x02\x00\x03\x00\x04"
    wire = encode_e2e_frame(0x01, 0x00, payload, data_id=0x0110, use_cobs=True)
    assert wire[-1] == 0x00
    assert 0x00 not in wire[:-1]

    dec = PacketDecoder(data_id=0x0110)
    res = None
    for b in wire:
        r = dec.parse_cobs_byte(b)
        if r is not None:
            res = r
    assert res == (0x01, 0x00, payload)

    # Malformed COBS (truncated) must reject cleanly
    dec_err = PacketDecoder(data_id=0x0110)
    assert dec_err.parse_cobs_byte(0x05) is None
    assert dec_err.parse_cobs_byte(0x11) is None
    assert dec_err.parse_cobs_byte(0x00) is None
    assert dec_err.last_status == "FRAME_ERROR"
