import struct
import pytest
from ros2_bridge.bridge_node import PacketDecoder, compute_autosar_crc8

def test_packet_decoder_success():
    decoder = PacketDecoder()
    
    payload = struct.pack('<f', 12.5)
    msg_id = 0x10
    seq = 0x01
    length = len(payload)
    
    header_payload = bytes([msg_id, seq, length]) + payload
    crc = compute_autosar_crc8(header_payload)
    packet = b'\xAA\x55' + header_payload + bytes([crc])
    
    result = None
    for b in packet:
        res = decoder.parse_byte(b)
        if res:
            result = res
            
    assert result is not None
    parsed_id, parsed_seq, parsed_payload = result
    assert parsed_id == 0x10
    assert parsed_seq == 0x01
    parsed_val = struct.unpack('<f', parsed_payload)[0]
    assert abs(parsed_val - 12.5) < 1e-5

def test_packet_decoder_invalid_checksum():
    decoder = PacketDecoder()
    # Invalid CRC byte at the end
    packet = b'\xAA\x55\x10\x01\x04\x00\x00\x48\x41\x00'
    
    result = None
    for b in packet:
        res = decoder.parse_byte(b)
        if res:
            result = res
            
    assert result is None

def test_packet_decoder_fragmented_stream():
    decoder = PacketDecoder()
    
    payload = struct.pack('<i', 4096)
    msg_id = 0x11
    seq = 0x05
    length = len(payload)
    
    header_payload = bytes([msg_id, seq, length]) + payload
    crc = compute_autosar_crc8(header_payload)
    packet = b'\xAA\x55' + header_payload + bytes([crc])
    
    parsed = None
    for byte in packet:
        res = decoder.parse_byte(byte)
        if res:
            parsed = res
            
    assert parsed is not None
    assert parsed[0] == 0x11
    assert parsed[1] == 0x05
    val = struct.unpack('<i', parsed[2])[0]
    assert val == 4096
