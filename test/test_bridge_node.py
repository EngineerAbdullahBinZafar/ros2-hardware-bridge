import struct
import sys

# Standalone mocks if ROS2 / pyserial are not installed in global test Python
try:
    import serial
    import rclpy
except ModuleNotFoundError:
    class MockSerial:
        def __init__(self, *args, **kwargs): pass

    sys.modules['serial'] = type(sys)('serial')
    sys.modules['serial'].Serial = MockSerial
    sys.modules['serial.tools'] = type(sys)('serial.tools')
    sys.modules['serial.tools.list_ports'] = type(sys)('serial.tools.list_ports')
    sys.modules['serial.tools.list_ports'].comports = lambda: []

    class MockNode:
        def get_logger(self):
            class Logger:
                def info(self, m): pass
                def debug(self, m): pass
                def warn(self, m): pass
                def error(self, m): pass
            return Logger()
        def declare_parameter(self, n, v): pass
        def get_parameter(self, n):
            class Param:
                value = 'AUTO'
            return Param()
        def create_publisher(self, c, t, q): pass
        def create_subscription(self, c, t, cb, q): pass
        def create_timer(self, i, cb): pass

    sys.modules['rclpy'] = type(sys)('rclpy')
    sys.modules['rclpy.node'] = type(sys)('rclpy.node')
    sys.modules['rclpy.node'].Node = MockNode
    sys.modules['std_msgs.msg'] = type(sys)('std_msgs.msg')
    sys.modules['std_msgs.msg'].Float32 = float
    sys.modules['std_msgs.msg'].Int32 = int
    sys.modules['std_msgs.msg'].String = str
    sys.modules['geometry_msgs.msg'] = type(sys)('geometry_msgs.msg')
    sys.modules['geometry_msgs.msg'].Twist = object
    sys.modules['sensor_msgs.msg'] = type(sys)('sensor_msgs.msg')
    sys.modules['sensor_msgs.msg'].Imu = object

from ros2_bridge.bridge_node import PacketDecoder

def test_packet_decoder_success():
    decoder = PacketDecoder()
    
    payload = struct.pack('<f', 12.5)
    msg_id = 0x10
    length = len(payload)
    checksum = (msg_id + length + sum(payload)) & 0xFF
    
    packet = b'\xAA\x55' + bytes([msg_id, length]) + payload + bytes([checksum])
    
    result = None
    for b in packet:
        res = decoder.parse_byte(b)
        if res:
            result = res
            
    assert result is not None
    parsed_id, parsed_payload = result
    assert parsed_id == 0x10
    parsed_val = struct.unpack('<f', parsed_payload)[0]
    assert abs(parsed_val - 12.5) < 1e-5

def test_packet_decoder_invalid_checksum():
    decoder = PacketDecoder()
    
    packet = b'\xAA\x55\x10\x04\x00\x00\x48\x41\x00'
    
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
    length = len(payload)
    checksum = (msg_id + length + sum(payload)) & 0xFF
    packet = b'\xAA\x55' + bytes([msg_id, length]) + payload + bytes([checksum])
    
    parsed = None
    for byte in packet:
        res = decoder.parse_byte(byte)
        if res:
            parsed = res
            
    assert parsed is not None
    assert parsed[0] == 0x11
    val = struct.unpack('<i', parsed[1])[0]
    assert val == 4096
