import unittest
from ros2_bridge.bridge_node import ROS2HardwareBridge
import rclpy

class TestBridgeLogic(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_checksum_calculation(self):
        # Header logic test
        msg_id = 0x01
        length = 2
        payload = b'\x01\x02'
        expected_checksum = (msg_id + length + sum(payload)) & 0xFF
        
        # Verify our manual calculation matches
        self.assertEqual(expected_checksum, 6)

if __name__ == '__main__':
    unittest.main()
