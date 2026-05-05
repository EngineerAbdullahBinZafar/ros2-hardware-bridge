import rclpy
from rclpy.node import Node
import serial
import struct
from std_msgs.msg import Float32, Int32, String
from geometry_msgs.msg import Twist

class ROS2HardwareBridge(Node):
    def __init__(self):
        super().__init__('hardware_bridge')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to {port} at {baud} baud")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to serial: {e}")
            return

        # Publishers (Data from Hardware -> ROS2)
        self.batt_pub = self.create_publisher(Float32, 'hw/battery', 10)
        self.enc_pub = self.create_publisher(Int32, 'hw/encoder', 10)
        
        # Subscribers (Data from ROS2 -> Hardware)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        
        # Timer for reading serial
        self.timer = self.create_timer(0.01, self.read_serial) # 100Hz

    def cmd_callback(self, msg):
        # Pack linear.x and angular.z into bytes (2 floats = 8 bytes)
        payload = struct.pack('ff', msg.linear.x, msg.angular.z)
        self.send_packet(0x01, payload)

    def send_packet(self, msg_id, payload):
        start = b'\xAA\x55'
        length = len(payload)
        checksum = (msg_id + length + sum(payload)) & 0xFF
        packet = start + struct.pack('BB', msg_id, length) + payload + struct.pack('B', checksum)
        self.ser.write(packet)

    def read_serial(self):
        if self.ser.in_waiting >= 5:
            header = self.ser.read(2)
            if header == b'\xAA\x55':
                msg_id = ord(self.ser.read(1))
                length = ord(self.ser.read(1))
                payload = self.ser.read(length)
                checksum = ord(self.ser.read(1))
                
                # Verify Checksum
                if (msg_id + length + sum(payload)) & 0xFF == checksum:
                    self.process_packet(msg_id, payload)

    def process_packet(self, msg_id, payload):
        if msg_id == 0x10: # Battery Voltage
            val = struct.unpack('f', payload)[0]
            self.batt_pub.publish(Float32(data=val))
        elif msg_id == 0x11: # Encoder Ticks
            val = struct.unpack('i', payload)[0]
            self.enc_pub.publish(Int32(data=val))

def main(args=None):
    rclpy.init(args=args)
    node = ROS2HardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
