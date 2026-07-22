import struct
import sys
import rclpy
from rclpy.node import Node
import serial
import serial.tools.list_ports
from std_msgs.msg import Float32, Int32, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

class PacketDecoder:
    """
    State machine frame decoder for COBS/framed serial packets.
    Prevents partial/fragmented reads from breaking packet synchronization.
    Frame format: HEADER (0xAA 0x55) | MSG_ID (1B) | LENGTH (1B) | PAYLOAD (NB) | CHECKSUM (1B)
    """
    STATE_HEADER_1 = 0
    STATE_HEADER_2 = 1
    STATE_MSG_ID = 2
    STATE_LENGTH = 3
    STATE_PAYLOAD = 4
    STATE_CHECKSUM = 5

    def __init__(self):
        self.state = self.STATE_HEADER_1
        self.msg_id = 0
        self.length = 0
        self.payload = bytearray()
        self.checksum = 0

    def parse_byte(self, b: int):
        if self.state == self.STATE_HEADER_1:
            if b == 0xAA:
                self.state = self.STATE_HEADER_2
        elif self.state == self.STATE_HEADER_2:
            if b == 0x55:
                self.state = self.STATE_MSG_ID
            else:
                self.state = self.STATE_HEADER_1
        elif self.state == self.STATE_MSG_ID:
            self.msg_id = b
            self.state = self.STATE_LENGTH
        elif self.state == self.STATE_LENGTH:
            self.length = b
            self.payload = bytearray()
            if self.length == 0:
                self.state = self.STATE_CHECKSUM
            else:
                self.state = self.STATE_PAYLOAD
        elif self.state == self.STATE_PAYLOAD:
            self.payload.append(b)
            if len(self.payload) == self.length:
                self.state = self.STATE_CHECKSUM
        elif self.state == self.STATE_CHECKSUM:
            self.checksum = b
            calc_checksum = (self.msg_id + self.length + sum(self.payload)) & 0xFF
            self.state = self.STATE_HEADER_1
            if calc_checksum == self.checksum:
                return (self.msg_id, bytes(self.payload))
        return None

class ROS2HardwareBridge(Node):
    """
    ROS2 Hardware Bridge Node.
    Provides robust, cross-platform serial communication between ROS2 system and microcontrollers (STM32, ESP32, Arduino).
    """
    def __init__(self):
        super().__init__('hardware_bridge')

        # Parameters
        self.declare_parameter('port', 'AUTO')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('reconnect_interval', 2.0)

        self.port_setting = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.reconnect_interval = self.get_parameter('reconnect_interval').value

        self.ser = None
        self.decoder = PacketDecoder()

        # Telemetry Publishers (Hardware -> ROS2)
        self.batt_pub = self.create_publisher(Float32, 'hw/battery', 10)
        self.enc_pub = self.create_publisher(Int32, 'hw/encoder', 10)
        self.imu_pub = self.create_publisher(Imu, 'hw/imu', 10)
        self.status_pub = self.create_publisher(String, 'hw/status', 10)

        # Control Subscribers (ROS2 -> Hardware)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)

        # Timers
        self.timer_serial_read = self.create_timer(0.005, self.read_serial)  # 200Hz polling
        self.timer_heartbeat = self.create_timer(1.0, self.send_heartbeat)

        self._connect_serial()

    def _detect_port(self):
        """Cross-platform serial port detection."""
        ports = serial.tools.list_ports.comports()
        if not ports:
            return None
        # Priority for USB-Serial adapters
        for p in ports:
            if "USB" in p.description or "ACM" in p.device or "COM" in p.device:
                return p.device
        return ports[0].device

    def _connect_serial(self):
        """Attempts to establish connection to serial port."""
        target_port = self.port_setting
        if target_port == 'AUTO':
            target_port = self._detect_port()

        if not target_port:
            self.get_logger().warn("No valid serial ports detected. Retrying...")
            return False

        try:
            self.ser = serial.Serial(target_port, self.baudrate, timeout=0.01)
            self.get_logger().info(f"Connected to hardware on [{target_port}] @ {self.baudrate} baud")
            self.status_pub.publish(String(data=f"CONNECTED:{target_port}"))
            return True
        except Exception as e:
            self.get_logger().error(f"Serial connection failed on [{target_port}]: {e}")
            self.ser = None
            return False

    def cmd_callback(self, msg: Twist):
        """Packs motor velocity command (linear.x, angular.z) into binary frame."""
        payload = struct.pack('<ff', msg.linear.x, msg.angular.z)
        self.send_packet(0x01, payload)

    def send_heartbeat(self):
        """Sends periodic system heartbeat (0x00) to keep MCU watchdog active."""
        if self.ser and self.ser.is_open:
            self.send_packet(0x00, b'\x01')
        else:
            self._connect_serial()

    def send_packet(self, msg_id: int, payload: bytes):
        """Constructs and transmits framed binary packet to serial bus."""
        if not self.ser or not self.ser.is_open:
            return

        start = b'\xAA\x55'
        length = len(payload)
        checksum = (msg_id + length + sum(payload)) & 0xFF
        packet = start + struct.pack('BB', msg_id, length) + payload + struct.pack('B', checksum)

        try:
            self.ser.write(packet)
        except Exception as e:
            self.get_logger().error(f"Write error on serial port: {e}")
            self.ser = None

    def read_serial(self):
        """Reads raw bytes and processes through packet decoder state machine."""
        if not self.ser or not self.ser.is_open:
            return

        try:
            bytes_available = self.ser.in_waiting
            if bytes_available > 0:
                raw_bytes = self.ser.read(bytes_available)
                for b in raw_bytes:
                    res = self.decoder.parse_byte(b)
                    if res:
                        msg_id, payload = res
                        self.process_packet(msg_id, payload)
        except Exception as e:
            self.get_logger().error(f"Serial communication lost: {e}")
            self.ser = None

    def process_packet(self, msg_id: int, payload: bytes):
        """Dispatches decoded payload to corresponding ROS2 topic."""
        try:
            if msg_id == 0x10 and len(payload) >= 4:  # Battery Voltage (Float32)
                val = struct.unpack('<f', payload[:4])[0]
                self.batt_pub.publish(Float32(data=val))

            elif msg_id == 0x11 and len(payload) >= 4:  # Encoder Ticks (Int32)
                val = struct.unpack('<i', payload[:4])[0]
                self.enc_pub.publish(Int32(data=val))

            elif msg_id == 0x12 and len(payload) >= 24:  # IMU Raw (6x Float32)
                ax, ay, az, gx, gy, gz = struct.unpack('<ffffff', payload[:24])
                imu_msg = Imu()
                imu_msg.linear_acceleration.x = ax
                imu_msg.linear_acceleration.y = ay
                imu_msg.linear_acceleration.z = az
                imu_msg.angular_velocity.x = gx
                imu_msg.angular_velocity.y = gy
                imu_msg.angular_velocity.z = gz
                self.imu_pub.publish(imu_msg)

            elif msg_id == 0x1F:  # Hardware Status String
                status_str = payload.decode('utf-8', errors='ignore')
                self.status_pub.publish(String(data=status_str))

        except Exception as e:
            self.get_logger().warn(f"Failed to process packet ID [0x{msg_id:02X}]: {e}")

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
