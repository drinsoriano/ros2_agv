import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

class DDSM115Driver(Node):
    def __init__(self):
        super().__init__('ddsm115_driver')

        # Set up the serial connection
        self.serial_port = '/dev/ttyUSB0'
        self.baudrate = 115200
        self.motor_ids = [1, 2, 3, 4]

        try:
            self.serial = serial.Serial(self.serial_port, self.baudrate, timeout=1)
            self.get_logger().info(f"Initialized serial port {self.serial_port} at baudrate {self.baudrate}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise e

        # ROS2 Subscriber for motor commands
        self.subscription = self.create_subscription(
            String,
            'motor_cmd',
            self.motor_callback,
            10)

        # ROS2 Publisher for motor speed (for LED control)
        self.speed_publisher = self.create_publisher(Int32, 'motor_speed', 10)

    def motor_callback(self, msg):
        data = msg.data.strip()

        # Check if the command is "brake"
        if data == "brake":
            self.send_brake_command()
            self.get_logger().info("Brake command received. Stopping all motors.")
            return

        # Otherwise, parse speed values
        try:
            left_speed, right_speed = map(int, data.split())

            # Send motor commands
            self.send_velocity_command(1, left_speed)  # Left Front
            self.send_velocity_command(2, -right_speed)  # Right Front
            self.send_velocity_command(3, left_speed)  # Left Rear
            self.send_velocity_command(4, -right_speed)  # Right Rear

            self.get_logger().info(f"Received: Left={left_speed} RPM, Right={right_speed} RPM")
        except ValueError:
            self.get_logger().error(f"Invalid command format: {data}")


    def calculate_crc8_maxim(self, data):
        crc = 0x00
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x01:
                    crc = (crc >> 1) ^ 0x8C
                else:
                    crc >>= 1
                crc &= 0xFF
        return crc

    def send_velocity_command(self, motor_id, speed_rpm):
        speed_rpm = max(-330, min(330, speed_rpm))
        if speed_rpm < 0:
            speed_rpm = (1 << 16) + speed_rpm

        command = [
            motor_id,
            0x64,
            (speed_rpm >> 8) & 0xFF,
            speed_rpm & 0xFF,
            0x00, 0x00,
            0x00, 0x00, 0x00
        ]

        crc = self.calculate_crc8_maxim(command)
        command.append(crc)
        self._send_command(command, motor_id, speed_rpm)

    def send_brake_command(self):
        for motor_id in self.motor_ids:
            command = [
                motor_id,
                0x64,
                0x00, 0x00,
                0x00, 0x00,
                0x00, 0xFF, 0x00
            ]

            crc = self.calculate_crc8_maxim(command)
            command.append(crc)
            self._send_command(command, motor_id, "Brake")

    def _send_command(self, command, motor_id, speed_or_action):
        try:
            self.serial.write(bytes(command))
            hex_command = ' '.join(f'{byte:02X}' for byte in command)
            self.get_logger().info(f"Sent: {hex_command} (Motor ID: {motor_id}, Action: {speed_or_action})")
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")

    def stop(self):
        self.serial.close()
        self.get_logger().info("Closed serial port.")

def main(args=None):
    rclpy.init(args=args)
    node = DDSM115Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
