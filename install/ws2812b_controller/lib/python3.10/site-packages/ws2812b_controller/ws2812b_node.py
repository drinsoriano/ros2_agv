import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Int32

class WS2812BController(Node):
    def __init__(self):
        super().__init__('ws2812b_controller')

        # Initialize Serial Communication with Arduino
        self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        self.get_logger().info("WS2812B Controller Node Started")

        # Subscribe to motor speed topic
        self.subscription = self.create_subscription(
            Int32,  # Listening for motor speed as Int32
            'motor_speed',
            self.speed_callback,
            10)

    def speed_callback(self, msg):
        speed = msg.data  # Get speed value
        command = f"SPEED:{speed}\n"  # Example command format
        self.serial_port.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent to Arduino: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = WS2812BController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
