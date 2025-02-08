import rclpy
from rclpy.node import Node
import termios
import sys
import tty
import time
from std_msgs.msg import String

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')

        # ROS2 Publisher for motor commands
        self.publisher_ = self.create_publisher(String, 'motor_cmd', 10)
        
        # Motion control variables
        self.forward_speed = 0
        self.turn_speed = 0
        self.speed_increment = 10
        self.turn_increment = 5  # Smoother turning
        self.auto_decay = 2  # Speed decay per loop (natural slowing effect)

        self.get_logger().info(
            "Keyboard Control Node initialized.\n"
            "Use W/A/S/D to control, Space to brake, and Q to quit."
        )

    def get_key(self):
        """ Reads a single key press from the keyboard. """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def send_motor_commands(self):
        """ Publishes motor speed commands to /motor_cmd topic. """
        # Calculate left and right motor speeds
        left_speed = self.forward_speed + self.turn_speed
        right_speed = self.forward_speed - self.turn_speed

        # Clamp speeds (-330 to 330 RPM)
        left_speed = max(-330, min(330, left_speed))
        right_speed = max(-330, min(330, right_speed))

        # Format command string (for DDSM115 driver node)
        cmd = f"{left_speed} {right_speed}"

        msg = String()
        msg.data = cmd
        self.publisher_.publish(msg)

        self.get_logger().info(f"Published Motor Command: {cmd}")

    def apply_brakes(self):
        """ Publishes a brake command to /motor_cmd. """
        msg = String()
        msg.data = "brake"
        self.publisher_.publish(msg)

        # Reset speed variables
        self.forward_speed = 0
        self.turn_speed = 0

        self.get_logger().info("Brake applied. Motors stopped.")

    def auto_decay_speed(self):
        """ Gradually reduces speed when no key is pressed (natural slowing effect). """
        if self.forward_speed > 0:
            self.forward_speed = max(0, self.forward_speed - self.auto_decay)
        elif self.forward_speed < 0:
            self.forward_speed = min(0, self.forward_speed + self.auto_decay)

        if self.turn_speed > 0:
            self.turn_speed = max(0, self.turn_speed - self.auto_decay)
        elif self.turn_speed < 0:
            self.turn_speed = min(0, self.turn_speed + self.auto_decay)

    def run(self):
        """ Main loop for keyboard control. """
        self.get_logger().info("Use W/A/S/D to control, Space to brake, and Q to quit.")
        try:
            while True:
                key = self.get_key()
                if key.lower() == 'w':
                    self.forward_speed += self.speed_increment
                elif key.lower() == 's':
                    self.forward_speed -= self.speed_increment
                elif key.lower() == 'a':
                    self.turn_speed -= self.turn_increment
                elif key.lower() == 'd':
                    self.turn_speed += self.turn_increment
                elif key == ' ':
                    self.apply_brakes()
                elif key.lower() == 'q':
                    self.apply_brakes()
                    break

                # Apply gradual speed decay if no key is pressed
                self.auto_decay_speed()

                # Send updated motor commands
                self.send_motor_commands()

                # Small delay to avoid excessive CPU usage
                time.sleep(0.1)

        except KeyboardInterrupt:
            pass
        finally:
            self.get_logger().info("Exiting keyboard control.")

def main(args=None):
    """ Entry point for the ROS2 node. """
    rclpy.init(args=args)
    node = KeyboardControlNode()
    node.run()
    rclpy.shutdown()
