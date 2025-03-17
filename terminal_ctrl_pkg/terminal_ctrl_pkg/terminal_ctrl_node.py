import sys
import tty
import termios
import select
import rclpy
from rclpy.node import Node
from msgs_pkg.msg import Arduino
from std_msgs.msg import Float64

class TerminalControl(Node):
    def __init__(self):
        super().__init__('terminal_ctrl_node')
        self.publisher = self.create_publisher(Arduino, 'terminal_ctrl_topic', 1)
        self.timer = self.create_timer(0.05, self.check_keyboard)

        # Save original terminal settings and switch to raw mode.
        self.old_terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def check_keyboard(self):
        left_motor_speed = 0.0
        right_motor_speed = 0.0
        left_motor_direction = 'LOW'
        right_motor_direction = 'LOW'
        left_multiplier = 0.7

        # Flush all waiting input and keep only the last key.
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key = None
            while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)

            # Process only the last key after flushing.
            if key.lower() == 'w':
                self.get_logger().info('forward')
                left_motor_speed = 100.0 * left_multiplier
                right_motor_speed = 100.0
                left_motor_direction = 'HIGH'
                right_motor_direction = 'HIGH'
            elif key.lower() == 's':
                self.get_logger().info('backward')
                left_motor_speed = 100.0 * left_multiplier
                right_motor_speed = 100.0
                left_motor_direction = 'LOW'
                right_motor_direction = 'LOW'
            elif key.lower() == 'a':
                self.get_logger().info('left')
                left_motor_speed = 100.0 * left_multiplier
                right_motor_speed = 100.0
                left_motor_direction = 'LOW'
                right_motor_direction = 'HIGH'
            elif key.lower() == 'd':
                self.get_logger().info('right')
                left_motor_speed = 100.0 * left_multiplier
                right_motor_speed = 100.0
                left_motor_direction = 'HIGH'
                right_motor_direction = 'LOW'

        msg = Arduino()
        msg.left_motor_speed.data = left_motor_speed
        msg.right_motor_speed.data = right_motor_speed
        msg.left_motor_direction.data = left_motor_direction
        msg.right_motor_direction.data = right_motor_direction
        self.publisher.publish(msg)

    def destroy_node(self):
        # Restore original terminal settings before shutting down.
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_terminal_settings)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    keyboard_control = TerminalControl()
    try:
        rclpy.spin(keyboard_control)
    except KeyboardInterrupt:
        pass
    finally:
        keyboard_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
