from inputs import get_key
import rclpy
from rclpy.node import Node
from msgs_pkg.msg import Arduino
from std_msgs.msg import Float64

class KeyboardControl(Node):
    def __init__(self):
        # Initialize the node with the name keyboard_control
        super().__init__('keyboard_control_node')
        # Create a publisher to publish messages of type Arduino to the topic keyboard_control_topic
        self.publisher = self.create_publisher(Arduino, 'keyboard_control_topic', 1)

        # Set up a timer to poll for keyboard events periodically.
        self.timer = self.create_timer(0.01, self.check_keyboard)

    def check_keyboard(self):
        left_motor_speed = 0.0
        right_motor_speed = 0.0
        left_motor_direction = 'LOW'
        right_motor_direction = 'LOW'
        left_multiplier = 0.7

        events = get_key()  # Grab all key events
        event = events[-1]
        if event.ev_type == "Key":
            if event.state == 1 or event.state == 2:
                match event.code:
                    case 'KEY_W':
                        self.get_logger().info('forward')
                        left_motor_speed = 100.0*left_multiplier
                        right_motor_speed = 100.0
                        left_motor_direction = 'HIGH'
                        right_motor_direction = 'HIGH'
                    case 'KEY_S':
                        self.get_logger().info('backward')
                        left_motor_speed = 100.0*left_multiplier
                        right_motor_speed = 100.0
                        left_motor_direction = 'LOW'
                        right_motor_direction = 'LOW'
                    case 'KEY_A':
                        self.get_logger().info('left')
                        left_motor_speed = 100.0*left_multiplier
                        right_motor_speed = 100.0
                        left_motor_direction = 'HIGH'
                        right_motor_direction = 'HIGH'                    
                    case 'KEY_D':
                        self.get_logger().info('right')
                        left_motor_speed = 100.0*left_multiplier
                        right_motor_speed = 100.0
                        left_motor_direction = 'HIGH'
                        right_motor_direction = 'HIGH'
            msg = Arduino()
            msg.left_motor_speed.data = left_motor_speed
            msg.right_motor_speed.data = right_motor_speed
            msg.left_motor_direction.data = left_motor_direction
            msg.right_motor_direction.data = right_motor_direction
            self.publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    keyboard_control = KeyboardControl()
    rclpy.spin(keyboard_control)
    keyboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


