import rclpy
from rclpy.node import Node
import time
from serial import Serial  # comment out the real serial import
#from .fake_serial import FakeSerial  # # Use FakeSerial for testing if no real port is available.
from std_msgs.msg import Float64
from msgs_pkg.msg import Arduino  # see [`msgs_pkg.msg.Arduino`](src/msgs_pkg/msg/Arduino.msg)

class SerialOut(Node):
    def __init__(self):
        super().__init__('serial_out_node')
        # Identify and set the appropriate serial port and baud rate.
        portID = '/dev/ttyUSB0'
        self.serial_port = Serial(portID, 115200, timeout=1)
        # create subscription to the keyboard_control_topic
        self.subscription = self.create_subscription(Arduino, 'terminal_ctrl_topic', self.serial_output, 1)

    def serial_output(self, msg: Arduino):
        # Convert the message to a string.
        data_str = (
            f"{msg.left_motor_speed.data},"
            f"{msg.right_motor_speed.data},"
            f"{msg.left_motor_direction.data}," 
            f"{msg.right_motor_direction.data}\n"
        )
        # Send the data via the serial port
        self.serial_port.write(data_str.encode('utf-8'))
        time.sleep(0.05)
        # Optionally read the fake response from the fake serial port.
        response = self.serial_port.readline().decode('utf-8')
        self.get_logger().info(f"Response: {response.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialOut()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()