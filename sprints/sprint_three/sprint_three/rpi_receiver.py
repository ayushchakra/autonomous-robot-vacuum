import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

ARDUINO_PORT = "/dev/ttyACM0"
SERIAL_BAUD_RATE = 9600


class RPITeleopReceiverNode(Node):
    input_keys_to_serial = {
        "q": str.encode("6"),
        "w": str.encode("1"),
        "e": str.encode("5"),
        "a": str.encode("4"),
        "s": str.encode("11"),
        "d": str.encode("3"),
        "z": str.encode("8"),
        "x": str.encode("2"),
        "c": str.encode("7"),
        "r": str.encode("9"),
        "t": str.encode("10"),
    }

    def __init__(self):
        super().__init__("rpi_teleop_receiver_node")
        self.initialize_serial()
        self.subscriber = self.create_subscription(
            String, "vel_dir", self.process_keyboard_input, 10
        )

    def initialize_serial(self):
        self.ser = serial.Serial("/dev/ttyACM0", 9600, timeout=1)

    def process_keyboard_input(self, msg: String):
        self.ser.write(self.input_keys_to_serial[msg.data])


def main(args=None):
    rclpy.init(args=args)
    node = RPITeleopReceiverNode()
    rclpy.spin(node)
    rclpy.shutdown()
