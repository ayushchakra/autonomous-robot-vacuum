import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

# USB Port for the arduino
ARDUINO_PORT = "/dev/ttyACM0"

# Baud rate for serial communication between raspi and arduino
SERIAL_BAUD_RATE = 9600


class RPITeleopReceiverNode(Node):
    """
    The RPITeleopReceiverNode serves to receive keyboard inputs from the laptop
    interface and send it over serial to the arduino.
    """

    # Dictionary that maps input key pressed to encoded integers to be sent
    # over serial.
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
        """
        Constructor for the ROS node, which initializes the serial port and
        subscribes to the keyboard inputs being sent by the laptop.
        """
        super().__init__("rpi_teleop_receiver_node")
        self.initialize_serial()
        self.subscriber = self.create_subscription(
            String, "vel_dir", self.process_keyboard_input, 10
        )

    def initialize_serial(self):
        """
        Opens the serial connection between the arduino
        """
        self.ser = serial.Serial(ARDUINO_PORT, SERIAL_BAUD_RATE, timeout=1)

    def process_keyboard_input(self, msg: String):
        """
        This is the callback for the keyboard subscription. Every time a key is
        received, it is mapped to its corresponding encdoed value and sent over
        serial to the arduino.
        """
        self.ser.write(self.input_keys_to_serial[msg.data])


def main(args=None):
    """
    Main runner function for the RPITeleopReceiverNode. The node is initialized and
    spun. When the node is terminated, shutdown is called.
    """
    rclpy.init(args=args)
    node = RPITeleopReceiverNode()
    rclpy.spin(node)
    rclpy.shutdown()
