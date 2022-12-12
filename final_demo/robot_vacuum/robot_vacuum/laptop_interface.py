import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tty
import sys
import select
import termios


class SendTeleopNode(Node):
    """
    The SendTeleopNode, which runs on a laptop processes keyboard inputs from
    the user and sends it over ROS to the raspberry pi on the physical robot.
    This is intended to serve as a controller in the remote control functionality
    of the robot.
    """

    def __init__(self):
        """
        Constructor for the ROS node.
        """
        # Initialize the Node.
        super().__init__("send_drive_command")
        self.settings = termios.tcgetattr(sys.stdin)

        # Create a ROS publisher that publishes which key is pressed
        self.publisher = self.create_publisher(String, "vel_dir", 10)

        # Create a timer to continuously call run_loop
        self.timer = self.create_timer(0.1, self.run_loop)

    def get_key(self):
        """
        This is a binding function that listens for keyboard input from
        the user and returns which key is pressed.
        """
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run_loop(self):
        """
        The run loop function is continuously called. It listens for the user's
        keyboard inputs and publishes it for the raspberry pi.
        """
        # Listen for a key press
        self.key = self.get_key()

        # If Ctrl+C is pressed, a stop command is sent to the raspberry pi and
        # a keyboard interrupt is raised
        if self.key == "\x03":
            self.publisher.publish(String(data="s"))
            raise KeyboardInterrupt

        # Publish the detected key press to the raspberry pi
        self.publisher.publish(String(data=self.key))


def main(args=None):
    """
    Main runner function for the SendTeleopNode. The node is initialized and
    spun. When the node is terminated, shutdown is called.
    """
    rclpy.init(args=args)
    node = SendTeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()
