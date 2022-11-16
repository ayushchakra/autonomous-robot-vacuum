import rclpy
from rclpy.node import Node
from std_msgs.msg import Char
import tty
import sys
import select
import termios

class SendTeleopNode(Node):
    def __init__(self):
        super().__init__('send_drive_command')
        self.settings = termios.tcgetattr(sys.stdin)
        self.publisher = self.create_publisher(Char, "vel_dir", 10)
        self.timer = self.create_timer(.1, self.run_loop)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run_loop(self):
        self.key = self.get_key()
        if self.key == "\x03":
            self.publisher.publish(self.key_to_vel["s"])
            raise KeyboardInterrupt
        self.publisher.publish(Char(data=self.key))

def main(args=None):
    rclpy.init()
    node = SendTeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()