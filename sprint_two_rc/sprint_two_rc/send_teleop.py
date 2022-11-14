import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import tty
import sys
import select
import termios
import math

class SendTeleopNode(Node):

    base_speed = .1

    key_to_vel = {
        "q": Twist(
            linear=Vector3(x=math.sqrt(base_speed**2/2), y=math.sqrt(base_speed**2/2))
        ),
        "w": Twist(
            linear=Vector3(x=base_speed)
        ),
        "e": Twist(
            linear=Vector3(x=math.sqrt(base_speed**2/2), y=-math.sqrt(base_speed**2/2))
        ),
        "a": Twist(
            linear=Vector3(y=base_speed)
        ),
        "s": Twist(
            linear=Vector3()
        ),
        "d": Twist(
            linear=Vector3(y=-base_speed)
        ),
        "z": Twist(
            linear=Vector3(x=-math.sqrt(base_speed**2/2), y=math.sqrt(base_speed**2/2))
        ),
        "x": Twist(
            linear=Vector3(x=-base_speed)
        ),
        "c": Twist(
            linear=Vector3(x=-math.sqrt(base_speed**2/2), y=-math.sqrt(base_speed**2/2))
        ),
    }

    def __init__(self):
        super().__init__('send_drive_command')
        self.settings = termios.tcgetattr(sys.stdin)
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(.1, self.run_loop)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run_loop(self):
        self.key = self.get_key()
        if self.key == "i":
            self.base_speed += .05
            print(f'New Linear Speed: {self.base_speed}')
        if self.key == 'k':
            self.base_speed -= 0.5
            print(f'New Linear Speed: {self.base_speed}')
        if self.key == "\x03":
            self.publisher.publish(self.key_to_vel["s"])
            raise KeyboardInterrupt
        if self.key in self.key_to_vel.keys():
            self.publisher.publish(self.key_to_vel[self.key])

def main(args=None):
    rclpy.init()
    node = SendTeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()