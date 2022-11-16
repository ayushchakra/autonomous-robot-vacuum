import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .stepper_motors.stepper_motor_control import *

class SimpleReceiveTeleop(Node):
    def __init__(self):
        super().__init__('simple_receive_teleop')
        self.subscriber = self.create_subscription(String, 'vel_dir', self.process_keyboard_input, 10)
        # setup_all_pins()

    def process_keyboard_input(self, msg: String):
        print(msg.data)
        if msg.data == 'q':
            drive_diag_NW()
        elif msg.data == 'w':
            drive_north()
        elif msg.data == 'e':
            drive_diag_NE()
        elif msg.data == 'a':
            drive_west()
        elif msg.data == 's':
            pass
        elif msg.data == 'd':
            drive_east()
        elif msg.data == 'z':
            drive_diag_SW()
        elif msg.data == 'x':
            drive_south()
        elif msg.data == 'c':
            drive_diag_SE()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleReceiveTeleop()
    rclpy.spin(node)
    rclpy.shutdown()
