import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .stepper_motors.stepper_motor_control import *

class SimpleReceiveTeleop(Node):
    def __init__(self):
        super().__init__('simple_receive_teleop')
        self.subscriber = self.create_subscription(String, 'vel_dir', self.process_keyboard_input, 10)
        setup_all_pins()

    def process_keyboard_input(self, msg: String):
        print(msg.data)
        if msg.data == 'q':
            drive_diag_NW(MOTOR_ONE_PINS, MOTOR_TWO_PINS, MOTOR_THREE_PINS, MOTOR_FOUR_PINS, 0.002, 500)
        elif msg.data == 'w':
            drive_north(MOTOR_ONE_PINS, MOTOR_TWO_PINS, MOTOR_THREE_PINS, MOTOR_FOUR_PINS, 0.002, 500)
        elif msg.data == 'e':
            drive_diag_NE(MOTOR_ONE_PINS, MOTOR_TWO_PINS, MOTOR_THREE_PINS, MOTOR_FOUR_PINS, 0.002, 500)
        elif msg.data == 'a':
            drive_west(MOTOR_ONE_PINS, MOTOR_TWO_PINS, MOTOR_THREE_PINS, MOTOR_FOUR_PINS, 0.002, 500)
        elif msg.data == 's':
            pass
        elif msg.data == 'd':
            drive_east(MOTOR_ONE_PINS, MOTOR_TWO_PINS, MOTOR_THREE_PINS, MOTOR_FOUR_PINS, 0.002, 500)
        elif msg.data == 'z':
            drive_diag_SW(MOTOR_ONE_PINS, MOTOR_TWO_PINS, MOTOR_THREE_PINS, MOTOR_FOUR_PINS, 0.002, 500)
        elif msg.data == 'x':
            drive_south(MOTOR_ONE_PINS, MOTOR_TWO_PINS, MOTOR_THREE_PINS, MOTOR_FOUR_PINS, 0.002, 500)
        elif msg.data == 'c':
            drive_diag_SE(MOTOR_ONE_PINS, MOTOR_TWO_PINS, MOTOR_THREE_PINS, MOTOR_FOUR_PINS, 0.002, 500)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleReceiveTeleop()
    rclpy.spin(node)
    rclpy.shutdown()
