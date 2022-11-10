import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ReceiveIPNode(Node):
    def __init__(self):
        super().__init__('receive_ip_node')
        self.create_subscription(String, 'ip_address', self.receive_ip, 10)

    def receive_ip(self, msg: String):
        print(msg)

def main(args=None):
    rclpy.init(args=args)
    receive_ip_node = ReceiveIPNode()
    rclpy.spin(receive_ip_node)
    rclpy.shutdown()