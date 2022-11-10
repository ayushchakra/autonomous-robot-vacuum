import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import netifaces as ni

class SendIPNode(Node):
    def __init__(self):
        super().__init__("send_ip_node")
        self.ip_address: String = String()
        self.ip_address.data = ni.ifaddresses('wlo1')[ni.AF_INET][0]['addr']
        self.timer = self.create_timer(1, self.send_ip_address)
        self.publisher = self.create_publisher(String, 'ip_address', 10)

    def send_ip_address(self):
        self.publisher.publish(self.ip_address)

def main(args=None):
    rclpy.init(args=args)
    send_ip_node = SendIPNode()
    rclpy.spin(send_ip_node)
    rclpy.shutdown()