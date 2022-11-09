import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ReceiveLaserScaneNode(Node):
    def __init__(self):
        super().__init__("receive_laser_scan_node")
        self.laser_sub = self.create_subscription(LaserScan, "scan", self.process_scan, 10)
    
    def process_scan(self, msg: LaserScan):
        print("scan received")

def main(args=None):
    rclpy.init(args=args)
    node = ReceiveLaserScaneNode()
    rclpy.spin(node)
    rclpy.shutdown()