import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sweeppy import Sweep
from serial.tools.list_ports import comports
import numpy as np

class LidarNode(Node):
    LIDAR_USB_PORT_NAME = ''
    def __init__(self):
        super().__init__('lidar_node')
        self.lidar_pub = self.create_publisher(LaserScan, "scan", 10)
        self.timer = self.create_timer(.1, self.run_loop)
        self.lidar_port = self.get_usb_port()
        self.scan: LaserScan = None

    def get_usb_port(self):
        available_ports = comports()
        self.usb_port = ''

    def run_loop(self):
        with Sweep(self.usb_port) as sweep:
            sweep.start_scanning()
            data = np.array([(point.angle, point.distance) for point in sweep.get_scans()[0]])
            self.scan.angle_max = max(data[:, 0])
            self.scan.angle_min = min(data[:, 0])
            self.scan.angle_increment = (self.scan.angle_max - self.scan.angle_min)/len(data)
            self.scan.ranges = data[:,1]
        self.lidar_pub.publish(self.scan)

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    rclpy.shutdown()