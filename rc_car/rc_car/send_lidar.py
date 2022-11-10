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
        # self.lidar_port = self.get_usb_port()
        self.lidar_port = '/dev/ttyUSB0'
        self.scan: LaserScan = None

    def get_usb_port(self):
        available_ports = comports()
        for port in available_ports:
            if self.LIDAR_USB_PORT_NAME in port:
                self.usb_port = port
                return
        print("LiDAR not found!")
        rclpy.shutdown()

    def run_loop(self):
        with Sweep(self.usb_port) as sweep:
            sweep.start_scanning()
            data = np.array([sample.distance, sample.angle/1000] for sample in next(sweep.get_scans())[0])
            self.scan.angle_max = max(data[:, 1])
            self.scan.angle_min = min(data[:, 1])
            self.scan.angle_increment = (self.scan.angle_max - self.scan.angle_min)/len(data)
            self.scan.ranges = data[:,0]
        self.lidar_pub.publish(self.scan)

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    rclpy.shutdown()
