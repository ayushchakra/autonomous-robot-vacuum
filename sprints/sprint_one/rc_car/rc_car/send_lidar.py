import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sweeppy import Sweep
from serial.tools.list_ports import comports
import numpy as np
import pdb
import time


class LidarNode(Node):
    LIDAR_USB_PORT_NAME = ""

    def __init__(self):
        super().__init__("lidar_node")
        self.lidar_pub = self.create_publisher(Float64MultiArray, "scan", 10)
        self.timer = self.create_timer(0.1, self.run_loop)
        # self.lidar_port = self.get_usb_port()
        self.lidar_port = "/dev/ttyUSB0"
        self.scan: Float64MultiArray = Float64MultiArray()

    def get_usb_port(self):
        available_ports = comports()
        for port in available_ports:
            if self.LIDAR_USB_PORT_NAME in port:
                self.usb_port = port
                return
        print("LiDAR not found!")
        rclpy.shutdown()

    def run_loop(self):
        with Sweep(self.lidar_port) as sweep:
            sweep.start_scanning()
            record_data = False
            for scan in sweep.get_scans():
                angles = [float(sample.angle / 1000) for sample in scan[0]]
                distances = [float(sample.distance) for sample in scan[0]]
                if record_data:
                    break
                record_data = True
            self.scan.data = distances + angles
        self.lidar_pub.publish(self.scan)
        print("sent")


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    rclpy.shutdown()
