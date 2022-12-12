import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np


class ReceiveLaserScanNode(Node):
    def __init__(self):
        super().__init__("receive_laser_scan_node")
        self.laser_sub = self.create_subscription(
            Float64MultiArray, "scan", self.process_scan, 10
        )
        self.x = []
        self.y = []
        plt.ion()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111)
        (self.line1,) = self.ax.plot(self.x, self.y, "b.")
        self.ax.set_xbound(lower=-250, upper=250)
        self.ax.set_ybound(lower=-250, upper=250)

    def process_scan(self, msg: Float64MultiArray):
        angles = np.array(msg.data[len(msg.data) // 2 :])
        distances = np.array(msg.data[: len(msg.data) // 2])
        self.x = distances * (np.cos(np.deg2rad(angles)))
        self.y = distances * (np.sin(np.deg2rad(angles)))
        print(self.y)
        self.line1.set_xdata(self.x)
        self.line1.set_ydata(self.y)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = ReceiveLaserScanNode()
    rclpy.spin(node)
    rclpy.shutdown()
