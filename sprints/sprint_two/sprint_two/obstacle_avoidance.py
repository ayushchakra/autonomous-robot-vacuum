import rclpy
from rclpy.node import Node
from .stepper_motors.stepper_motor_control import *
from rplidar import RPLidar, RPLidarException
import numpy as np
import matplotlib.pyplot as plt


class ObstacleAvoidanceNode(Node):

    angle_by_dir = {
        "N": [315, 45],
        "E": [45, 135],
        "S": [135, 225],
        "W": [225, 315],
    }

    def __init__(self):
        super().__init__("obstacle_avoidance_node")
        self.lidar_port = "/dev/ttyUSB0"
        self.lidar = RPLidar(self.lidar_port)
        self.timer = self.create_timer(0.1, self.run_loop)
        self.lidar_scan = None
        self.close_points_by_dir = {
            "N": 0,
            "E": 0,
            "S": 0,
            "W": 0,
        }
        self.close_points_by_dir = {
            "N": 0,
            "E": 0,
            "S": 0,
            "W": 0,
        }
        self.next_dir = None

    def get_lidar_scan(self):
        successful_scan = False
        while not successful_scan:
            try:
                for i, scan in enumerate(self.lidar.iter_scans):
                    self.lidar_scan = scan
                    if i > 0:
                        successful_scan = True
            except RPLidarException:
                self.lidar.stop_motor()
                self.lidar.stop()
                self.lidar.disconnect()
                self.lidar = RPLidar(self.port)
                continue

    def process_lidar(self):
        for direction, angle_range in self.angle_by_dir.items():
            if direction == "N":
                self.close_points_by_dir[direction] = len(
                    [
                        point
                        for point in self.lidar_scan
                        if (point[1] > angle_range[0] or point[1] < angle_range[1])
                        and point[0] < 1000
                    ]
                )
            else:
                self.close_points_by_dir[direction] = len(
                    [
                        point
                        for point in self.lidar_scan
                        if point[1] > angle_range[0]
                        and point[1] < angle_range[1]
                        and point[0] < 1000
                    ]
                )
        self.next_dir = min(self.close_points_by_dir, key=self.close_points_by_dir.get)

    def fake_lidar_scan(self):
        self.lidar_scan = np.array(
            [
                np.random.randint(low=0, high=15, size=50),
                np.random.randint(low=0, high=180, size=50),
                np.random.randint(low=100, high=2000, size=50),
            ]
        ).T

    def visualize_lidar(self):
        x = self.lidar_scan[:, 2] * np.cos(np.deg2rad(self.lidar_scan[:, 1]))
        y = self.lidar_scan[:, 2] * np.sin(np.deg2rad(self.lidar_scan[:, 1]))
        print(self.next_dir)
        plt.plot(x, y, "o")
        # plt.xticks([-2000, 2000])
        # plt.yticks([-2000, 2000])
        plt.xlim([-2000, 2000])
        plt.ylim([-2000, 2000])
        if self.next_dir == "W":
            plt.quiver(0, 0, 0, -1000)
        if self.next_dir == "N":
            plt.quiver(0, 0, -1000, 0)
        if self.next_dir == "E":
            plt.quiver(0, 0, 0, 1000)
        if self.next_dir == "S":
            plt.quiver(0, 0, 1000, 0)
        plt.show()

    def run_loop(self):
        # self.get_lidar_scan()
        self.fake_lidar_scan()
        self.process_lidar()
        self.visualize_lidar()
        # self.drive_cmd_by_dir[self.next_dir]()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown()
