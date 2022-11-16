import rclpy
from rclpy.node import Node
from .stepper_motors.stepper_motor_control import *
from rplidar import RPLidar, RPLidarException

class ObstacleAvoidanceNode(Node):

    # angle_by_dir = {
    #     'N': [337.5, 22.5],
    #     'NE': [22.5, 67.5],
    #     'E': [67.5, 112.5],
    #     'SE': [112.5, 157.5],
    #     'S': [157.5, 202.5],
    #     'SW': [202.5, 247.5],
    #     'W': [247.5, 292.5],
    #     'NW': [292.5, 337.5]
    # }

    angle_by_dir = {
        'N': [315, 45],
        'E': [45, 135],
        'S': [135, 225],
        'W': [225, 315],
    }

    # drive_cmd_by_dir = {
    #     'N': drive_north,
    #     'NE': drive_diag_NE,
    #     'E': drive_east,
    #     'SE': drive_diag_SE,
    #     'S': drive_south,
    #     'SW': drive_diag_SW,
    #     'W': drive_west,
    #     'NW': drive_diag_NW,
    # }


    drive_cmd_by_dir = {
        'N': drive_north,
        'E': drive_east,
        'S': drive_south,
        'W': drive_west,
    }

    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.lidar_port = '/dev/ttyUSB0'
        # self.lidar = RPLidar(self.lidar_port)
        self.timer = self.create_timer(.1, self.run_loop)
        self.lidar_scan = None
        # self.close_points_by_dir = {
        #     'N': 0,
        #     'NE': 0,
        #     'E': 0,
        #     'SE': 0,
        #     'S': 0,
        #     'SW': 0,
        #     'W': 0,
        #     'NW': 0
        # }
        self.close_points_by_dir = {
            'N': 0,
            'E': 0,
            'S': 0,
            'W': 0,
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
            if direction == 'N':
                self.close_points_by_dir[direction] = len([point for point in self.lidar_scan if (point[1] > angle_range[0] or point[1] < angle_range[1]) and point[0] < 1000])
            else:
                self.close_points_by_dir[direction] = len([point for point in self.lidar_scan if point[1] > angle_range[0] and point[1] < angle_range[1] and point[0] < 1000])
        self.next_dir = min(self.close_points_by_dir, key=self.close_points_by_dir.get)

    def run_loop(self):
        self.get_lidar_scan()
        self.process_lidar()
        self.drive_cmd_by_dir[self.next_dir]()

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown()