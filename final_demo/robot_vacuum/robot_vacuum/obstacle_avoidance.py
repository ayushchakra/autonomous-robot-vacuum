import rclpy
from rclpy.node import Node
from rplidar import RPLidar, RPLidarException
import serial
import pdb

LIDAR_PORT = '/dev/ttyUSB0'
SERIAL_PORT = '/dev/ttyACM0'
CLOSE_DIST_THRESH = 500

class ObstacleAvoidanceNode(Node):

    angle_by_dir = {
        'S': [337.5, 22.5],
        'SW': [22.5, 67.5],
        'W': [67.5, 112.5],
        'NW': [112.5, 157.5],
        'N': [157.5, 202.5],
        'NE': [202.5, 247.5],
        'E': [247.5, 292.5],
        'SE': [292.5, 337.5],
    }

    dir_to_serial = {
        'S': str.encode('2'),
        'SW': str.encode('8'),
        'W': str.encode('4'),
        'NW': str.encode('6'),
        'N': str.encode('1'),
        'NE': str.encode('5'),
        'E': str.encode('3'),
        'SE': str.encode('7'),
        'R': str.encode('9')
    }

    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.timer = self.create_timer(1, self.run_loop)

        self.ser_con = None
        self.initialize_serial()

        self.lidar = RPLidar(LIDAR_PORT)
        self.lidar_scan = None
        self.close_points_by_dir = {
            'N': 0,
            'NE': 0,
            'E': 0,
            'SE': 0,
            'S': 0,
            'SW': 0,
            'W': 0,
            'NW': 0
        }
        self.dist_by_dir = {
            'N': 0,
            'NE': 0,
            'E': 0,
            'SE': 0,
            'S': 0,
            'SW': 0,
            'W': 0,
            'NW': 0
        }

        self.drive_dir = None

    def initialize_serial(self):
        self.ser_con = serial.Serial(SERIAL_PORT, 9600, timeout=1)

    def run_loop(self):
        self.update_lidar_scan()
        self.process_lidar_scan()
        self.update_drive_command()

    def update_lidar_scan(self):
        try: 
            for i, scan in enumerate(self.lidar.iter_scans()):
                self.lidar_scan = scan
                if i > 0:
                    break
        except RPLidarException:
            self.lidar.stop_motor()
            self.lidar.stop()
            self.lidar.disconnect()
            self.lidar = RPLidar(LIDAR_PORT)

    def process_lidar_scan(self):
        for direction, angle_range in self.angle_by_dir.items():
            if direction == 'S':
                self.close_points_by_dir[direction] = len([point for point in\
                    self.lidar_scan if (point[1] > angle_range[0] or point[1]\
                    < angle_range[1]) and point[2] < CLOSE_DIST_THRESH])
                self.dist_by_dir[direction] = sum([point[2] for point in\
                    self.lidar_scan if point[1] > angle_range[0] or point[1]\
                    < angle_range[1]])
            else:
                self.close_points_by_dir[direction] = len([point for point in\
                    self.lidar_scan if point[1] > angle_range[0] and point[1]\
                    < angle_range[1] and point[2] < CLOSE_DIST_THRESH])
                self.dist_by_dir[direction] = sum([point[2] for point in\
                    self.lidar_scan if point[1] > angle_range[0] and point[1]\
                    < angle_range[1]])

    def update_drive_command(self):
        if min(self.close_points_by_dir.values()) > 0:
            self.ser_con.write(self.dir_to_serial['R'])
            print("here")
        else:
            self.drive_dir = min(self.dist_by_dir, key=self.close_points_by_dir.get)
            self.ser_con.write(self.dir_to_serial[self.drive_dir])
            print(self.drive_dir)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown()
