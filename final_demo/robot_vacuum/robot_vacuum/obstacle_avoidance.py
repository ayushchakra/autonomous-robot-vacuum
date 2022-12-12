import rclpy
from rclpy.node import Node
from rplidar import RPLidar, RPLidarException
import serial

# USB Port for the LiDAR
LIDAR_PORT = "/dev/ttyUSB0"

# USB Port for the Ardunio
SERIAL_PORT = "/dev/ttyACM0"

# Close distance threshold for points close to the robot
CLOSE_DIST_THRESH = 500


class ObstacleAvoidanceNode(Node):
    """
    ROS Node for running the obstacle avoidance algorithm on the robot. The
    robot processes a LiDAR scan every second and navigate towards the direction
    of the least obstacles. If all directions possess obstacles within the close
    distance threshold, then the robot just spins in place.
    """

    # Dictionary that maps each cardinal direction to a LiDAR angle scan range.
    angle_by_dir = {
        "S": [337.5, 22.5],
        "SW": [22.5, 67.5],
        "W": [67.5, 112.5],
        "NW": [112.5, 157.5],
        "N": [157.5, 202.5],
        "NE": [202.5, 247.5],
        "E": [247.5, 292.5],
        "SE": [292.5, 337.5],
    }

    # Dictionary that maps each cardinal direction to an encoded integer to be
    # sent over serial to the arduino.
    dir_to_serial = {
        "S": str.encode("2"),
        "SW": str.encode("8"),
        "W": str.encode("4"),
        "NW": str.encode("6"),
        "N": str.encode("1"),
        "NE": str.encode("5"),
        "E": str.encode("3"),
        "SE": str.encode("7"),
        "R": str.encode("9"),
    }

    def __init__(self):
        """
        Constructor for the obstacle avoidance node. This function initializes
        the serial connection, connection to the LiDAR, and instance variables
        for the drive command logic.
        """
        super().__init__("obstacle_avoidance_node")

        # Creates a timer to continuously call run_loop ever second.
        self.timer = self.create_timer(1, self.run_loop)

        # Initializes the serial connection port
        self.ser_con = None
        self.initialize_serial()

        # Initializes the LiDAR
        self.lidar = RPLidar(LIDAR_PORT)
        self.lidar_scan = None

        # Dictionary that maps each cardinal directions to the number of points
        # that are within CLOSE_DIST_THRESH of the robot in that angle range.
        self.close_points_by_dir = {
            "N": 0,
            "NE": 0,
            "E": 0,
            "SE": 0,
            "S": 0,
            "SW": 0,
            "W": 0,
            "NW": 0,
        }

        # Dictionary that maps each cardinal direction to the total distance
        # to the obstacles in that range.
        self.dist_by_dir = {
            "N": 0,
            "NE": 0,
            "E": 0,
            "SE": 0,
            "S": 0,
            "SW": 0,
            "W": 0,
            "NW": 0,
        }
        # The current drive dirction of the robot
        self.drive_dir = None

    def initialize_serial(self):
        """
        Initializes the serial connection to the arduino.
        """
        self.ser_con = serial.Serial(SERIAL_PORT, 9600, timeout=1)

    def run_loop(self):
        """
        Main runner for the ROS node that is continuously called.
        """
        # Update the current lidar scan
        self.update_lidar_scan()

        # Process the scan to determine the optimal driving direction
        self.process_lidar_scan()

        # Send the computed drive command to the arduino
        self.update_drive_command()

    def update_lidar_scan(self):
        """
        This function dictates the communication with the LiDAR to process
        the current scan.
        """
        # Nested in a try-except statement because there are sometimes problems
        # with establishing communication if the LiDAR wasn't properly shtudown
        # on the previous iteration.
        try:
            # Conduct two scans and only store the second one. This was implemented
            # as we found that there was some data loss with only processing the
            # first scan
            for i, scan in enumerate(self.lidar.iter_scans()):
                self.lidar_scan = scan
                if i > 0:
                    break
        except RPLidarException:
            # If RPLiDARException is raised, power cycle the LiDAR
            self.lidar.stop_motor()
            self.lidar.stop()
            self.lidar.disconnect()
            self.lidar = RPLidar(LIDAR_PORT)

    def process_lidar_scan(self):
        """
        After the LiDAR data is updated, it is processed to determine the number
        of close points at each cardinal direction and the total distance to
        obstacles in each direction
        """
        # Iterate through each possible direction and corresponding angle range
        for direction, angle_range in self.angle_by_dir.items():
            # Special case for south since the angle wraps around 360
            if direction == "S":
                # Find the number of points that are within the CLOSE_DIST_THRESH
                self.close_points_by_dir[direction] = len(
                    [
                        point
                        for point in self.lidar_scan
                        if (point[1] > angle_range[0] or point[1] < angle_range[1])
                        and point[2] < CLOSE_DIST_THRESH
                    ]
                )
                # Sum the total distance to the obstacles in the current range
                self.dist_by_dir[direction] = sum(
                    [
                        point[2]
                        for point in self.lidar_scan
                        if point[1] > angle_range[0] or point[1] < angle_range[1]
                    ]
                )
            else:
                # Find the number of points that are within the CLOSE_DIST_THRESH
                self.close_points_by_dir[direction] = len(
                    [
                        point
                        for point in self.lidar_scan
                        if point[1] > angle_range[0]
                        and point[1] < angle_range[1]
                        and point[2] < CLOSE_DIST_THRESH
                    ]
                )
                # Sum the total distance to the obstacles in the current range
                self.dist_by_dir[direction] = sum(
                    [
                        point[2]
                        for point in self.lidar_scan
                        if point[1] > angle_range[0] and point[1] < angle_range[1]
                    ]
                )

    def update_drive_command(self):
        # If there are close obstacles in all directions, rotate in palce
        if min(self.close_points_by_dir.values()) > 0:
            self.ser_con.write(self.dir_to_serial["R"])
        else:
            # Set the drive direction to the direction with the minimum total
            # distance to obstacles
            self.drive_dir = min(self.dist_by_dir, key=self.close_points_by_dir.get)
            self.ser_con.write(self.dir_to_serial[self.drive_dir])


def main(args=None):
    """
    Main runner function for the ObstacleAvoidanceNode. The node is initialized and
    spun. When the node is terminated, shutdown is called.
    """
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    rclpy.shutdown()
