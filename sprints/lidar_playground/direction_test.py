from rplidar import RPLidar, RPLidarException
import numpy as np
import time

port = '/dev/ttyUSB0'
lidar = RPLidar(port)

curr_scan = None
angle_by_dir = {
    'S': [337.5, 22.5],
    'SW': [45, 135],
    'N': [135, 225],
    'E': [225, 315]
}
angle_by_dir = {
    'S': [337.5, 22.5],
    'SW': [22.5, 67.5],
    'W': [67.5, 112.5],
    'NW': [112.5, 157.5],
    'N': [157.5, 202.5],
    'NE': [202.5, 247.5],
    'E': [247.5, 292.5],
    'SE': [292.5, 337.5]
}
close_points_by_dir = {
    'N': 0,
    'NE': 0,
    'E': 0,
    'SE': 0,
    'S': 0,
    'SW': 0,
    'W': 0,
    'NW': 0
}

while curr_scan is None:
    try:
        for i, scan in enumerate(lidar.iter_scans()):
            curr_scan = scan
            if i > 0:
                break
    except RPLidarException:
        lidar.stop_motor()
        lidar.stop()
        lidar.disconnect()
        lidar = RPLidar(port)
        continue

for direction, angle_range in angle_by_dir.items():
    if direction == 'S':
        close_points_by_dir[direction] = len([point for point in curr_scan if (point[1] > angle_range[0] or point[1] < angle_range[1]) and point[2] < 1000])
    else:
        close_points_by_dir[direction] = len([point for point in curr_scan if point[1] > angle_range[0] and point[1] < angle_range[1] and point[2] < 1000])

print(close_points_by_dir)
print(min(close_points_by_dir, key=close_points_by_dir.get))

lidar.stop_motor()
lidar.stop()
lidar.disconnect()