from rplidar import RPLidar
import pdb
import numpy as np
import matplotlib.pyplot as plt
import time

lidar = RPLidar('/dev/ttyUSB0')

# info = lidar.get_info()
# print(info)

# health = lidar.get_health()
# print(health)

# x = []
# y = []

# plt.ion()
# fig = plt.figure()
# ax = fig.add_subplot(111)
# line1, = ax.plot(x,y,'b.')
# ax.set_xbound(lower=-1000, upper=1000)
# ax.set_ybound(lower=-1000, upper=1000)

close_points_by_dir = {
    'N': 0,
    'E': 0,
    'S': 0,
    'W': 0,
}
angle_by_dir = {
    'E': [315, 45],
    'N': [45, 135],
    'W': [135, 225],
    'S': [225, 315],
}

for i, scan in enumerate(lidar.iter_scans()):
    angles = [point[1] for point in scan]
    dists = [point[2]/10 for point in scan]
    for direction, angle_range in angle_by_dir.items():
        if direction == 'E':
            close_points_by_dir[direction] = len([point for point in scan if (point[1] > angle_range[0] or point[1] < angle_range[1]) and point[2] < 1000])
        else:
            close_points_by_dir[direction] = len([point for point in scan if point[1] > angle_range[0] and point[1] < angle_range[1] and point[2] < 1000])

    next_dir = min(close_points_by_dir, key=close_points_by_dir.get)

    if next_dir == 'N':
        plt.quiver(0, 0, 0, 1000)
    if next_dir == 'E':
        plt.quiver(0, 0, 1000, 0)
    if next_dir == 'S':
        plt.quiver(0, 0, 0, -1000)
    if next_dir == 'W':
        plt.quiver(0, 0, -1000, 0)

    x = dists * np.cos(np.deg2rad(angles))
    y = dists * np.sin(np.deg2rad(angles))
    plt.plot(x, y, 'o', label='LiDAR Scan')
    plt.plot(0, 0, 'ko', markersize=1, label='Robot Drive Velocity')
    plt.title('LiDAR Processing Simulation')
    plt.xlabel('x (cm)')
    plt.ylabel('y (cm)')
    plt.legend()
    plt.show()
    # line1.set_xdata(x)
    # line1.set_ydata(y)
    # fig.canvas.draw()
    # fig.canvas.flush_events()


lidar.stop()
lidar.stop_motor()
lidar.disconnect()