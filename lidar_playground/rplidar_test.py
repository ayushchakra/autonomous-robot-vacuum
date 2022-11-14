from rplidar import RPLidar
import pdb
import numpy as np
import matplotlib.pyplot as plt

lidar = RPLidar('/dev/ttyUSB0')

info = lidar.get_info()
print(info)

# health = lidar.get_health()
# print(health)

x = []
y = []

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
line1, = ax.plot(x,y,'b.')
ax.set_xbound(lower=-250, upper=250)
ax.set_ybound(lower=-250, upper=250)

for i, scan in enumerate(lidar.iter_scans()):
    print('%d: Got %d measurments' % (i, len(scan)))
    angles = [point[1] for point in scan]
    dists = [point[2]/10 for point in scan]

    x = dists * np.cos(np.deg2rad(angles))
    y = dists * np.sin(np.deg2rad(angles))

    line1.set_xdata(x)
    line1.set_ydata(y)
    fig.canvas.draw()
    fig.canvas.flush_events()


lidar.stop()
lidar.stop_motor()
lidar.disconnect()