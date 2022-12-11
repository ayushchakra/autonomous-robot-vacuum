"""
Note: For this file to be properly used, the sweeppy module
must be downloaded from here: https://github.com/scanse/sweep-sdk
"""

import pdb
from sweeppy import Sweep
import matplotlib.pyplot as plt
import numpy as np

x = []
y = []

with Sweep('/dev/ttyUSB1') as sweep:
    sweep.start_scanning()

#    plt.ion()
#    fig = plt.figure()
#    ax = fig.add_subplot(111)
#    line1, = ax.plot(x,y,'b.')
#    ax.set_xbound(lower=-250, upper=250)
#    ax.set_ybound(lower=-250, upper=250)

    for scan in sweep.get_scans():
        angles = []
        distances = []
        for sample in scan[0]:
            angles.append(sample.angle)
            distances.append(sample.distance)
        angles = np.array(angles)
        distances = np.array(distances)
        pdb.set_trace()
        x = distances*(np.cos(np.deg2rad(angles/1000)))
        y = distances*(np.sin(np.deg2rad(angles/1000)))

 #       line1.set_xdata(x)
 #       line1.set_ydata(y)
 #       fig.canvas.draw()
 #       fig.canvas.flush_events()
            
