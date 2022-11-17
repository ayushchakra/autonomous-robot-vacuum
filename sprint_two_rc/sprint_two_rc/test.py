import numpy as np

test = np.concatenate([np.random.randint(low=0, high=15, size=5), np.random.randint(low=0, high=15, size=5)])
print(test)

lidar_scan = np.array([np.random.randint(low=0, high=15, size=50), np.random.randint(low=0, high=180, size=50), np.random.randint(low=100, high=2000, size=50)]).T
print(lidar_scan)