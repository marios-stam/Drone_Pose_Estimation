from visualize import log_data
import numpy as np

logs_path = "/home/marios/catkin_ws/src/Drone_Pose_Estimation/logs/"
raw = log_data(logs_path+'raw_values.txt')

stds = np.std(raw.x), np.std(raw.y), np.std(raw.z)
means = np.mean(raw.x), np.mean(raw.y), np.mean(raw.z)
print(stds)
print(means)
