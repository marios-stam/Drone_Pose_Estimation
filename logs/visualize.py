import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class log_data:
    def __init__(self, file_name) -> None:
        self.load_data(file_name)

    def load_data(self, file_name):
        data = np.loadtxt(file_name)
        t = data[:, 0]
        x, y, z = data[:, 1], data[:, 2], data[:, 3]
        qx, qy, qz, qw = data[:, 4], data[:, 5], data[:, 6], data[:, 7]

        self.t, self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw = t, x, y, z, qx, qy, qz, qw

        return t, x, y, z, qx, qy, qz, qw


def plot_filtered_data(raw, filtered_data_list):
    markers = ["o", "x", "+", "*", "s", "d", "^", "v", ">", "<", "p", "h"]
    labels = ['filtered', "filtered_accel", "filtered_jerk"]

    marker_size = 3
    marker_size_raw = marker_size+4

    for i in range(3):
        plt.subplot(3, 1, i+1)

        plt.plot(raw.t, raw.x, label='raw', marker='.',
                 markersize=marker_size_raw)

        for j, filtered in enumerate(filtered_data_list):
            y_axis_value = filtered.x if i == 0 else filtered.y if i == 1 else filtered.z

            plt.plot(filtered.t, y_axis_value,  # linestyle="",
                     marker=markers[j], markersize=marker_size, label=labels[j])

        plt.legend()
        plt.grid()

        title = "X" if i == 0 else "Y" if i == 1 else "Z"
        plt.title("X")
        plt.xlabel("t")
        plt.ylabel(title.lower())

    plt.subplots_adjust(hspace=0.46)


def plot_coordinates(raw, filtered, filtered_accel, filtered_jerk):
    marker_size = 3
    marker_size_raw = marker_size+4

    fig = plt.figure()
    fig.suptitle('Measurement noise:{}    Process noise:{} '.format(
        noises[0], noises[1]), fontsize=16)

    plt.subplot(3, 1, 1)
    plt.plot(raw.t, raw.x, linestyle="", marker='x',
             markersize=marker_size_raw, label='raw')
    plt.plot(filtered.t, filtered.x, linestyle="",
             marker='o', markersize=marker_size, label='filtered')
    plt.plot(filtered_accel.t, filtered_accel.x, linestyle="",
             marker='o', markersize=marker_size, label='filtered_accel')

    plt.plot(filtered_jerk.t, filtered_jerk.x, linestyle="",
             marker='o', markersize=marker_size, label='filtered_jerk')

    plt.legend()
    plt.grid()

    plt.title("X")
    plt.xlabel("t")
    plt.ylabel("x")

    plt.subplot(3, 1, 2)
    # plt.figure()
    plt.plot(raw.t, raw.y, linestyle="",
             marker='x', markersize=marker_size_raw, label='raw')
    plt.plot(filtered.t, filtered.y,  # linestyle="",
             marker='o', markersize=marker_size, label='filtered')
    plt.plot(filtered_accel.t, filtered_accel.y,  # linestyle="",
             marker='o', markersize=marker_size, label='filtered_accel')
    plt.plot(filtered_jerk.t, filtered_jerk.y, linestyle="",
             marker='o', markersize=marker_size, label='filtered_jerk')
    plt.legend()
    plt.grid()

    plt.title("Y")
    plt.xlabel("t")
    plt.ylabel("y")

    plt.subplot(3, 1, 3)
    # plt.figure()
    plt.plot(raw.t, raw.z, linestyle="",
             marker='x', markersize=marker_size_raw, label='raw')
    plt.plot(filtered.t, filtered.z, linestyle="",
             marker='o', markersize=marker_size, label='filtered')
    plt.plot(filtered_accel.t, filtered_accel.z, linestyle="",
             marker='o', markersize=marker_size, label='filtered_accel')
    plt.plot(filtered_jerk.t, filtered_jerk.z, linestyle="",
             marker='o', markersize=marker_size, label='filtered_jerk')

    plt.legend()
    plt.grid()

    plt.title("Z")
    plt.xlabel("t")
    plt.ylabel("z")

    # set the spacing between subplots
    plt.subplots_adjust(hspace=0.46)


def plot3D(raw, filtered, filtered_accel, filtered_jerk):
    marker_size = 3
    marker_size_raw = marker_size+4
    # 3d plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(raw.x, raw.y, raw.z, linestyle="",
            marker='x', markersize=marker_size, label='raw')

    ax.plot(filtered.x, filtered.y, filtered.z,
            marker='o', markersize=marker_size, label='filtered')

    ax.plot(filtered_accel.x, filtered_accel.y, filtered_accel.z,
            marker='o', markersize=marker_size, label='filtered_accel')

    ax.plot(filtered_jerk.x, filtered_jerk.y, filtered_jerk.z,
            marker='o', markersize=marker_size, label='filtered_jerk')

    ax.legend()

    ax.set_xlabel('X', fontsize=20)
    ax.set_ylabel('Y', fontsize=20)
    ax.set_zlabel('Z', fontsize=20)


def plot_filter_variables(filtered, filtered_accel, filtered_jerk):
    marker_size = 3
    marker_size_raw = marker_size+4

    fig = plt.figure()
    fig.suptitle('Measurement noise:{}    Process noise:{} '.format(
        noises[0], noises[1]), fontsize=16)

    plt.subplot(3, 1, 1)
    plt.plot(filtered.t, filtered.x, linestyle="",
             marker='o', markersize=marker_size, label='filtered')
    plt.plot(filtered_accel.t, filtered_accel.x, linestyle="",
             marker='o', markersize=marker_size, label='filtered_accel')

    plt.plot(filtered_jerk.t, filtered_jerk.x, linestyle="",
             marker='o', markersize=marker_size, label='filtered_jerk')

    plt.legend()
    plt.grid()

    plt.title("Likelihood")
    plt.xlabel("t")
    plt.ylabel("Likelihood")

    plt.subplot(3, 1, 2)
    plt.plot(filtered.t, filtered.y,  # linestyle="",
             marker='o', markersize=marker_size, label='filtered')
    plt.plot(filtered_accel.t, filtered_accel.y,  # linestyle="",
             marker='o', markersize=marker_size, label='filtered_accel')
    plt.plot(filtered_jerk.t, filtered_jerk.y, linestyle="",
             marker='o', markersize=marker_size, label='filtered_jerk')
    plt.legend()
    plt.grid()

    plt.title("K")
    plt.xlabel("t")
    plt.ylabel("K")

    plt.subplot(3, 1, 3)
    plt.plot(filtered.t, filtered.z, linestyle="",
             marker='o', markersize=marker_size, label='filtered')
    plt.plot(filtered_accel.t, filtered_accel.z, linestyle="",
             marker='o', markersize=marker_size, label='filtered_accel')
    plt.plot(filtered_jerk.t, filtered_jerk.z, linestyle="",
             marker='o', markersize=marker_size, label='filtered_jerk')

    plt.legend()
    plt.grid()

    plt.title("P")
    plt.xlabel("t")
    plt.ylabel("p")

    # set the spacing between subplots
    plt.subplots_adjust(hspace=0.46)


if __name__ == '__main__':
    logs_path = "/home/marios/catkin_ws/src/Drone_Pose_Estimation/logs/"

    noises = np.loadtxt(logs_path+"noises.txt")

    raw = log_data(logs_path+'raw_values.txt')
    filtered = log_data(logs_path+'filtered_values.txt')
    filtered_variables = log_data(logs_path+'filtered_variables.txt')

    filtered_accel = log_data(logs_path+'filtered_values_accel.txt')
    filtered_accel_variables = log_data(
        logs_path+'filtered_variables_accel.txt')

    filtered_jerk = log_data(logs_path+'filtered_values_jerk.txt')
    filtered_jerk_variables = log_data(
        logs_path+'filtered_variables_jerk.txt')

    plot_coordinates(raw, filtered, filtered_accel, filtered_jerk)

    plot3D(raw, filtered, filtered_accel, filtered_jerk)

    plot_filter_variables(filtered_variables,
                          filtered_accel_variables, filtered_jerk_variables)

    plt.show()
