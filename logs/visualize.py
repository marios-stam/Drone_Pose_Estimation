import numpy as np
import matplotlib.pyplot as plt


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


if __name__ == '__main__':
    logs_path = "/home/marios/catkin_ws/src/Drone_Pose_Estimation/logs/"
    raw = log_data(logs_path+'raw_values.txt')
    filtered = log_data(logs_path+'filtered_values.txt')

    # z_mean = np.mean(z)
    # z_std = z.std()
    # print("z_mean:", z_mean)
    # print("z_std:", z_std)

    fig = plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(raw.t, raw.x)
    plt.plot(filtered.t, filtered.x)
    plt.legend(["raw", "filtered"])
    plt.grid()

    plt.title("X")
    plt.xlabel("t")
    plt.ylabel("x")

    plt.subplot(3, 1, 2)
    plt.plot(raw.t, raw.y)
    plt.plot(filtered.t, filtered.y)
    plt.legend(["raw", "filtered"])
    plt.grid()

    plt.title("Y")
    plt.xlabel("t")
    plt.ylabel("y")

    plt.subplot(3, 1, 3)
    plt.plot(raw.t, raw.z)
    plt.plot(filtered.t, filtered.z)
    plt.legend(["raw", "filtered"])
    plt.grid()

    plt.title("Z")
    plt.xlabel("t")
    plt.ylabel("z")

    # set the spacing between subplots
    plt.subplots_adjust(hspace=0.46)

    plt.show()
