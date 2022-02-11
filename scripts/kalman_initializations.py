import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise


def init_kalman_1D(dt):
    # init Kalman Params
    f = KalmanFilter(dim_x=2, dim_z=1)
    f.x = np.array([[2.],    # position
                    [0.]])   # velocity

    # state transition matrix
    f.F = np.array([[1., dt],
                    [0., 1.]])
    # measurement function
    f.H = np.array([[1., 0.]])

    # covariance matrix
    f.P *= 1

    #  measurement noise
    f.R *= 0.1

    # process noise
    f.Q = Q_discrete_white_noise(dim=2, dt=dt, var=0.1)

    return f


def update_F_1D(dt):
    return np.array([[1., dt],
                     [0., 1.]])


def init_kalman_3D(dt):
    f = KalmanFilter(dim_x=6, dim_z=3)

    # initial conditions
    f.x = np.array(np.zeros((6, 1)))
    f.x[2] = 80

    # state transition matrix
    f.F = update_F_3D(dt)

    # measurement function
    f.H = np.array([
        [1., 0., 0., 0., 0., 0.],
        [0., 1., 0., 0., 0., 0.],
        [0., 0., 1., 0., 0., 0.]
    ])

    # covariance matrix
    f.P *= 1

    #  measurement noise
    f.R = np.eye(3) * 0.1

    print("f.R.shape", f.R.shape)

    # process noise
    # making Q from 2x2(for each dimension) to 6x6 matrix
    Q = np.zeros((6, 6))

    Qi = Q_discrete_white_noise(dim=2, dt=dt, var=0.1)
    Q[0, 0:2] = Qi[0, :]
    Q[1, 2:4] = Qi[0, :]
    Q[2, 4:6] = Qi[0, :]

    Q[3, 0:2] = Qi[1, :]
    Q[4, 2:4] = Qi[1, :]
    Q[5, 4:6] = Qi[1, :]

    f.Q = np.eye(6) * 0.1

    return f


def init_kalman_3D_accel(dt):
    """
    state_x=[
        x,y,z,
        x_dot,y_dot,z_dot,
        x_dot_dot,y_dot_dot,z_dot_dot
        ]
    """
    f = KalmanFilter(dim_x=3*3, dim_z=3)

    # initial conditions
    f.x = np.array(np.zeros((3*3, 1)))
    f.x[2] = 80

    # state transition matrix
    f.F = update_F_3D_accel(dt)

    # measurement function
    f.H = np.array([
        [1., 0., 0., 0., 0., 0., 0., 0., 0.],
        [0., 1., 0., 0., 0., 0., 0., 0., 0.],
        [0., 0., 1., 0., 0., 0., 0., 0., 0.]
    ])

    # covariance matrix
    f.P *= 1

    #  measurement noise
    f.R = np.eye(3) * 1

    print("f.R.shape", f.R.shape)

    # process noise
    # making Q from 2x2(for each dimension) to 6x6 matrix
    Q = np.zeros((6, 6))

    Qi = Q_discrete_white_noise(dim=2, dt=dt, var=0.1)
    Q[0, 0:2] = Qi[0, :]
    Q[1, 2:4] = Qi[0, :]
    Q[2, 4:6] = Qi[0, :]

    Q[3, 0:2] = Qi[1, :]
    Q[4, 2:4] = Qi[1, :]
    Q[5, 4:6] = Qi[1, :]

    f.Q = np.eye(9) * 0.1

    return f


def update_F_3D(dt):
    return np.array([[1, 0, 0, dt, 0, 0],
                    [0, 1, 0, 0, dt, 0],
                    [0, 0, 1, 0, 0, dt],
                    [0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0, 1]
                     ])


def update_F_3D_accel(dt):
    return np.array([
        [1, 0, 0, dt, 0, 0,    dt**2/2, 0,         0],
        [0, 1, 0, 0, dt, 0,    0,       dt**2/2,   0],
        [0, 0, 1, 0, 0, dt,    0,       0,         dt**2/2],
        [0, 0, 0, 1, 0, 0,     dt,       0,         0],
        [0, 0, 0, 0, 1, 0,     0,       dt,         0],
        [0, 0, 0, 0, 0, 1,     0,       0,         dt],
        [0, 0, 0, 0, 0, 0,     1,       0,         0],
        [0, 0, 0, 0, 0, 0,     0,       1,         0],
        [0, 0, 0, 0, 0, 0,     0,       0,         1]
    ])
