#!/usr/bin/env python
# license removed for brevity
from filterpy.common import Q_discrete_white_noise
from filterpy.kalman import KalmanFilter
import sys
import os
import tf
from math import pi
from visualization_msgs.msg import Marker
from tf import transformations
import rospy
import math
import numpy as np


def get_measurement():
    try:
        (translation, rotation) = listener.lookupTransform(
            'world', 'robot', rospy.Time(0))
        t = rospy.get_time()-t0

        if get_measurement.prev_x == translation[0]:
            return None

        get_measurement.prev_x = translation[0]
        return np.concatenate((np.array([t]), np.array(translation), np.array(rotation)))

    except:
        return None


get_measurement.prev_x = None


def update_F_1D(dt):
    return np.array([[1., dt],
                     [0., 1.]])


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


def update_F_3D_jerk(dt):
    return np.array(
        [
            [1, 0, 0, dt, 0, 0,    dt**2/2, 0,
                0,       dt**3/6, 0,         0],
            [0, 1, 0, 0, dt, 0,    0,       dt**2 /
                2,   0,       0,       dt**3/6,   0],
            [0, 0, 1, 0, 0, dt,    0,       0,
                dt**2/2, 0,       0,         dt**3/6],
            [0, 0, 0, 1, 0, 0,     dt,       0,
                0,      dt**2/2, 0,         0],
            [0, 0, 0, 0, 1, 0,     0,       dt,
                0,      0,       dt**2/2,   0],
            [0, 0, 0, 0, 0, 1,     0,       0,
                dt,      0,       0,         dt**2/2],
            [0, 0, 0, 0, 0, 0,     1,       0,
                0,       dt,      0,         0],
            [0, 0, 0, 0, 0, 0,     0,       1,
                0,       0,       dt,        0],
            [0, 0, 0, 0, 0, 0,     0,       0,
                1,       0,       0,         dt],
            [0, 0, 0, 0, 0, 0,     1,       0,
                0,       1,       0,         0],
            [0, 0, 0, 0, 0, 0,     0,       1,
                0,       0,       1,         0],
            [0, 0, 0, 0, 0, 0,     0,       0,
                1,       0,       0,         1]
        ])


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

    f.Q = np.array([
        [dt**4/4, 0, 0, dt**3/2, 0, 0],
        [0, dt**4/4, 0, 0, dt**3/2, 0],
        [0, 0, dt**4/4, 0, 0, dt**3/2],
        [dt**3/2, 0, 0, dt**2, 0, 0],
        [0, dt**3/2, 0, 0, dt**2, 0],
        [0, 0, dt**3/2, 0, 0, dt**2]
    ])

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

    f.Q = np.array([
        [dt**4/4, 0, 0, dt**3/2, 0, 0, dt**2, 0, 0],
        [0, dt**4/4, 0, 0, dt**3/2, 0, 0, dt**2, 0],
        [0, 0, dt**4/4, 0, 0, dt**3/2, 0, 0, dt**2],
        [dt**3/2, 0, 0, dt**2, 0, 0, dt, 0, 0],
        [0, dt**3/2, 0, 0, dt**2, 0, 0, dt, 0],
        [0, 0, dt**3/2, 0, 0, dt**2, 0, 0, dt],
        [dt**2, 0, 0, dt, 0, 0, 1, 0, 0],
        [0, dt**2, 0, 0, dt, 0, 0, 1, 0],
        [0, 0, dt**2, 0, 0, dt, 0, 0, 1]
    ])

    return f


def init_kalman_3D_jerk(dt):
    """
    state_x=[
        x,y,z,
        x_dot,y_dot,z_dot,
        x_dot_dot,y_dot_dot,z_dot_dot,
        x_dot_dot_dot,y_dot_dot_dot,z_dot_dot_dot
        ]
    """
    f = KalmanFilter(dim_x=4*3, dim_z=3)

    # initial conditions
    f.x = np.array(np.zeros((4*3, 1)))
    f.x[2] = 80

    # state transition matrix
    f.F = update_F_3D_jerk(dt)

    # measurement function
    f.H = np.array([
        [1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
        [0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
        [0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
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

    f.Q = np.eye(12) * 0.001

    return f


def estimate_and_publish(measurement, kalman_obj: KalmanFilter, model_type):
    kalman_obj.predict()
    kalman_obj.update(measurement)
    estimation = kalman_obj.x

    pos = [estimation[0], estimation[1], estimation[2]]
    q = [0, 0, 0, 1]

    if model_type == "vel":
        tf_name = "robot_kalman"
    else:
        tf_name = "robot_kalman" + "_" + model_type

    tf_br.sendTransform(pos, q, rospy.Time.now(), tf_name, "world")
    publish_parameters(kalman_obj, tf_name)


def publish_parameters(kalman_obj: KalmanFilter, tf_name):
    pos = [kalman_obj.log_likelihood, kalman_obj.K[0][0], kalman_obj.P[0, 0]]
    q = [0, 0, 0, 1]

    tf_br.sendTransform(pos, q, rospy.Time.now(),
                        tf_name+"_variables", "world")


if __name__ == '__main__':
    # tf subscriber
    rospy.init_node('kalman_estimation', anonymous=True)
    listener = tf.TransformListener()
    tf_br = tf.TransformBroadcaster()

    loop_frequency = 30  # hz
    rate = rospy.Rate(loop_frequency)
    dt_init = 1/loop_frequency

    # copy dt_init
    dt = dt_init

    f1D = init_kalman_1D(dt)
    f3D = init_kalman_3D(dt)
    f3D_accel = init_kalman_3D_accel(dt)
    f3D_jerk = init_kalman_3D_jerk(dt)

    t0 = rospy.get_time()
    first_measurement_taken = False

    lossed_frames = 0
    while not rospy.is_shutdown():
        z = get_measurement()  # returns array [t,x,y,z,qx,qy,qz,qw]

        if type(z) != type(None) and first_measurement_taken == False:
            first_measurement_taken = True
            t0 = z[0]
        else:
            if not first_measurement_taken:
                continue

        try:
            dt = z[0]-t0
            measurement = z[1:4]
        except:
            measurement = None

        estimate_and_publish(measurement, f3D, "vel")
        estimate_and_publish(measurement, f3D_accel, "accel")
        estimate_and_publish(measurement, f3D_jerk, "jerk")

        rate.sleep()
