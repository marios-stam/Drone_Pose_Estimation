#!/usr/bin/env python
# license removed for brevity
import sys
import os
import tf
from math import pi
from visualization_msgs.msg import Marker
from tf import transformations
import rospy
import math
import numpy as np

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise


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
    # making Q from 3x3 to 6x6 matrix
    # Q = Q_discrete_white_noise(dim=3, dt=dt, var=0.1)
    # Q = np.concatenate((Q, Q), axis=0)
    # Q = np.concatenate((Q, Q), axis=1)

    f.Q = np.eye(6) * 0.1

    return f


def update_F_3D(dt):
    return np.array([[1, 0, 0, dt, 0, 0],
                    [0, 1, 0, 0, dt, 0],
                    [0, 0, 1, 0, 0, dt],
                    [0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0, 1]
                     ])


if __name__ == '__main__':
    # tf subscriber
    rospy.init_node('kalman_estimation', anonymous=True)
    listener = tf.TransformListener()
    tf_br = tf.TransformBroadcaster()

    loop_frequency = 20  # hz
    rate = rospy.Rate(loop_frequency)
    dt = 1/loop_frequency

    f1D = init_kalman_1D(dt)
    f3D = init_kalman_3D(dt)

    t0 = rospy.get_time()
    first_measurement_taken = False

    lossed_frames = 0
    while not rospy.is_shutdown():
        z = get_measurement()  # returns array [t,x,y,z,qx,qy,qz,qw]

        if z is None:
            lossed_frames += 1
            continue

        if first_measurement_taken == False:
            first_measurement_taken = True
            t0 = z[0]

        dt = z[0]-t0

        # f1D.F = update_F_1D(dt)
        # f1D.predict()
        # f1D.update(z[1])
        # estimation = f1D.x
        # print("estimation:", estimation)
        # pos = [estimation[0], 0, 0]
        # tf_br.sendTransform(pos, [0, 0, 0, 1],
        #                     rospy.Time.now(), "robot_kalman", "world")

        measurement = z[1:4]
        print(len(measurement))
        f3D.F = update_F_3D(dt)
        f3D.predict()
        f3D.update(measurement)

        estimation = f3D.x
        print("estimation:", estimation)
        pos = [estimation[0], estimation[1], estimation[2]]
        tf_br.sendTransform(pos, [0, 0, 0, 1],
                            rospy.Time.now(), "robot_kalman", "world")

        rate.sleep()
