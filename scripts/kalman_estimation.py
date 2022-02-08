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


def init_kalman(dt):
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

if __name__ == '__main__':
    # tf subscriber
    rospy.init_node('kalman_estimation', anonymous=True)
    listener = tf.TransformListener()
    tf_br = tf.TransformBroadcaster()

    loop_frequency = 10  # hz
    rate = rospy.Rate(loop_frequency)
    dt = 1/loop_frequency

    f = init_kalman(dt)

    t0 = rospy.get_time()
    lossed_frames = 0
    while not rospy.is_shutdown():
        z = get_measurement()  # returns array [t,x,y,z,qx,qy,qz,qw]

        if z is None:
            lossed_frames += 1
            continue

        dt = z[0]-t0
        f.F = np.array([[1., dt],
                        [0., 1.]])

        f.predict()
        f.update(z[1])

        estimation = f.x
        print("estimation:", estimation)

        pos = [estimation[0], 0, 0]
        tf_br.sendTransform(pos, [0, 0, 0, 1],
                            rospy.Time.now(), "robot_kalman", "world")

        rate.sleep()
