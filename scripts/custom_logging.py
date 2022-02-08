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


def lookup_transform(tf_name, measurements, i):
    if i >= 100:
        return measurements, i

    try:

        (translation, rotation) = listener.lookupTransform(
            'world', tf_name, rospy.Time(0))
        print("mlkia2")
        measurements[i, 0] = rospy.get_time()-t0
        measurements[i, 1:4] = translation
        measurements[i, 4:9] = rotation

        i += 1

        return measurements, i

    except:
        return measurements, i


if __name__ == '__main__':
    # tf subscriber
    rospy.init_node('kalman_estimation', anonymous=True)
    listener = tf.TransformListener()
    rate = rospy.Rate(20.0)

    # matrix cols: t, x, y, z, qx, qy, qz, qw
    measurements_number = 100
    log_raw, i_raw = np.zeros((measurements_number, 8)), 0
    log_filtered, i_filtered = np.zeros((measurements_number, 8)), 0

    i = 0
    t0 = rospy.get_time()
    while not rospy.is_shutdown():
        log_raw, i_raw = lookup_transform('robot', log_raw, i_raw)
        log_filtered, i_filtered = lookup_transform(
            'robot_kalman', log_filtered, i_filtered)

        print("i_raw: ", i_raw, "i_filtered: ", i_filtered)

        if i_raw > measurements_number and i_filtered > measurements_number:
            break

        rate.sleep()

    # save measurements
    logs_path = "/home/marios/catkin_ws/src/Drone_Pose_Estimation/logs/"
    np.savetxt(logs_path + 'raw_values.txt', log_raw)
    np.savetxt(logs_path + 'filtered_values.txt', log_filtered)
