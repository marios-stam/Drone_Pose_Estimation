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


if __name__ == '__main__':
    # tf subscriber
    rospy.init_node('kalman_estimation', anonymous=True)
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (translation, rotation) = listener.lookupTransform(
                'world', 'robot', rospy.Time(0))

            print(translation)
        except:
            continue
