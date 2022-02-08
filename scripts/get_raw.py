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

# from GLOBAL_PARAMETERS import CAMERA_HEIGHT, USB_CAM, USE_ARUCO, CONTOUR_AREA_THRESHOLD
USE_ARUCO = 1
USB_CAM = 0

curr_folder = os.path.dirname(os.path.realpath(__file__))
print("current file:", curr_folder)
sys.path.append(curr_folder)

if USE_ARUCO:
    from drone_pose_estimation import pose_extractor
else:
    from Drone_Pose_Estimation import pose_extractor_CV_techniques


class DroneMarker(Marker):
    def __init__(self, pos=[0, 0, 0], rpy=[0, 0, 0]):
        super().__init__()
        self.header.frame_id = "world"
        self.header.stamp = rospy.get_rostime()
        self.ns = "robot"
        self.id = 0
        self.type = Marker.MESH_RESOURCE
        self.mesh_resource = "package://thesis_drone/resources/meshes/Quadcopter.stl"
        self.action = 0

        self.updatePose(pos, rpy)

        scale_fac = 1/400
        self.scale.x = scale_fac
        self.scale.y = scale_fac
        self.scale.z = scale_fac

        self.color.r = 0.0
        self.color.g = 1.0
        self.color.b = 0.0
        self.color.a = 1.0

        self.lifetime = rospy.Duration(0)

    def updatePose(self, pos, rpy):
        self.pose.position.x = pos[0]
        self.pose.position.y = pos[1]
        self.pose.position.z = pos[2]

        quatern = transformations.quaternion_from_euler(
            rpy[0]-pi/2, -rpy[1], rpy[2])
        # quatern=transformations.quaternion_from_euler(pi/2 ,0, rpy[2])
        self.pose.orientation.x = quatern[0]
        self.pose.orientation.y = quatern[1]
        self.pose.orientation.z = quatern[2]
        self.pose.orientation.w = quatern[3]


def main():
    droneMarkPub = rospy.Publisher('robotMarker', Marker, queue_size=10)
    # tf broadcaster
    tf_br = tf.TransformBroadcaster()

    rospy.init_node('drone_tracker')
    loop_frequency = 20  # hz
    rate = rospy.Rate(loop_frequency)

    translation = (0.0, 0.0, 0.0)
    rotation = (0.0, 0.0, 0.0, 1.0)

    robotMarker = DroneMarker()

    if USE_ARUCO:
        pose_estimator = pose_extractor(USB_cam=USB_CAM)
    else:
        pose_estimator = pose_extractor_CV_techniques(camera_height=CAMERA_HEIGHT,
                                                      contour_area_thresh=CONTOUR_AREA_THRESHOLD, USB_cam=USB_CAM)
        listener = tf.TransformListener()

    while not rospy.is_shutdown():
        if not USE_ARUCO:
            try:
                (trans, rot) = listener.lookupTransform(
                    '/world', 'tello', rospy.Time(0))
                pose_estimator.update_actual_height(trans[2])
                pose_estimator.update_rpy(rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

        tvec, rpy, found_pose = pose_estimator.getPose()

        if found_pose:
            x, y, z = tvec[0][0][0], tvec[0][0][1], tvec[0][0][2]
            scale_div = 1
            # x, y, z = -x/scale_div, -y/scale_div, z/scale_div
            x, y, z = x/scale_div, y/scale_div, z/scale_div

            pos = [x, y, z]

            rpy = (0, math.pi, rpy[2])
            robotMarker.updatePose(pos, rpy)

            # publish tf
            q = transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
            tf_br.sendTransform(pos, q, rospy.Time.now(), "robot", "world")

            droneMarkPub.publish(robotMarker)

        droneMarkPub.publish(robotMarker)
        rate.sleep()


if __name__ == '__main__':
    main()
