# Drone-Pose-Estimation

A program that estimates the position and rotation of a flying drone based on the live feed of web-camera on top of the work-space using:

- Aruco Marker
- Computer vision Techniques

In this Branch a Kalman filter with diffrent models has been implemented for better and less noisier tracking.To launch the program use the following command:

`roslaunch Drone_Pose_Estimation raw_estimate_logging.launch`

Keep in mind that before using you have to make sure that you have calibrated the package on your camera and the camera is facing the drone from above where the Aruco tag is placed on it.

![Pose estimation](https://github.com/marios-stam/Drone_Pose_Estimation/blob/d82b5bcb481d162889ba36eb9ecd2105f87f0356/photos/2.jpeg "Pose estimation")
![Pose estimation2](https://github.com/marios-stam/Drone_Pose_Estimation/blob/83deb58fe64a7d7a37af4b42531c5a17a8588c79/photos/3.png "Pose estimation")

I am pllaning to translate the report in English,untill then if you have any question feel free to ask me.
