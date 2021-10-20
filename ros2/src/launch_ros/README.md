# launch_ros

The launch_ros ROS package contains example launch files, which will launch all required components for running some of the main ROS SDK functionality. For example running the laser scan publisher, along with rviz (with rviz config) and the laser scan to rviz transform. The launch files have been designed so that running them will provide almost instant visual output.

## launch_ros/launch
The launch folder contains examples of the actual .launch files. These can be run like so:
    ros2 launch launch_laser_scan_publisher.launch.py

## camera_ros/src

The src folder contains the actual cpp and header source files which define the publisher and subscriber examples. The files define the following execeutbales:

### camera_publisher

Contains a basic example of connecting to an RTSP camera, and publishing both camera configuration data and image data.

### camera_subscriber

Contains a basic example of receiving camera configuration data and camera image data, from the published camera topics.

### camera_subscriber_to_video

Contains a basic example of receiving camera configuration data and camera image data, from the published camera topics, and saving them to the local machine in the format of a video file.

### video_capture_manager

Contains some helper code to manage the connection to an RTSP camera.

## CMakeLists

This is the build file which defines how the camera_ros ROS package is built and installed.

## package

This file contains properties of the camera_ros package, such as package name, versions, authors etc.