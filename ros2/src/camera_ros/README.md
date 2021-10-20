# camera_ros

The camera_ros ROS package contains example **publishers and subscribers** which interface to an **RTSP camera**.

## camera_ros/config

The config folder contains an example **configuration YAML file**, which define the basic **settings** needed to to run the package executables.

## camera_ros/src

The src folder contains the actual **cpp and header source files** which define the publisher and subscriber examples. The files define the following execeutbales:

### camera_publisher

Contains a basic example of **connecting to an RTSP camera**, and **publishing** both camera configuration data and image data.

### camera_subscriber

Contains a basic example of **receiving camera configuration data and camera image data**, from the published camera topics.

### camera_subscriber_to_video

Contains a basic example of **receiving camera configuration data and camera image data**, from the published camera topics, and saving them to the local machine as a **video file**.

### video_capture_manager

Contains some **helper code** to manage the **connection to an RTSP camera**.

## CMakeLists

This is the **build file** which defines how the camera_ros ROS package is built and installed.

## package

This file contains **properties** of the camera_ros package, such as package name, versions, authors etc.