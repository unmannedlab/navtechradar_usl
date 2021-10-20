# nav_ros

The nav_ros ROS package contains example **publishers and subscribers** which interface to **Navtech Radar**.

The nav_ros ROS package also contains an example **publisher and subscriber** which interfaces to both a **Navtech Radar and RTSP camera** simultaneously, and in a synchronised mannor.

## nav_ros/config

The config folder contains the example **configuration YAML files**, which define the basic **settings** needed to to run the package executables.

## nav_ros/src

The src folder contains the actual **cpp and header source files** which define the publisher and subscriber examples. The files define the following execeutbales:

### b_scan_publisher

Contains a basic example of **connecting to a Navtech radar**, and **publishing** radar data as a b-scan image. With bins represented as pixels in the horizontal, and azimuths represented as pixels in the vertical.

### colossus_and_camera_publisher

Contains a basic example of **connecting to a Navtech radar and an RTSP camera**, and **publishing** radar fft data and camer image data, in a synchronised mannor.

### colossus_and_camera_subscriber_to_video

Contains a basic example of **subscribing to radar and camera topics**, and **saving** the published topics as video data.

### colossus_publisher

Contains a basic example of **connecting to a Navtech radar**, and **publishing** radar configuration and radar fft data.

### colossus_subscriber

Contains a basic example of **subscribing to radar topics**, and **displaying** basic radar configurration information.

### colossus_subscriber_to_video

Contains a basic example of **subscribing to radar topics**, and **saving** the published topic as video data.

### colossus_test_tool

Contains a basic example of **connecting to a Navtech Radar**, and **testing** teh radar connection, and checking for any packet loss.

### laser_scan_publisher

Contains a basic example of **connecting to a Navtech Radar**, and **publishing** radar data, as a ROS laser scan.

See the ROS laser scan message definition for information about the message type.

### laser_scan_subscriber

Contains a basic example of **subscribing to a ROS laser scan**, and **displaying** basic laser scan information.

### laser_scan_subscriber_to_video

Contains a basic example of **subscribing to a ROS laser scan**, and **saving**  the published topic as video data.

### navigation_mode_point_cloud_publisher

Contains a basic example of **connecting to a Navtech Radar**, running the radar in navigation mode, and **publishing** navigation radar data, as a ROS point cloud.

See the ROS point cloud message definition for information about the message type.

### point_cloud_publisher

Contains a basic example of **connecting to a Navtech Radar**, and **publishing** radar data, as a ROS point cloud.

See the ROS point cloud message definition for information about the message type.

## CMakeLists

This is the **build file** which defines how the nav_ros ROS package is built and installed.

## package

This file contains **properties** of the nav_ros package, such as package name, versions, authors etc.