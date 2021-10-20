# launch_ros

The launch_ros ROS package contains example **launch files**, which will **launch all required components** for running some of the main ROS SDK functionality. For example running the laser scan publisher, along with rviz (with rviz config) and the laser scan to rviz transform. The launch files have been designed so that running them will provide almost instant visual output.

## launch_ros/launch
The launch folder contains examples of the **actual .launch files**. These can be run (from the launch directory) like so:

    ros2 launch launch_b_scan_publisher.launch.py

### launch_b_scan_publisher.launch

A launch file which will start the **b_scan_publisher** executable, along with a **static transform**, and **RVIZ visualisation**

### launch_camera_publisher.launch

A launch file which will start the **camera_publisher** executable, along with a **static transform**, and **RVIZ visualisation**

### launch_colossus_and_camera_publisher.launch

A launch file which will start the **colossus_and_camera_publisher** executable, along with a **static transform**, and **RVIZ visualisation**

### launch_colossus_publisher.launch

A launch file which will start the **colossus_publisher** executable, along with a **static transform**, and **RVIZ visualisation**

### launch_laser_scan_publisher.launch

A launch file which will start the **laser_scan_publisher** executable, along with a **static transform**, and **RVIZ visualisation**

### launch_navigation_mode_point_cloud_publisher.launch

A launch file which will start the **navigation_mode_point_cloud_publisher** executable, along with a **static transform**, and **RVIZ visualisation**

### launch_point_cloud_publisher.launch

A launch file which will start the **point_cloud_publisher** executable, along with a **static transform**, and **RVIZ visualisation**

## CMakeLists

This is the build file which defines how the launch_ros ROS package is built and installed.

## package

This file contains properties of the launch_ros package, such as package name, versions, authors etc.