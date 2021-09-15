# Navtech ROS2 Driver

The Navtech ROS2 Driver contains examples of ROS2 publishers, which connect to and publish data from an RTSP camera, and a Navtech radar. The Driver also contains examples of subscribers which subscribe to the camera and radar topics, and can convert the data from them into video files which are more easily interpreted than thee raw data.

## SDK Requirements

### C++ 17

The ROS2 folder contains the ROS2 project files, for the publishers and subscribers, as detailed above.

* C++17 Compiler
* GCC 9.x and above
* Clang 10 and above

### Microsoft .NET

* .NET 4.8 and above

## Linux Requirements

To use the shell scripts provided in the 'utility scripts' folder we require bash on Ubuntu. The safest thing to do is execute:
sudo dpkg-reconfigure -p critical dash

## License

The ROS2 driver which is released under The MIT License (MIT).
See file LICENSE.txt or go to <https://opensource.org/licenses/MIT> for full license details.

## ROS2 Packages

## Requirements
As above and:
Python3
OpenCV 4.5.3
Numpy 1.17.4

To use ROS commands, ROS2 must first be sourced using: source /opt/ros/galactic/setup.bash
Note - this must be done in every new terminal
Alternatively, make this permanentt by adding to your bash file: echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
This will source ROS for every newly opened terminal window

All ROS2 packages must be built and installed before being run
Build with: colcon build
Install with: . install/setup.bash

Packages can be run like so: ros2 run <package_name> <executable_name>
For example: ros2 run nav_ros colossus_publisher

Packages can be run with their corresponding paramater files like so:
ros2 run <package_name> <executable_name> --ros-args --params-file <params_file_path>
For example:
ros2 run nav_ros colossus_publisher --ros-args --params-file ./src/nav_ros/config/colossus_publisher.yaml

## camera_ros

Contains examples of publishers and subscribers to handle the connection to, and delivery of data from, an RTSP video stream
Also contains a subscriber to consume a published RTSP stream, and to convert it to a video file

## interfaces

Contains the custom message types used within the ROS2 Navtech driver

## launch_ros

Contains examples of launch files which can be used with the ROS2 Navtech driver

## nav_ros

Contains examples of publishers and subscribers to handle the connection to, and delivery of data from, a Navtech radar, using the Navtech IASDK
Also contains a subscriber to consume a published radar fft data stream, and to convert it to a (cartesian coordinate) video file

## oxb_ros

Contains custom publishers and subscribers for a partner company

## utility_scripts

## join_radar_camera_videos

A bash script to scale and stack together (side by side) a camera and a radar video, which have been generated using the scripts above. This code uses ffmpeg commands to achieve the video conversion.

Firstly, allow the script to be executable: sudo chmod +x join_radar_camera_videos

Then run with: ./join_radar_camera_videos

## join_radar_camera_videos.py

A Python script to scale and stack together (side by side) a camera and a radar video, which have been generated using the scripts above. This code uses numpy and OpenCV to achieve the video conversion.

Run like so: python3 join_radar_camera_videos.py

## show_colossus_timestamps

A python script to examine a ROS bag file, and to print out the azimuth, sweep counter, timestamp, and time difference, between radar FFT messages from the bag file.

Run like so: python3 show_colossus_timestamps.py