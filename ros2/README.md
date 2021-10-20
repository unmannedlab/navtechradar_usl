# Navtech ROS2 Driver

The Navtech ROS2 Driver repository contains examples of ROS2 publishers and subscribers, which interface with RTSP cameras and Navtech Radar. Both basic and complete examples are provided which allow simple publishing of data, simple subscribing to data, and also immediate viewing of radar/camera data, using the ROS2 visulaisation tool, RVIZ. The Navtech ROS2 driver is dependent on the Navtech SDK.

Please see lower level README.md files, for more specific information on the ROS project folders.

## Navtech SDK Requirements

### C++ 17

The ROS2 folder contains the ROS2 project files, for the publishers and subscribers, as detailed above.

* C++17 Compiler
Install with the following commands:

    sudo add-apt-repository ppa:ubuntu-toolchain-r/test
	
    sudo apt-get update
	
    sudo apt install gcc-9 gcc-9-base gcc-9-doc g++-9

    sudo apt install libstdc++-9-dev libstdc++-9-doc 

* GCC 9.x and above
Install with the following commands:

    sudo add-apt-repository ppa:ubuntu-toolchain-r/test

    sudo apt update

    sudo apt install gcc-9

* Clang 10 and above
Install with the following command:

    sudo apt-get install clang-10

### Microsoft .NET
* .NET 4.8 and above

## Linux Requirements
To use the shell scripts provided in the 'utility scripts' folder we require bash on Ubuntu. The safest thing to do is execute:

    sudo dpkg-reconfigure -p critical dash

## ROS2 Requirements
As above and:
ROS2 Galactic Geoclone - CPP and Python bindings
Installation instructions can be found here: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Binary.html

## Python Requirements
Python3.0+
sudo apt install python3.8

Numpy 1.17.4
Can be installed with the following command:
sudo apt install python3-numpy

## OpenCV Requirements
OpenCV 4.5.3 - CPP and Python bindings
Installation instructions here: https://www.linuxfromscratch.org/blfs/view/svn/general/opencv.html

## FFMPEG Requirements
ffmpeg version 4.2.4-1ubuntu0.1
Can be installed with the following command:
sudo apt install ffmpeg

## License

The ROS2 driver which is released under The MIT License (MIT).
See file LICENSE.txt or go to <https://opensource.org/licenses/MIT> for full license details.

## ROS2 Packages

To use ROS commands, ROS2 must first be sourced using: source /opt/ros/galactic/setup.bash
Note - this must be done in every new terminal
Alternatively, make this permanent by adding to your bash file: echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc
This will source ROS for every newly opened terminal window

All ROS2 packages must be built and installed before being run
Build with: colcon build
Install with: . install/setup.bash

Packages can be run like so:

    ros2 run <package_name> <executable_name>

For example:

    ros2 run nav_ros colossus_publisher

Packages can be run with their corresponding paramater files like so:

    ros2 run <package_name> <executable_name> --ros-args --params-file <params_file_path>

For example:

    ros2 run nav_ros colossus_publisher --ros-args --params-file ./src/nav_ros/config/colossus_publisher.yaml

## camera_ros

Contains examples of publishers and subscribers to handle the connection to, and delivery of data from, an RTSP video stream.

**See the README.md under 'camera_ros', for more detialed instructions**

## launch_ros

Contains examples of launch files which can be used with the ROS2 Navtech driver.

**See the README.md under 'launch_ros', for more detialed instructions**

## messages

Contains the custom message types used within the ROS2 Navtech driver.

**See the README.md under 'messages', for more detialed instructions**

## nav_ros

Contains examples of publishers and subscribers to handle the connection to, and delivery of data from, an Navtech radar.

**See the README.md under 'nav_ros', for more detialed instructions**

## utility_scripts

Contains helpful utility scripts, for manipulating camera data, radar data and bag files.

**See the README.md under 'utility_scripts', for more detialed instructions**