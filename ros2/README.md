# Navtech ROS2 Driver

The Navtech ROS2 Driver repository contains examples of ROS2 publishers and subscribers, which interface with RTSP cameras and Navtech Radar. Both basic and complete examples are provided which allow simple publishing of data, simple subscribing to data, and also immediate viewing of radar/camera data, using the ROS2 visulaisation tool, RVIZ. The Navtech ROS2 driver is dependent on the Navtech SDK.

Please see lower level README.md files, for more specific information on the ROS project folders.

## Navtech SDK Requirements

### C++ 17

The ROS2 folder contains the ROS2 project files, for the publishers and subscribers, as detailed above.

* C++17 Compiler
Install with the following commands:

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test

sudo apt-get update

sudo apt install gcc-9 gcc-9-base gcc-9-doc g++-9

sudo apt install libstdc++-9-dev libstdc++-9-doc
```

* GCC 9.x and above
Install with the following commands:

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test

sudo apt update

sudo apt install gcc-9
```

* Clang 10 and above
Install with the following command:

```bash
sudo apt-get install clang-10
```

### Microsoft .NET
* .NET 4.8 and above

## Linux Requirements
To use the shell scripts provided in the 'utility scripts' folder we require bash on Ubuntu. First you must execute:
```bash
sudo dpkg-reconfigure -p critical dash
```

## ROS2 Requirements
As above and:
ROS2 Galactic Geoclone - CPP and Python bindings
Installation instructions can be found here: https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Binary.html

## Python Requirements
Python3.0+

```bash
sudo apt install python3.8
```

Numpy 1.17.4
Can be installed with the following command:

```bash
sudo apt install python3-numpy
```

## OpenCV Requirements
OpenCV 4.5.3 - CPP and Python bindings
Installation instructions here: https://www.linuxfromscratch.org/blfs/view/svn/general/opencv.html

## FFMPEG Requirements
ffmpeg version 4.2.4-1ubuntu0.1
Can be installed with the following command:

```bash
sudo apt install ffmpeg
```

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
Install with: 

Packages can be run like so:

```bash
ros2 run <package_name> <executable_name>
```

For example:

```bash
ros2 run nav_ros colossus_publisher
```

Packages can be run with their corresponding paramater files like so:

```bash
ros2 run <package_name> <executable_name> --ros-args --params-file <params_file_path>
```

For example:

```bash
ros2 run nav_ros colossus_publisher --ros-args --params-file ./src/nav_ros/config/colossus_publisher.yaml
```

## camera_ros

Contains examples of publishers and subscribers to handle the connection to, and delivery of data from, an RTSP video stream.

**See the README.md under 'camera_ros', for more detailed instructions**

## launch_ros

Contains examples of launch files which can be used with the ROS2 Navtech driver.

**See the README.md under 'launch_ros', for more detailed instructions**

## messages

Contains the custom message types used within the ROS2 Navtech driver.

**See the README.md under 'messages', for more detailed instructions**

## nav_ros

Contains examples of publishers and subscribers to handle the connection to, and delivery of data from, an Navtech radar.

**See the README.md under 'nav_ros', for more detailed instructions**



# ROS2 Example install procedure

## Update Ubuntu 20.04

```bash
sudo apt update

sudo apt upgrade
```
	
## Install IASDK prerequisites

```bash
sudo apt install build-essential clang g++ protobuf-compiler libprotobuf-dev cmake
```
	
## Install ROS2 prerequisites

```bash
sudo apt install -y build-essential libssl-dev libffi-dev python3-dev python3 python3-pip software-properties-common
```

## Install ROS2

```bash
sudo apt update && sudo apt install curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu 

$(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt install ros-galactic-desktop

source /opt/ros/galactic/setup.bash

ros2 run demo_nodes_cpp talker
```
	
## Check install by running examples

```bash
source /opt/ros/galactic/setup.bash

ros2 run demo_nodes_py listener
```

check that the talker and listener are connected

## Install OpenCV

```bash
sudo apt install git

git clone https://github.com/opencv/opencv.git

git clone https://github.com/opencv/opencv_contrib.git

sudo apt install build-essential cmake git pkg-config libpng-dev libtiff-dev gfortran openexr libgtk-3-dev libavcodec-dev libgstreamer-plugins-base1.0-dev libgstreamer1.0-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libjpeg-dev libatlas-base-dev python3-dev python3-numpy libtbb2 libtbb-dev libdc1394-22-dev libopenexr-devls

cd opencv

mkdir build

cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE    -D CMAKE_INSTALL_PREFIX=/usr/local          -D WITH_CUDA=OFF        -D INSTALL_PYTHON_EXAMPLES=ON          -D OPENCV_GENERATE_PKGCONFIG=ON         -D  OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules  -D OPENCV_ENABLE_NONFREE=ON         -D BUILD_EXAMPLES=ON ..

make -j8  // (replace the 8 by the number of usable cores on your machine)

sudo make install

sudo apt install python3-opencv libopencv-dev
```

## Check OpenCV is installed

```bash
python3 -c "import cv2; print(cv2.__version__)"^C
```

check that version >=4.5.3 is reported
	
```bash
pkg-config --modversion opencv4
```
	
check that version >=4.5.3 is reported
	
## Install the Navtech Radar IASDK

```bash
git clone https://bitbucket.org/navtechradar/iasdk-public.git
```

## Install Colcon (the ROS2 build tool)

```bash
sudo apt install python3-colcon-common-extensions

cd ~/iasdk/cpp_17

colcon build
```

Check the above command does not produce any errors


## Build the ROS2 IASDK pacakges

```bash
cd ~/iasdk/ros2/

source /opt/ros/galactic/setup.bash

colcon build

. install/setup.bash
```

**Note - The following commands must be run in each new terminal opened, or add to bashrc**

```bash
source /opt/ros/galactic/setup.bash

. install/setup.bash
```