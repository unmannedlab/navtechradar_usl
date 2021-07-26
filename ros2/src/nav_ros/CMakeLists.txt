## Define CMake minimum required version
cmake_minimum_required(VERSION 3.5)
project(nav_ros)

## Define C++ standard as C++ 17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

## Add compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Find required packages for build
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${colcon_INCLUDE_DIRS}
  ../../../cpp
)

## Add additional CPP libs
add_library(iasdk_static STATIC ../../../cpp/radarclient.cpp ../../../cpp/tcpradarclient.cpp ../../../cpp/tcpsocket.cpp ../../../cpp/timer.cpp ../../../cpp/threadedclass.cpp)

## Build executable for Colossus publisher
add_executable(colossus_publisher src/colossus_publisher.cpp)
ament_target_dependencies(colossus_publisher rclcpp std_msgs)

## Build executable for Colossus subscriber
add_executable(colossus_subscriber src/colossus_subscriber.cpp)
ament_target_dependencies(colossus_subscriber rclcpp std_msgs)

## Install the built executables
install(TARGETS
  colossus_publisher
  colossus_subscriber
  DESTINATION lib/${PROJECT_NAME})

## Install the package
ament_package()