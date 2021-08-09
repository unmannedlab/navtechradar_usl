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

set (Protobuf_USE_STATIC_LIBS ON)

## Find required packages for build
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interfaces REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(OpenCV 4.5.3 REQUIRED)
find_package(Protobuf REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${OpenCV_INCLUDE_DIRS}
  ../../../cpp_17
  ../../../cpp_17/protobuf
  ../../../cpp_17/utility
  ../../../cpp_17/network
  ../../../cpp_17/navigation
  ../../../cpp_17/build/iasdk/protobuf
)

## Add additional CPP libs
add_library(iasdk_static STATIC ../../../cpp_17/network/radar_client.cpp ../../../cpp_17/network/tcp_radar_client.cpp ../../../cpp_17/network/tcp_socket.cpp ../../../cpp_17/utility/timer.cpp ../../../cpp_17/utility/threaded_class.cpp)

## Build executable for Colossus publisher
add_executable(colossus_publisher src/colossus_publisher.cpp)
target_link_libraries(colossus_publisher iasdk_static)
ament_target_dependencies(colossus_publisher rclcpp std_msgs interfaces)

## Build executable for Colossus subscriber
add_executable(colossus_subscriber src/colossus_subscriber_main.cpp src/colossus_subscriber.cpp)
target_link_libraries(colossus_subscriber iasdk_static)
ament_target_dependencies(colossus_subscriber rclcpp std_msgs interfaces)

## Build executable for Colossus subscriber to video
add_executable(colossus_subscriber_to_video src/colossus_subscriber_to_video_main.cpp src/colossus_subscriber_to_video.cpp)
target_link_libraries(colossus_subscriber_to_video iasdk_static)
ament_target_dependencies(colossus_subscriber_to_video rclcpp std_msgs interfaces OpenCV)

## Install the built executables
install(TARGETS
  colossus_publisher
  colossus_subscriber
  colossus_subscriber_to_video
  DESTINATION lib/${PROJECT_NAME})

## Install the package
ament_package()