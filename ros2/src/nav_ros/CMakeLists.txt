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
find_package(interfaces REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4.5.3 REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(${PROTOBUF_INCLUDE_DIR})
if(PROTOBUF_FOUND)
    message ("Protobuf found")
else()
    message (FATAL_ERROR "Cannot find Protobuf")
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${OpenCV_INCLUDE_DIRS}
  ../../../cpp_17
  ../../../cpp_17/utility
  ../../../cpp_17/network
  ../../../cpp_17/navigation
)

set(protobuf_files ../../../cpp_17/protobuf/configurationdata.proto ../../../cpp_17/protobuf/discovery.proto ../../../cpp_17/protobuf/health.proto ../../../cpp_17/protobuf/healthinfo.proto ../../../cpp_17/protobuf/networkinfo.proto ../../../cpp_17/protobuf/nvramcontents.proto ../../../cpp_17/protobuf/radarmodel.proto ../../../cpp_17/protobuf/softwareversions.proto)
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${protobuf_files})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

## Add additional CPP libs
add_library(iasdk_static STATIC ../../../cpp_17/navigation/peak_finder.cpp ../../../cpp_17/network/radar_client.cpp ../../../cpp_17/network/tcp_radar_client.cpp ../../../cpp_17/network/tcp_socket.cpp ../../../cpp_17/utility/timer.cpp ../../../cpp_17/utility/threaded_class.cpp ../../../cpp_17/network/colossus_network_message.cpp ../../../cpp_17/utility/ip_address.cpp  ../../../cpp_17/navigation/sector_blanking.cpp ../../../cpp_17/utility/net_conversion.cpp)
add_library(iasdk_protobuf STATIC ${PROTO_SRCS} ${PROTO_HDRS})

target_link_libraries(iasdk_protobuf ${PROTOBUF_LIBRARY})

## Build executable for Colossus test tool
add_executable(colossus_test_tool src/colossus_test_tool_main.cpp src/colossus_test_tool.cpp)
target_link_libraries(colossus_test_tool iasdk_static iasdk_protobuf)
ament_target_dependencies(colossus_test_tool rclcpp std_msgs interfaces)

## Build executable for Colossus publisher
add_executable(colossus_publisher src/colossus_publisher_main.cpp src/colossus_publisher.cpp)
target_link_libraries(colossus_publisher iasdk_static iasdk_protobuf)
ament_target_dependencies(colossus_publisher rclcpp std_msgs interfaces)

## Build executable for Colossus subscriber
add_executable(colossus_subscriber src/colossus_subscriber_main.cpp src/colossus_subscriber.cpp)
target_link_libraries(colossus_subscriber iasdk_static)
ament_target_dependencies(colossus_subscriber rclcpp std_msgs interfaces)

## Build executable for Colossus subscriber to video
add_executable(colossus_subscriber_to_video src/colossus_subscriber_to_video_main.cpp src/colossus_subscriber_to_video.cpp)
target_link_libraries(colossus_subscriber_to_video iasdk_static)
ament_target_dependencies(colossus_subscriber_to_video rclcpp std_msgs interfaces OpenCV)

## Build executable for laser scan publisher
add_executable(laser_scan_publisher src/laser_scan_publisher_main.cpp src/laser_scan_publisher.cpp)
target_link_libraries(laser_scan_publisher iasdk_static iasdk_protobuf)
ament_target_dependencies(laser_scan_publisher rclcpp std_msgs sensor_msgs interfaces)

## Build executable for Laser scan subscriber
add_executable(laser_scan_subscriber src/laser_scan_subscriber_main.cpp src/laser_scan_subscriber.cpp)
target_link_libraries(laser_scan_subscriber iasdk_static)
ament_target_dependencies(laser_scan_subscriber rclcpp std_msgs sensor_msgs interfaces)

## Build executable for Laser scan subscriber to video
add_executable(laser_scan_subscriber_to_video src/laser_scan_subscriber_to_video_main.cpp src/laser_scan_subscriber_to_video.cpp)
target_link_libraries(laser_scan_subscriber_to_video iasdk_static)
ament_target_dependencies(laser_scan_subscriber_to_video rclcpp std_msgs sensor_msgs interfaces OpenCV)

## Build executable for point cloud publisher
add_executable(point_cloud_publisher src/point_cloud_publisher_main.cpp src/point_cloud_publisher.cpp)
target_link_libraries(point_cloud_publisher iasdk_static iasdk_protobuf)
ament_target_dependencies(point_cloud_publisher rclcpp std_msgs sensor_msgs interfaces)

## Build executable for navigation mode point cloud publisher
add_executable(navigation_mode_point_cloud_publisher src/navigation_mode_point_cloud_publisher_main.cpp src/navigation_mode_point_cloud_publisher.cpp)
target_link_libraries(navigation_mode_point_cloud_publisher iasdk_static iasdk_protobuf)
ament_target_dependencies(navigation_mode_point_cloud_publisher rclcpp std_msgs sensor_msgs interfaces)

## Build executable for B scan publisher
add_executable(b_scan_publisher src/b_scan_publisher_main.cpp src/b_scan_publisher.cpp)
target_link_libraries(b_scan_publisher iasdk_static iasdk_protobuf)
ament_target_dependencies(b_scan_publisher rclcpp std_msgs sensor_msgs interfaces OpenCV)

## Install the built executables
install(TARGETS
  colossus_test_tool
  colossus_publisher
  colossus_subscriber
  colossus_subscriber_to_video
  laser_scan_publisher
  laser_scan_subscriber
  laser_scan_subscriber_to_video
  point_cloud_publisher
  navigation_mode_point_cloud_publisher
  b_scan_publisher
  DESTINATION lib/${PROJECT_NAME})

## Install the package
ament_package()