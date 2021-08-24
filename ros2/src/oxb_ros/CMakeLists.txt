## Define CMake minimum required version
cmake_minimum_required(VERSION 3.5)
project(oxb_ros)

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
find_package(OpenCV 4.5.3 REQUIRED)
find_package(Protobuf REQUIRED)

if(OPENCV_FOUND)
    message ("OpenCV found")
else()
    message (FATAL_ERROR "Cannot find OpenCV")
endif()


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
add_library(iasdk_static STATIC ../../../cpp_17/network/radar_client.cpp ../../../cpp_17/network/tcp_radar_client.cpp ../../../cpp_17/network/tcp_socket.cpp ../../../cpp_17/utility/timer.cpp ../../../cpp_17/utility/threaded_class.cpp ../../../cpp_17/network/colossus_network_message.cpp ../../../cpp_17/network/IP_address.cpp)
add_library(iasdk_protobuf STATIC ${PROTO_SRCS} ${PROTO_HDRS})

target_link_libraries(iasdk_protobuf ${PROTOBUF_LIBRARY})

## Build executable for Colossus and camera publisher
add_executable(colossus_and_camera_publisher src/colossus_and_camera_publisher_main.cpp src/colossus_and_camera_publisher.cpp ../camera_ros/src/video_capture_manager.cpp)
target_link_libraries(colossus_and_camera_publisher iasdk_static iasdk_protobuf)
ament_target_dependencies(colossus_and_camera_publisher rclcpp std_msgs interfaces OpenCV)

## Build executable for Colossus and dual camera publisher
add_executable(colossus_and_dual_camera_publisher src/colossus_and_dual_camera_publisher_main.cpp src/colossus_and_dual_camera_publisher.cpp ../camera_ros/src/video_capture_manager.cpp)
target_link_libraries(colossus_and_dual_camera_publisher iasdk_static iasdk_protobuf)
ament_target_dependencies(colossus_and_dual_camera_publisher rclcpp std_msgs interfaces OpenCV)

## Build executable for Colossus and camera subscriber to video
add_executable(colossus_and_camera_subscriber_to_video src/colossus_and_camera_subscriber_to_video_main.cpp src/colossus_and_camera_subscriber_to_video.cpp)
target_link_libraries(colossus_and_camera_subscriber_to_video iasdk_static iasdk_protobuf)
ament_target_dependencies(colossus_and_camera_subscriber_to_video rclcpp std_msgs interfaces OpenCV)

## Install the built executables
install(TARGETS
  colossus_and_camera_publisher
  colossus_and_dual_camera_publisher
  colossus_and_camera_subscriber_to_video
  DESTINATION lib/${PROJECT_NAME})

## Install the package
ament_package()