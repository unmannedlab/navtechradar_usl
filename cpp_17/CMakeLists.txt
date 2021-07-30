project(iasdk C CXX)

cmake_minimum_required(VERSION 3.16)

set (CMAKE_CXX_FLAGS " -Wall -pthread -std=c++17")
add_definitions(${CMAKE_CXX_FLAGS})

include_directories(${CMAKE_CURRENT_BINARY_DIR}/protobuf)
include_directories(${CMAKE_SOURCE_DIR})

set (Protobuf_USE_STATIC_LIBS ON)
find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})

add_subdirectory(protobuf)
include_directories(protobuf)

add_subdirectory(utility)
include_directories(utility)

add_subdirectory(network)
include_directories(network)

add_subdirectory(navigation)
include_directories(navigation)

add_executable(testclient testclient_main.cpp)
target_link_libraries(testclient iasdk_network iasdk_utility iasdk_protobuf)

add_executable(navigationclient navigation_main.cpp)
target_link_libraries(navigationclient iasdk_network iasdk_utility iasdk_protobuf iasdk_navigation)
