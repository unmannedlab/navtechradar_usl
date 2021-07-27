project(iasdk C CXX)

cmake_minimum_required(VERSION 3.16)

set (CMAKE_CXX_FLAGS " -Wall -Werror -pthread -std=c++17")
add_definitions(${CMAKE_CXX_FLAGS})

add_subdirectory(utility)
include_directories(utility)

add_subdirectory(network)
include_directories(network)

add_executable(testclient_static testclient_main.cpp)
target_link_libraries(testclient_static iasdk_network_static iasdk_utility_static)

add_executable(testclient_dynamic testclient_main.cpp)
target_link_libraries(testclient_dynamic iasdk_network iasdk_utility)
