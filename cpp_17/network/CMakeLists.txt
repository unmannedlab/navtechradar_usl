add_library(
    iasdk_network STATIC 
    radar_client.cpp 
    tcp_radar_client.cpp 
    tcp_socket.cpp 
    colossus_network_message.cpp
)

target_link_libraries(iasdk_network iasdk_utility iasdk_protobuf)
