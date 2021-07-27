add_library(iasdk_network_static STATIC radar_client.cpp tcp_radar_client.cpp tcp_socket.cpp)
add_library(iasdk_network SHARED radar_client.cpp tcp_radar_client.cpp tcp_socket.cpp)
