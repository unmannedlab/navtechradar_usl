#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "interfaces/msg/configuration_data_message.hpp"
#include "interfaces/msg/fft_data_message.hpp"
#include "radar_client.h"
#include "colossus_publisher.h"

using namespace std;
using namespace Navtech;
using namespace rclcpp;

int main(int argc, char* argv[]){
    init(argc, argv);
    node = std::make_shared<Colossus_publisher>();

    RCLCPP_INFO(node->get_logger(), "Starting radar client");
    radar_client = allocate_owned<Radar_client>(radar_ip, radar_port);
    radar_client->set_fft_data_callback(std::bind(&Colossus_publisher::fft_data_handler, node.get(), std::placeholders::_1));
    radar_client->set_configuration_data_callback(std::bind(&Colossus_publisher::configuration_data_handler, node.get(), std::placeholders::_1));

    radar_client->start();

    while (ok()) {
        spin(node);
    }

    radar_client->stop_fft_data();
    radar_client->set_configuration_data_callback();
    radar_client->set_fft_data_callback();
    radar_client->stop();
    shutdown();
    RCLCPP_INFO(node->get_logger(), "Stopped radar client");
}
