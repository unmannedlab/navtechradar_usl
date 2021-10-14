#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "interfaces/msg/configuration_data_message.hpp"
#include "interfaces/msg/fft_data_message.hpp"
#include "radar_client.h"
#include "b_scan_publisher.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<B_scan_colossus_publisher>();

    RCLCPP_INFO(node->get_logger(), "Starting radar client");
    node->radar_client = Navtech::allocate_owned<Navtech::Radar_client>(Navtech::Utility::IP_address { node->radar_ip }, node->radar_port);
    node->radar_client->set_fft_data_callback(std::bind(&B_scan_colossus_publisher::fft_data_handler, node.get(), std::placeholders::_1));
    node->radar_client->set_configuration_data_callback(std::bind(&B_scan_colossus_publisher::configuration_data_handler, node.get(), std::placeholders::_1));

    node->radar_client->start();

    while (rclcpp::ok()) {
        spin(node);
    }

    node->radar_client->stop_fft_data();
    node->radar_client->set_configuration_data_callback();
    node->radar_client->set_fft_data_callback();
    node->radar_client->stop();
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "Stopped radar client");
}
