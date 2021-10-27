#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "messages/msg/radar_configuration_message.hpp"
#include "radar_client.h"
#include "laser_scan_publisher.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Laser_scan_publisher>();

    RCLCPP_INFO(node->get_logger(), "Starting radar client");
    node->set_radar_client(Navtech::allocate_owned<Navtech::Radar_client>(Navtech::Utility::IP_address { node->get_radar_ip() }, node->get_radar_port()));
    node->get_radar_client()->set_fft_data_callback(std::bind(&Laser_scan_publisher::fft_data_handler, node.get(), std::placeholders::_1));
    node->get_radar_client()->set_configuration_data_callback(std::bind(&Laser_scan_publisher::configuration_data_handler, node.get(), std::placeholders::_1));

    node->get_radar_client()->start();

    while (rclcpp::ok()) {
        spin(node);
    }

    node->get_radar_client()->stop_fft_data();
    node->get_radar_client()->set_configuration_data_callback();
    node->get_radar_client()->set_fft_data_callback();
    node->get_radar_client()->stop();
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "Stopped radar client");
}
