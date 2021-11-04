#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "messages/msg/radar_configuration_message.hpp"
#include "radar_client.h"
#include "navigation/peak_finder.h"
#include "navigation_mode_point_cloud_publisher.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Navigation_mode_point_cloud_publisher>();

    RCLCPP_INFO(node->get_logger(), "Starting radar client");
    node->set_peak_finder(Navtech::allocate_owned<Navtech::Peak_finder>());
    node->get_peak_finder()->set_target_callback(std::bind(&Navigation_mode_point_cloud_publisher::target_data_handler, node.get(), std::placeholders::_1));

    node->set_radar_client(Navtech::allocate_owned<Navtech::Radar_client>(Navtech::Utility::IP_address{ node->get_radar_ip() }, node->get_radar_port()));
    node->get_radar_client()->set_fft_data_callback(std::bind(&Navigation_mode_point_cloud_publisher::fft_data_handler, node.get(), std::placeholders::_1));
    node->get_radar_client()->set_configuration_data_callback(std::bind(&Navigation_mode_point_cloud_publisher::configuration_data_handler, node.get(), std::placeholders::_1, std::placeholders::_2));
    node->get_radar_client()->set_navigation_config_callback(std::bind(&Navigation_mode_point_cloud_publisher::navigation_config_data_handler, node.get(), std::placeholders::_1));
    node->get_radar_client()->set_navigation_data_callback(std::bind(&Navigation_mode_point_cloud_publisher::navigation_data_handler, node.get(), std::placeholders::_1));

    node->get_radar_client()->start();

    while (rclcpp::ok()) {
        spin(node);
    }

    node->get_radar_client()->stop_navigation_data();
    node->get_peak_finder()->set_target_callback();
    node->get_radar_client()->set_fft_data_callback();
    node->get_radar_client()->set_configuration_data_callback();
    node->get_radar_client()->set_navigation_data_callback();
    node->get_radar_client()->stop();
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "Stopped radar client");
}
