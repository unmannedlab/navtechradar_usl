#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <math.h>

#include "interfaces/msg/configuration_data_message.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "laser_scan_subscriber.h"

Laser_scan_subscriber::Laser_scan_subscriber() : Node{ "laser_scan_subscriber" }
{
    using std::placeholders::_1;

    rclcpp::QoS qos_radar_configuration_subscriber(radar_configuration_queue_size);
    qos_radar_configuration_subscriber.reliable();

    configuration_data_subscriber = 
    Node::create_subscription<interfaces::msg::ConfigurationDataMessage>(
    "radar_data/configuration_data",
    qos_radar_configuration_subscriber,
    std::bind(&Laser_scan_subscriber::configuration_data_callback, this, _1));

    rclcpp::QoS qos_radar_laser_scan_subscriber(radar_laser_scan_queue_size);
    qos_radar_laser_scan_subscriber.reliable();

    laser_scan_subscriber =
    Node::create_subscription<sensor_msgs::msg::LaserScan>(
    "radar_data/laser_scan",
    qos_radar_laser_scan_subscriber,
    std::bind(&Laser_scan_subscriber::laser_scan_callback, this, _1));
}

void Laser_scan_subscriber::configuration_data_callback(const interfaces::msg::ConfigurationDataMessage::SharedPtr msg) const
{
    RCLCPP_INFO(Node::get_logger(), "Configuration Data recieved");
    RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", msg->azimuth_samples);
    RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", msg->encoder_size);
    RCLCPP_INFO(Node::get_logger(), "Bin Size: %f", msg->bin_size);
    RCLCPP_INFO(Node::get_logger(), "Range In Bins: %i", msg->range_in_bins);
    RCLCPP_INFO(Node::get_logger(), "Expected Rotation Rate: %i", msg->expected_rotation_rate);
}

void Laser_scan_subscriber::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
{
    RCLCPP_INFO(Node::get_logger(), "Laser Scan Received");
    time_t epoch = msg->header.stamp.sec;
    RCLCPP_INFO(Node::get_logger(), "Timestamp: %s", asctime(gmtime(&epoch)));
    RCLCPP_INFO(Node::get_logger(), "Start angle: %f", msg->angle_min * (180 / M_PI));
    RCLCPP_INFO(Node::get_logger(), "End angle: %f", msg->angle_max * (180 / M_PI));
    RCLCPP_INFO(Node::get_logger(), "Angle increment: %f", msg->angle_increment * (180 / M_PI));
    RCLCPP_INFO(Node::get_logger(), "Ranges: %li", msg->ranges.size());
    RCLCPP_INFO(Node::get_logger(), "Intensities: %li", msg->intensities.size());
}
