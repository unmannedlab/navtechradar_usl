#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "interfaces/msg/configuration_data_message.hpp"
#include "interfaces/msg/fft_data_message.hpp"
#include "colossus_subscriber.h"

Colossus_subscriber::Colossus_subscriber() : Node{ "colossus_subscriber" }{
    using std::placeholders::_1;

    configuration_data_subscriber = 
        Node::create_subscription<interfaces::msg::ConfigurationDataMessage>(
        "radar_data/configuration_data",
            5,
            std::bind(&Colossus_subscriber::configuration_data_callback, this, _1));
    fft_data_subscriber =
        Node::create_subscription<interfaces::msg::FftDataMessage>(
        "radar_data/fft_data",
            1600,
            std::bind(&Colossus_subscriber::fft_data_callback, this, _1));
}

void Colossus_subscriber::configuration_data_callback(const interfaces::msg::ConfigurationDataMessage::SharedPtr msg) const{
    RCLCPP_INFO(Node::get_logger(), "Configuration Data recieved");
    RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", msg->azimuth_samples);
    RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", msg->encoder_size);
    RCLCPP_INFO(Node::get_logger(), "Bin Size: %i", msg->bin_size);
    RCLCPP_INFO(Node::get_logger(), "Range In Bins: %i", msg->range_in_bins);
    RCLCPP_INFO(Node::get_logger(), "Expected Rotation Rate: %i", msg->expected_rotation_rate);
}
void Colossus_subscriber::fft_data_callback(const interfaces::msg::FftDataMessage::SharedPtr msg) const{
    RCLCPP_INFO(Node::get_logger(), "FFT Data Received");
    RCLCPP_INFO(Node::get_logger(), "Angle: %f", msg->angle);
    RCLCPP_INFO(Node::get_logger(), "Azimuth: %i", msg->azimuth);
    RCLCPP_INFO(Node::get_logger(), "Sweep Counter: %i", msg->sweep_counter);
    RCLCPP_INFO(Node::get_logger(), "NTP Seconds: : %i", msg->ntp_seconds);
    RCLCPP_INFO(Node::get_logger(), "NTP Split Seconds: %i", msg->ntp_split_seconds);
}
