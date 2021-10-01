#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <exception>
#include <cassert>

#include "interfaces/msg/configuration_data_message.hpp"
#include "interfaces/msg/fft_data_message.hpp"
#include "colossus_subscriber.h"
#include "net_conversion.h"

using namespace Navtech::Utility;

Colossus_subscriber::Colossus_subscriber() : Node{ "colossus_subscriber" }
{
    using std::placeholders::_1;

    rclcpp::QoS qos_radar_configuration_subscriber(radar_configuration_queue_size);
    qos_radar_configuration_subscriber.reliable();

    configuration_data_subscriber = 
    Node::create_subscription<interfaces::msg::ConfigurationDataMessage>(
        "radar_data/configuration_data",
        qos_radar_configuration_subscriber,
        std::bind(&Colossus_subscriber::configuration_data_callback, this, _1));

    rclcpp::QoS qos_radar_fft_subscriber(radar_fft_queue_size);
    qos_radar_fft_subscriber.reliable();

    fft_data_subscriber =
    Node::create_subscription<interfaces::msg::FftDataMessage>(
        "radar_data/fft_data",
        qos_radar_fft_subscriber,
        std::bind(&Colossus_subscriber::fft_data_callback, this, _1));
}

void Colossus_subscriber::configuration_data_callback(const interfaces::msg::ConfigurationDataMessage::SharedPtr msg) const
{
    //RCLCPP_INFO(Node::get_logger(), "Configuration Data recieved");

    //try {
    //    auto val_16 = from_vector_to<uint16_t>(msg->azimuth_samples);

    //    if (val_16.has_value()) {
    //        std::cout << "val_16 value - " << val_16.value() << std::endl;
    //    }
    //    else {
    //        std::cout << "val_16 has no value!" << std::endl;
    //    }

    //    // RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", msg->azimuth_samples);
    //}
    //catch (std::exception& e) {
    //    std::cout << e.what() << std::endl;
    //}
    // 
    //RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", msg->encoder_size);
    //RCLCPP_INFO(Node::get_logger(), "Bin Size: %f", msg->bin_size);
    //RCLCPP_INFO(Node::get_logger(), "Range In Bins: %i", msg->range_in_bins);
    //RCLCPP_INFO(Node::get_logger(), "Expected Rotation Rate: %i", msg->expected_rotation_rate);
}

void Colossus_subscriber::fft_data_callback(const interfaces::msg::FftDataMessage::SharedPtr msg) const
{
    RCLCPP_INFO(Node::get_logger(), "FFT Data Received");
    //RCLCPP_INFO(Node::get_logger(), "Angle: %f", from_uint32_host(*reinterpret_cast<const uint32_t*>(&msg->angle[0])));

    auto angle = from_vector_to<uint64_t>(msg->angle);
    if (angle.has_value()) {
        RCLCPP_INFO(Node::get_logger(), "Angle: %f", from_uint64_host(angle.value()));
    }

    auto azimuth = from_vector_to<uint16_t>(msg->azimuth);
    if (azimuth.has_value()) {
        RCLCPP_INFO(Node::get_logger(), "Azimuth: %i", to_uint16_host(azimuth.value()));
    }
   
    //RCLCPP_INFO(Node::get_logger(), "Azimuth: %i", to_uint16_host(from_vector_to<uint16_t>(msg->azimuth)));
    //RCLCPP_INFO(Node::get_logger(), "Sweep Counter: %i", to_uint16_host(*reinterpret_cast<const uint16_t*>(&msg->sweep_counter[0])));
    //RCLCPP_INFO(Node::get_logger(), "NTP Seconds: : %i", to_uint32_host(*reinterpret_cast<const uint32_t*>(&msg->ntp_seconds[0])));
    //RCLCPP_INFO(Node::get_logger(), "NTP Split Seconds: %i", to_uint32_host(*reinterpret_cast<const uint32_t*>(&msg->ntp_split_seconds[0])));
}
