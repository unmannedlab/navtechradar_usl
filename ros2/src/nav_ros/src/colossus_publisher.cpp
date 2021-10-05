#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include "interfaces/msg/configuration_data_message.hpp"
#include "interfaces/msg/fft_data_message.hpp"
#include "radar_client.h"
#include "colossus_publisher.h"
#include "net_conversion.h"

Colossus_publisher::Colossus_publisher():Node{ "colossus_publisher" }
{
    declare_parameter("radar_ip", "");
    declare_parameter("radar_port", 0);

    radar_ip = get_parameter("radar_ip").as_string();
    radar_port = get_parameter("radar_port").as_int();

    rclcpp::QoS qos_radar_configuration_publisher(radar_configuration_queue_size);
    qos_radar_configuration_publisher.reliable();

    configuration_data_publisher =
    Node::create_publisher<interfaces::msg::ConfigurationDataMessage>(
        "radar_data/configuration_data",
        qos_radar_configuration_publisher);

    rclcpp::QoS qos_radar_fft_publisher(radar_fft_queue_size);
    qos_radar_fft_publisher.reliable();

    fft_data_publisher =
    Node::create_publisher<interfaces::msg::FftDataMessage>(
        "radar_data/fft_data",
        qos_radar_fft_publisher);
}

void Colossus_publisher::fft_data_handler(const Navtech::Fft_data::Pointer& data)
{
    auto message = interfaces::msg::FftDataMessage();
    message.angle = Navtech::Utility::to_vector(Navtech::Utility::to_uint64_host(data->angle));
    message.azimuth = Navtech::Utility::to_vector(Navtech::Utility::to_uint16_network(data->azimuth));
    message.sweep_counter = Navtech::Utility::to_vector(Navtech::Utility::to_uint16_network(data->sweep_counter));
    message.ntp_seconds = Navtech::Utility::to_vector(Navtech::Utility::to_uint32_network(data->ntp_seconds));
    message.ntp_split_seconds = Navtech::Utility::to_vector(Navtech::Utility::to_uint32_network(data->ntp_split_seconds));
    message.data = data->data;
    message.data_length = Navtech::Utility::to_vector(Navtech::Utility::to_uint16_network(data->data.size()));

    if (data->azimuth < last_azimuth) {
        rotation_count++;
        rotated_once = true;
    }
    last_azimuth = data->azimuth;

    if (rotation_count >= config_publish_count) {
        configuration_data_publisher->publish(config_message);
        rotation_count = 0;
    }

    if (!rotated_once) {
        return;
    }

    fft_data_publisher->publish(message);
}

void Colossus_publisher::configuration_data_handler(const Navtech::Configuration_data::Pointer& data){
    RCLCPP_INFO(Node::get_logger(), "Configuration Data Received");
    RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", data->azimuth_samples);
    RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", data->encoder_size);
    RCLCPP_INFO(Node::get_logger(), "Bin Size: %f", data->bin_size);
    RCLCPP_INFO(Node::get_logger(), "Range In Bins: : %i", data->range_in_bins);
    RCLCPP_INFO(Node::get_logger(), "Expected Rotation Rate: %i", data->expected_rotation_rate);
    RCLCPP_INFO(Node::get_logger(), "Publishing Configuration Data");

    azimuth_samples = data->azimuth_samples;
    config_message.azimuth_samples = Navtech::Utility::to_vector(Navtech::Utility::to_uint16_network(data->azimuth_samples));
    config_message.encoder_size = Navtech::Utility::to_vector(Navtech::Utility::to_uint16_network(data->encoder_size));
    config_message.bin_size = Navtech::Utility::to_vector(Navtech::Utility::to_uint64_host(data->bin_size));
    config_message.range_in_bins = Navtech::Utility::to_vector(Navtech::Utility::to_uint16_network(data->range_in_bins));
    config_message.expected_rotation_rate = Navtech::Utility::to_vector(Navtech::Utility::to_uint16_network(data->expected_rotation_rate));
    configuration_data_publisher->publish(config_message);

    radar_client->start_fft_data();
}