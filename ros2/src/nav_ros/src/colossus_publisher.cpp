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

Publisher<interfaces::msg::ConfigurationDataMessage>::SharedPtr configuration_data_publisher;
Publisher<interfaces::msg::FftDataMessage>::SharedPtr fft_data_publisher;
extern std::shared_ptr<Radar_client> radar_client;
extern string radar_ip;
extern uint16_t radar_port;
extern std::shared_ptr<Colossus_publisher> node;

Colossus_publisher::Colossus_publisher():Node{ "colossus_publisher" }{
    declare_parameter("radar_ip", "");
    declare_parameter("radar_port", 0);

    radar_ip = get_parameter("radar_ip").as_string();
    radar_port = get_parameter("radar_port").as_int();

    configuration_data_publisher =
        Node::create_publisher<interfaces::msg::ConfigurationDataMessage>(
            "radar_data/configuration_data",
            5);
    fft_data_publisher =
        Node::create_publisher<interfaces::msg::FftDataMessage>(
            "radar_data/fft_data",
            1600);
}

void Colossus_publisher::fft_data_handler(const Fft_data::Pointer& data){
    //RCLCPP_INFO(Node::get_logger(), "Publishing FFT Data");

    auto message = interfaces::msg::FftDataMessage();
    message.angle = data->angle;
    message.azimuth = data->azimuth;
    message.sweep_counter = data->sweep_counter;
    message.ntp_seconds = data->ntp_seconds;
    message.ntp_split_seconds = data->ntp_split_seconds;
    message.data = data->data;
    message.data_length = data->data.size();

    //RCLCPP_INFO(Node::get_logger(), "Data 0: %u", static_cast<int>(data->Data[0]));
    //RCLCPP_INFO(Node::get_logger(), "Data 1: %u", static_cast<int>(data->Data[1]));
    //RCLCPP_INFO(Node::get_logger(), "Data 2: %u", static_cast<int>(data->Data[2]));
    //RCLCPP_INFO(Node::get_logger(), "Data 3: %u", static_cast<int>(data->Data[3]));
    //RCLCPP_INFO(Node::get_logger(), "Data 4: %u", static_cast<int>(data->Data[4]));

    fft_data_publisher->publish(message);
}

void Colossus_publisher::configuration_data_handler(const Configuration_data::Pointer& data){
    RCLCPP_INFO(Node::get_logger(), "Configuration Data Received");
    RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", data->azimuth_samples);
    RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", data->encoder_size);
    RCLCPP_INFO(Node::get_logger(), "Bin Size: %f", data->bin_size);
    RCLCPP_INFO(Node::get_logger(), "Range In Bins: : %i", data->range_in_bins);
    RCLCPP_INFO(Node::get_logger(), "Expected Rotation Rate: %i", data->expected_rotation_rate);
    RCLCPP_INFO(Node::get_logger(), "Publishing Configuration Data");

    auto message = interfaces::msg::ConfigurationDataMessage();
    message.azimuth_samples = data->azimuth_samples;
    message.encoder_size = data->encoder_size;
    message.bin_size = data->bin_size;
    message.range_in_bins = data->range_in_bins;
    message.expected_rotation_rate = data->expected_rotation_rate;
    configuration_data_publisher->publish(message);

    radar_client->start_fft_data();
}