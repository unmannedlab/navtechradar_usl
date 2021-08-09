#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "interfaces/msg/configuration_data_message.hpp"
#include "interfaces/msg/fft_data_message.hpp"
#include "radar_client.h"
#include "Pointer_types.h"
#include "colossus_publisher.h"

using namespace std;
using namespace Navtech;
using namespace rclcpp;

namespace {
    Publisher<interfaces::msg::ConfigurationDataMessage>::SharedPtr configuration_data_publisher;
    Publisher<interfaces::msg::FftDataMessage>::SharedPtr fft_data_publisher;
    Owner_of<RadarClient> radar_client{ };
    string radar_ip;
    uint16_t radar_port;
}

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

void Colossus_publisher::fft_data_handler(const FFTDataPtr_t& data){
    //RCLCPP_INFO(Node::get_logger(), "Publishing FFT Data");

    auto message = interfaces::msg::FftDataMessage();
    message.angle = data->Angle;
    message.azimuth = data->Azimuth;
    message.sweep_counter = data->SweepCounter;
    message.ntp_seconds = data->NTPSeconds;
    message.ntp_split_seconds = data->NTPSplitSeconds;
    message.data = data->Data;
    message.data_length = data->Data.size();

    //RCLCPP_INFO(Node::get_logger(), "Data 0: %u", static_cast<int>(data->Data[0]));
    //RCLCPP_INFO(Node::get_logger(), "Data 1: %u", static_cast<int>(data->Data[1]));
    //RCLCPP_INFO(Node::get_logger(), "Data 2: %u", static_cast<int>(data->Data[2]));
    //RCLCPP_INFO(Node::get_logger(), "Data 3: %u", static_cast<int>(data->Data[3]));
    //RCLCPP_INFO(Node::get_logger(), "Data 4: %u", static_cast<int>(data->Data[4]));

    fft_data_publisher->publish(message);
}

void Colossus_publisher::configuration_data_handler(const ConfigurationDataPtr_t& data){
    RCLCPP_INFO(Node::get_logger(), "Configuration Data Received");
    RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", data->AzimuthSamples);
    RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", data->EncoderSize);
    RCLCPP_INFO(Node::get_logger(), "Bin Size: %i", data->BinSize);
    RCLCPP_INFO(Node::get_logger(), "Range In Bins: : %i", data->RangeInBins);
    RCLCPP_INFO(Node::get_logger(), "Expected Rotation Rate: %i", data->ExpectedRotationRate);
    RCLCPP_INFO(Node::get_logger(), "Publishing Configuration Data");

    auto message = interfaces::msg::ConfigurationDataMessage();
    message.azimuth_samples = data->AzimuthSamples;
    message.encoder_size = data->EncoderSize;
    message.bin_size = data->BinSize;
    message.range_in_bins = data->RangeInBins;
    message.expected_rotation_rate = data->ExpectedRotationRate;
    configuration_data_publisher->publish(message);

    radar_client->StartFFTData();
}

std::shared_ptr<Colossus_publisher> node;

int main(int argc, char* argv[]){
    init(argc, argv);
    node = std::make_shared<Colossus_publisher>();

    RCLCPP_INFO(node->get_logger(), "Starting radar client");
    radar_client = allocate_owned<RadarClient>(radar_ip, radar_port);
    radar_client->SetFFTDataCallback(std::bind(&Colossus_publisher::fft_data_handler, node.get(), std::placeholders::_1));
    radar_client->SetConfigurationDataCallback(std::bind(&Colossus_publisher::configuration_data_handler, node.get(), std::placeholders::_1));

    radar_client->Start();

    while (ok()) {
        spin(node);
    }

    radar_client->StopFFTData();
    radar_client->SetConfigurationDataCallback();
    radar_client->SetFFTDataCallback();
    radar_client->Stop();
    shutdown();
    RCLCPP_INFO(node->get_logger(), "Stopped radar client");
}
