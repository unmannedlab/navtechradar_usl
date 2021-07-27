#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/configuration_data_message.hpp"
#include "interfaces/msg/fft_data_message.hpp"
#include "radarclient.h"

using namespace std;
using namespace std::chrono_literals;
using namespace Navtech;

rclcpp::Publisher<interfaces::msg::ConfigurationDataMessage>::SharedPtr configuration_data_publisher;
rclcpp::Publisher<interfaces::msg::FftDataMessage>::SharedPtr fft_data_publisher;

class Colossus_publisher : public rclcpp::Node
{
public:
    Colossus_publisher() : Node{ "colossus_publisher" }
    {
        configuration_data_publisher = Node::create_publisher<interfaces::msg::ConfigurationDataMessage>("configuration_data", 5);
        fft_data_publisher = Node::create_publisher<interfaces::msg::FftDataMessage>("fft_data", 1600);
    }
};

RadarClientPtr_t radar_client;
std::shared_ptr<Colossus_publisher> node;

void fft_data_handler(const FFTDataPtr_t& data)
{
    RCLCPP_INFO(node->get_logger(), "Publishing FFT Data");

    auto message = interfaces::msg::FftDataMessage();
    message.angle = data->Angle;
    message.azimuth = data->Azimuth;
    message.sweep_counter = data->SweepCounter;
    message.ntp_seconds = data->NTPSeconds;
    message.ntp_split_seconds = data->NTPSplitSeconds;
    message.data = data->Data;

    fft_data_publisher->publish(message);
}

void configuration_data_handler(const ConfigurationDataPtr_t& data)
{
    RCLCPP_INFO(node->get_logger(), "Configuration Data recieved");
    RCLCPP_INFO(node->get_logger(), "Azimuth Samples: %i", data->AzimuthSamples);
    RCLCPP_INFO(node->get_logger(), "Encoder Size: %i", data->EncoderSize);
    RCLCPP_INFO(node->get_logger(), "Bin Size: %i", data->BinSize);
    RCLCPP_INFO(node->get_logger(), "Range In Bins: : %i", data->RangeInBins);
    RCLCPP_INFO(node->get_logger(), "Expected Rotation Rate: %i", data->ExpectedRotationRate);
    RCLCPP_INFO(node->get_logger(), "Publishing Configuration Data");

    auto message = interfaces::msg::ConfigurationDataMessage();
    message.azimuth_samples = data->AzimuthSamples;
    message.encoder_size = data->EncoderSize;
    message.bin_size = data->BinSize;
    message.range_in_bins = data->RangeInBins;
    message.expected_rotation_rate = data->ExpectedRotationRate;
    configuration_data_publisher->publish(message);

    radar_client->StartFFTData();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    node = std::make_shared<Colossus_publisher>();

    RCLCPP_INFO(node->get_logger(), "Starting radar client");
    radar_client = std::make_shared<RadarClient>("10.77.2.210");
    radar_client->SetFFTDataCallback(std::bind(&fft_data_handler, std::placeholders::_1));
    radar_client->SetConfigurationDataCallback(std::bind(&configuration_data_handler, std::placeholders::_1));
    radar_client->Start();

    while (rclcpp::ok())
    {
        rclcpp::spin(node);
    }

    radar_client->StopFFTData();
    radar_client->SetConfigurationDataCallback();
    radar_client->SetFFTDataCallback();
    radar_client->Stop();
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "Stopped radar client");
}
