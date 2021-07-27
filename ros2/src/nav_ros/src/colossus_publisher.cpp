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

rclcpp::Publisher<interfaces::msg::ConfigurationDataMessage>::SharedPtr Configuration_data_publisher;
rclcpp::Publisher<interfaces::msg::FftDataMessage>::SharedPtr Fft_data_publisher;

class Colossus_publisher : public rclcpp::Node
{
public:
    Colossus_publisher() : Node{ "colossus_publisher" }
    {
        Configuration_data_publisher = Node::create_publisher<interfaces::msg::ConfigurationDataMessage>("configuration_data", 5);
        Fft_data_publisher = Node::create_publisher<interfaces::msg::FftDataMessage>("fft_data", 1600);
    }
};

RadarClientPtr_t radarClient;
std::shared_ptr<Colossus_publisher> node;

void FFTDataHandler(const FFTDataPtr_t& data)
{
    RCLCPP_INFO(node->get_logger(), "Publishing FFT Data");

    auto message = interfaces::msg::FftDataMessage();
    message.angle = data->Angle;
    message.azimuth = data->Azimuth;
    message.sweep_counter = data->SweepCounter;
    message.ntp_seconds = data->NTPSeconds;
    message.ntp_split_seconds = data->NTPSplitSeconds;
    message.data = data->Data;

    Fft_data_publisher->publish(message);
}

void ConfigurationDataHandler(const ConfigurationDataPtr_t& data)
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
    Configuration_data_publisher->publish(message);

    radarClient->StartFFTData();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    node = std::make_shared<Colossus_publisher>();

    RCLCPP_INFO(node->get_logger(), "Starting radar client");
    radarClient = std::make_shared<RadarClient>("10.77.2.210");
    radarClient->SetFFTDataCallback(std::bind(&FFTDataHandler, std::placeholders::_1));
    radarClient->SetConfigurationDataCallback(std::bind(&ConfigurationDataHandler, std::placeholders::_1));
    radarClient->Start();

    while (rclcpp::ok())
    {
        rclcpp::spin(node);
    }

    radarClient->StopFFTData();
    radarClient->SetConfigurationDataCallback();
    radarClient->SetFFTDataCallback();
    radarClient->Stop();
    rclcpp::shutdown();
    RCLCPP_INFO(node->get_logger(), "Stopped radar client");
}
