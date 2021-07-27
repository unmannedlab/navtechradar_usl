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
        Fft_data_publisher = Node::create_publisher<interfaces::msg::FftDataMessage>("fft_data", 400);
    }
};

RadarClientPtr_t radarClient;
std::shared_ptr<Colossus_publisher> node;

void FFTDataHandler(const FFTDataPtr_t& data)
{
    auto message = interfaces::msg::ConfigurationDataMessage();
    //message.data = "FFT Data";
    //RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", message.data.c_str());
    //Fft_data_publisher->publish(message);
}

void ConfigurationDataHandler(const ConfigurationDataPtr_t& data)
{
    RCLCPP_INFO(node->get_logger(), "Configuration Data recieved");
    RCLCPP_INFO(node->get_logger(), "Expected Rotation Rate: %i", data->ExpectedRotationRate);
    RCLCPP_INFO(node->get_logger(), "Range In Bins: : %i", data->RangeInBins);
    RCLCPP_INFO(node->get_logger(), "Bin Size: %f", data->BinSize / 10000.0);
    RCLCPP_INFO(node->get_logger(), "Range In Metres: %f", data->BinSize / 10000.0 * data->RangeInBins);
    RCLCPP_INFO(node->get_logger(), "Azimuth Samples: %i", data->AzimuthSamples);

    //byte configDataBytes[10];
    //memcpy(configDataBytes, &data, sizeof(configDataBytes));

    //auto message = std_msgs::Int16MultiArray();
    //message.data = configDataBytes;
    //RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", message.data.c_str());
    //Configuration_data_publisher->publish(message);

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
