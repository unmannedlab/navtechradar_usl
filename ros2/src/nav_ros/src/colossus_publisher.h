#include <rclcpp/rclcpp.hpp>
#include "radar_client.h"

class Colossus_publisher : public ::rclcpp::Node
{
public:
    Colossus_publisher();

    std::shared_ptr<Navtech::Radar_client> radar_client{};
    std::string radar_ip{ "" };
    uint16_t radar_port{ 0 };

    void fft_data_handler(const Navtech::Fft_data::Pointer& data);
    void configuration_data_handler(const Navtech::Configuration_data::Pointer& data);

private:
    int azimuth_samples{ 0 };
    int last_azimuth{ 0 };
    bool rotated_once{ false };

    rclcpp::Publisher<interfaces::msg::ConfigurationDataMessage>::SharedPtr configuration_data_publisher{};
    rclcpp::Publisher<interfaces::msg::FftDataMessage>::SharedPtr fft_data_publisher{};
};