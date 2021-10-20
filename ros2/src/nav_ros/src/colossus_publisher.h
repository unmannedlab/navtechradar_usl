#include <rclcpp/rclcpp.hpp>
#include "radar_client.h"
#include "messages/msg/radar_configuration_message.hpp"

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
    constexpr static int radar_configuration_queue_size{ 1 };
    constexpr static int radar_fft_queue_size{ 400 };

    int azimuth_samples{ 0 };
    int last_azimuth{ 0 };
    bool rotated_once{ false };
    int rotation_count{ 0 };
    int config_publish_count{ 4 };

    messages::msg::RadarConfigurationMessage config_message = messages::msg::RadarConfigurationMessage{};

    rclcpp::Publisher<messages::msg::RadarConfigurationMessage>::SharedPtr configuration_data_publisher{};
    rclcpp::Publisher<messages::msg::RadarFftDataMessage>::SharedPtr fft_data_publisher{};
};