#include <rclcpp/rclcpp.hpp>
#include "radar_client.h"
#include "interfaces/msg/configuration_data_message.hpp"

class Colossus_test_tool : public ::rclcpp::Node
{
public:
    Colossus_test_tool();

    const int sweep_counter_increment{1};
    const int azimuth_start_index{ 1 };
    const int azimuth_increment{14};
    const int azimuth_limit{ 5587 };

    std::shared_ptr<Navtech::Radar_client> radar_client{};
    std::string radar_ip{ "" };
    uint16_t radar_port{ 0 };

    uint16_t previous_sweep_counter{ 0 };
    uint16_t previous_azimuth{ 0 };

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

    interfaces::msg::ConfigurationDataMessage config_message = interfaces::msg::ConfigurationDataMessage{};

    rclcpp::Publisher<interfaces::msg::ConfigurationDataMessage>::SharedPtr configuration_data_publisher{};
    rclcpp::Publisher<interfaces::msg::FftDataMessage>::SharedPtr fft_data_publisher{};
};