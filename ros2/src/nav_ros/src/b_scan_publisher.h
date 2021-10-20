#include <rclcpp/rclcpp.hpp>
#include "radar_client.h"
#include "messages/msg/configuration_data_message.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <vector>

class B_scan_colossus_publisher : public ::rclcpp::Node
{
public:
    B_scan_colossus_publisher();

    std::shared_ptr<Navtech::Radar_client> radar_client{};
    std::string radar_ip{ "" };
    uint16_t radar_port{ 0 };
    uint16_t start_azimuth{ 0 };
    uint16_t end_azimuth{ 0 };
    uint16_t start_bin{ 0 };
    uint16_t end_bin{ 0 };
    uint16_t azimuth_offset{ 0 };

    std::vector <uint8_t> intensity_values;

    void fft_data_handler(const Navtech::Fft_data::Pointer& data);
    void configuration_data_handler(const Navtech::Configuration_data::Pointer& data);
    void image_data_handler(const Navtech::Fft_data::Pointer& data);

private:
    constexpr static int radar_configuration_queue_size{ 1 };
    constexpr static int b_scan_image_queue_size{ 4 };

    int azimuth_samples{ 0 };
    int range_in_bins{ 0 };
    int last_azimuth{ 0 };
    bool rotated_once{ false };
    int buffer_length{ 0 };
    int rotation_count{ 0 };
    int config_publish_count{ 4 };

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr b_scan_image_publisher{};
};