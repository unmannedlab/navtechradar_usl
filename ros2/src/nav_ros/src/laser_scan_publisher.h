#include <rclcpp/rclcpp.hpp>
#include "radar_client.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>

class Laser_scan_publisher : public ::rclcpp::Node
{
public:
    Laser_scan_publisher();

    std::shared_ptr<Navtech::Radar_client> radar_client{};
    std::string radar_ip{ "" };
    uint16_t radar_port{ 0 };
    uint16_t start_azimuth{ 0 };
    uint16_t end_azimuth{ 0 };
    uint16_t start_bin{ 0 };
    uint16_t end_bin{ 0 };
    uint16_t power_threshold{ 0 };

    std::vector <float> range_values;
    std::vector <float> intensity_values;

    void fft_data_handler(const Navtech::Fft_data::Pointer& data);
    void configuration_data_handler(const Navtech::Configuration_data::Pointer& data);
    void publish_laser_scan();

private:
    constexpr static int radar_configuration_queue_size{ 1 };
    constexpr static int radar_laser_scan_queue_size{ 400 };

    int azimuth_samples{ 0 };
    int last_azimuth{ 0 };
    bool rotated_once{ false };

    rclcpp::Publisher<interfaces::msg::ConfigurationDataMessage>::SharedPtr configuration_data_publisher{};
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher{};
};