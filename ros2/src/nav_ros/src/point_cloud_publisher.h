#include <rclcpp/rclcpp.hpp>
#include "radar_client.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "messages/msg/radar_configuration_message.hpp"
#include <vector>

class Point_cloud_publisher : public ::rclcpp::Node
{
public:
    Point_cloud_publisher();

    std::shared_ptr<Navtech::Radar_client> radar_client{};
    std::string radar_ip{ "" };
    uint16_t radar_port{ 0 };
    uint16_t start_azimuth{ 0 };
    uint16_t end_azimuth{ 0 };
    uint16_t start_bin{ 0 };
    uint16_t end_bin{ 0 };
    uint16_t power_threshold{ 0 };
    uint16_t azimuth_offset{ 0 };

    std::vector <float> azimuth_values;
    std::vector <float> bin_values;
    std::vector <float> intensity_values;

    void fft_data_handler(const Navtech::Fft_data::Pointer& data);
    void configuration_data_handler(const Navtech::Configuration_data::Pointer& data);
    void publish_point_cloud(const Navtech::Fft_data::Pointer& data);
    std::vector<uint8_t> floats_to_uint8_t_vector(float x, float y, float z, float intensity);

private:
    constexpr static int radar_configuration_queue_size{ 1 };
    constexpr static int radar_point_cloud_queue_size{ 4 };

    int azimuth_samples{ 0 };
    float bin_size{ 0 };
    int range_in_bins{ 0 };
    int expected_rotation_rate{ 0 };
    int last_azimuth{ 0 };
    bool rotated_once{ false };
    int rotation_count{ 0 };
    int config_publish_count{ 4 };

    messages::msg::RadarConfigurationMessage config_message = messages::msg::RadarConfigurationMessage{};

    rclcpp::Publisher<messages::msg::RadarConfigurationMessage>::SharedPtr configuration_data_publisher{};
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher{};
};