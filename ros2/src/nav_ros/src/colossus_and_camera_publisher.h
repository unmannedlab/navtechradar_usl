#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "radar_client.h"

#include "messages/msg/camera_configuration_message.hpp"
#include "sensor_msgs/msg/image.hpp"

class Colossus_and_camera_publisher : public ::rclcpp::Node
{
public:
    Colossus_and_camera_publisher();

    std::shared_ptr<Navtech::Radar_client> radar_client{};
    std::string radar_ip{ "" };
    uint16_t radar_port{ 0 };
    std::string camera_url = { "" };

    void fft_data_handler(const Navtech::Fft_data::Pointer& data);
    void configuration_data_handler(const Navtech::Configuration_data::Pointer& data);
    void camera_image_handler(cv::Mat image);
    void cleanup_and_shutdown();

private:
    constexpr static int camera_configuration_queue_size{ 1 };
    constexpr static int camera_image_queue_size{ 25 };
    constexpr static int radar_configuration_queue_size{ 1 };
    constexpr static int radar_fft_queue_size{ 400 };

    int bearing_count { 0 };
    int azimuth_samples { 0 };
    int fps{ 0 };
    int last_azimuth{ 0 };
    bool rotated_once{ false };
    int rotation_count{ 0 };
    int frame_count{ 0 };
    int config_publish_count{ 4 };
    bool configuration_sent{ false };

    messages::msg::ConfigurationDataMessage config_message = messages::msg::ConfigurationDataMessage();

    rclcpp::Publisher<messages::msg::CameraConfigurationMessage>::SharedPtr camera_configuration_publisher{};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_image_publisher{};
    sensor_msgs::msg::Image camera_message = sensor_msgs::msg::Image();
    rclcpp::Publisher<messages::msg::ConfigurationDataMessage>::SharedPtr configuration_data_publisher{};
    rclcpp::Publisher<messages::msg::FftDataMessage>::SharedPtr fft_data_publisher{};
};

extern std::shared_ptr<Colossus_and_camera_publisher> node;