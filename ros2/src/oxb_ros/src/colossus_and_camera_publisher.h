#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "radar_client.h"

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

    void camera_image_handler(cv::Mat image, int fps);

    void cleanup_and_shutdown();

private:
    int bearing_count{ 0 };

    int azimuth_samples{ 0 };

    int last_azimuth{ 0 };

    bool rotated_once{ false };

    rclcpp::Publisher<interfaces::msg::CameraImageMessage>::SharedPtr camera_image_publisher;

    interfaces::msg::CameraImageMessage camera_message = interfaces::msg::CameraImageMessage();

    rclcpp::Publisher<interfaces::msg::ConfigurationDataMessage>::SharedPtr configuration_data_publisher{};

    rclcpp::Publisher<interfaces::msg::FftDataMessage>::SharedPtr fft_data_publisher{};
};