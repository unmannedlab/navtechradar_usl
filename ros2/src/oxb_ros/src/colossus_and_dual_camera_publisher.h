#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "radar_client.h"

class Colossus_and_dual_camera_publisher : public ::rclcpp::Node {
public:
    Colossus_and_dual_camera_publisher();

    void fft_data_handler(const Navtech::Fft_data::Pointer& data);

    void configuration_data_handler(const Navtech::Configuration_data::Pointer& data);

    void camera_image_handler(cv::Mat image_left, cv::Mat image_right, int fps_left, int fps_right);

    void cleanup_and_shutdown();

    std::shared_ptr<Navtech::Radar_client> radar_client{};

    std::string radar_ip{ "" };

    uint16_t radar_port{ 0 };

    std::string camera_left_url = { "" };

    std::string camera_right_url = { "" };

private:
    rclcpp::Publisher<interfaces::msg::ConfigurationDataMessage>::SharedPtr configuration_data_publisher{};

    rclcpp::Publisher<interfaces::msg::FftDataMessage>::SharedPtr fft_data_publisher{};

    rclcpp::Publisher<interfaces::msg::CameraImageMessage>::SharedPtr camera_left_image_publisher;

    interfaces::msg::CameraImageMessage camera_left_message = interfaces::msg::CameraImageMessage();

    rclcpp::Publisher<interfaces::msg::CameraImageMessage>::SharedPtr camera_right_image_publisher;

    interfaces::msg::CameraImageMessage camera_right_message = interfaces::msg::CameraImageMessage();

    int bearing_count{ 0 };

    int azimuth_samples{ 0 };

    int last_azimuth{ 0 };

    bool rotated_once{ false };
};