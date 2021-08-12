#include <rclcpp/rclcpp.hpp>

class Colossus_and_camera_subscriber_to_video : public ::rclcpp::Node {
public:
    Colossus_and_camera_subscriber_to_video();

    void configuration_data_callback(const interfaces::msg::ConfigurationDataMessage::SharedPtr msg) const;

    void fft_data_callback(const interfaces::msg::FftDataMessage::SharedPtr msg) const;

    void camera_image_callback(const interfaces::msg::CameraImageMessage::SharedPtr data) const;

    rclcpp::Subscription<interfaces::msg::ConfigurationDataMessage>::SharedPtr configuration_data_subscriber;
    rclcpp::Subscription<interfaces::msg::FftDataMessage>::SharedPtr fft_data_subscriber;
    rclcpp::Subscription<interfaces::msg::CameraImageMessage>::SharedPtr camera_data_subscriber;
};