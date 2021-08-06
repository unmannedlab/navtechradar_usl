#include <rclcpp/rclcpp.hpp>

using namespace rclcpp;

class Camera_subscriber : public ::rclcpp::Node {
public:
    Camera_subscriber();

private:
    void camera_image_callback(const interfaces::msg::CameraImageMessage::SharedPtr data) const;

    rclcpp::Subscription<interfaces::msg::CameraImageMessage>::SharedPtr camera_data_subscriber;
};