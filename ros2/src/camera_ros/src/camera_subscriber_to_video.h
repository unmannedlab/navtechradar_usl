#include <rclcpp/rclcpp.hpp>

using namespace rclcpp;

class Camera_subscriber_to_video : public ::rclcpp::Node {
public:
    Camera_subscriber_to_video();

private:
    void camera_image_callback(const interfaces::msg::CameraImageMessage::SharedPtr data) const;

    rclcpp::Subscription<interfaces::msg::CameraImageMessage>::SharedPtr camera_data_subscriber;
};