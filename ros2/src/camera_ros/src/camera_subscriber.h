#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "interfaces/msg/camera_image_message.hpp"
#include "opencv2/opencv.hpp"

class Camera_subscriber : public ::rclcpp::Node {
public:
    Camera_subscriber();

private:
    void camera_image_callback(const interfaces::msg::CameraImageMessage::SharedPtr data) const;

    rclcpp::Subscription<interfaces::msg::CameraImageMessage>::SharedPtr camera_data_subscriber;
};