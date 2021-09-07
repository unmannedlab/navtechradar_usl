#include <rclcpp/rclcpp.hpp>

#include "interfaces/msg/camera_configuration_message.hpp"
#include "sensor_msgs/msg/image.hpp"

class Camera_subscriber_to_video : public ::rclcpp::Node {
public:
    Camera_subscriber_to_video();

private:
    void configuration_data_callback(const interfaces::msg::CameraConfigurationMessage::SharedPtr data) const;
    void camera_image_callback(const sensor_msgs::msg::Image::SharedPtr data) const;

    rclcpp::Subscription<interfaces::msg::CameraConfigurationMessage>::SharedPtr camera_configuration_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_data_subscriber;
};