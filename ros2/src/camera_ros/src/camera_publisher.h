#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include "sensor_msgs/msg/image.hpp"

class Camera_publisher : public ::rclcpp::Node {
public:
    Camera_publisher();

    std::string camera_url{ "" };

    void camera_image_handler(cv::Mat image, int fps);

private:
    constexpr static int camera_configuration_queue_size{ 1 };
    constexpr static int camera_image_queue_size{ 25 };

    bool configuration_sent{ false };

    rclcpp::Publisher<interfaces::msg::CameraConfigurationMessage>::SharedPtr camera_configuration_publisher{};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_image_publisher{};
};

extern std::shared_ptr<Camera_publisher> node;