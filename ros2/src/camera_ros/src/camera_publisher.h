#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

class Camera_publisher : public ::rclcpp::Node {
public:
    Camera_publisher();

    std::string camera_url{ "" };

    void camera_image_handler(cv::Mat image, int fps);

private:

    rclcpp::Publisher<interfaces::msg::CameraImageMessage>::SharedPtr camera_image_publisher{};
};