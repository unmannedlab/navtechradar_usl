#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

class Camera_publisher : public ::rclcpp::Node {
public:
    Camera_publisher();

    void camera_image_handler(cv::Mat image, int fps);
};