#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

using namespace rclcpp;
using namespace cv;

class Camera_publisher : public ::rclcpp::Node {
public:
    Camera_publisher();

    void camera_image_handler(Mat image, int fps);
};