#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "interfaces/msg/camera_image_message.hpp"
#include "opencv2/opencv.hpp"
#include "camera_subscriber.h"

using namespace std;
using namespace rclcpp;
using namespace cv;

Camera_subscriber::Camera_subscriber():rclcpp::Node{ "camera_subscriber" }{
    using std::placeholders::_1;

    camera_data_subscriber =
        Node::create_subscription<interfaces::msg::CameraImageMessage>(
        "camera_data/image_data",
            100,
            std::bind(&Camera_subscriber::camera_image_callback, this, _1));
}

void Camera_subscriber::camera_image_callback(const interfaces::msg::CameraImageMessage::SharedPtr data) const{
    RCLCPP_INFO(Node::get_logger(), "Camera Data received");
    RCLCPP_INFO(Node::get_logger(), "Image Rows: %i", data->image_rows);
    RCLCPP_INFO(Node::get_logger(), "Image Cols: %i", data->image_cols);
    RCLCPP_INFO(Node::get_logger(), "Image Channels: %i", data->image_channels);
    RCLCPP_INFO(Node::get_logger(), "NTP Seconds: %i", data->ntp_seconds);
    RCLCPP_INFO(Node::get_logger(), "NTP Split Seconds: %i", data->ntp_split_seconds);

    auto dataType = CV_8UC3;
    if (data->image_channels == 1) {
        dataType = CV_8UC1;
    }
    Mat test_image = Mat(data->image_rows, data->image_cols, dataType, data->image_data.data()).clone();
    //imwrite("test.jpg", test_image);
}