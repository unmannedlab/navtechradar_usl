#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#include "interfaces/msg/camera_image_message.hpp"
#include "camera_publisher.h"

using namespace std;
using namespace rclcpp;
using namespace cv;
using namespace chrono;

Publisher<interfaces::msg::CameraImageMessage>::SharedPtr camera_image_publisher {};
std::string camera_url {""};
std::shared_ptr<Camera_publisher> node {};

Camera_publisher::Camera_publisher():rclcpp::Node{ "camera_publisher" }{
    declare_parameter("camera_url", "");

    camera_url = get_parameter("camera_url").as_string();

    camera_image_publisher =
        Node::create_publisher<interfaces::msg::CameraImageMessage>(
            "camera_data/image_data",
            100);
}

void Camera_publisher::camera_image_handler(Mat image, int fps){
    //RCLCPP_INFO(Node::get_logger(), "Publishing Camera Image Data");
    //RCLCPP_INFO(Node::get_logger(), "Mat rows: %i", image.rows);
    //RCLCPP_INFO(Node::get_logger(), "Mat columns: %i", image.cols);
    //RCLCPP_INFO(Node::get_logger(), "Mat size: %i", image.rows * image.cols);
    //RCLCPP_INFO(Node::get_logger(), "Mat type: %i", image.type());

    auto buffer_length = image.cols * image.rows * sizeof(uint8_t) * 3;
    vector<unsigned char> vectorBuffer(image.ptr(0), image.ptr(0) + buffer_length);

    auto millisec_since_epoch = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    auto sec_since_epoch = duration_cast<seconds>(system_clock::now().time_since_epoch()).count();

    //RCLCPP_INFO(Node::get_logger(), "Image buffer size: %li", buffer_length);

    auto message = interfaces::msg::CameraImageMessage();
    message.image_data = vectorBuffer;
    message.image_rows = image.rows;
    message.image_cols = image.cols;
    message.image_channels = image.channels();
    message.image_fps = fps;
    message.ntp_seconds = sec_since_epoch;
    message.ntp_split_seconds = millisec_since_epoch - (sec_since_epoch * 1000);

    camera_image_publisher->publish(message);
}