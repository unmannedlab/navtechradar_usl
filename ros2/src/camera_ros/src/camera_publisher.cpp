#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

#include "interfaces/msg/camera_configuration_message.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "camera_publisher.h"

using namespace std;
using namespace rclcpp;
using namespace cv;
using namespace chrono;

Camera_publisher::Camera_publisher():rclcpp::Node{ "camera_publisher" }
{
    declare_parameter("camera_url", "");
    camera_url = get_parameter("camera_url").as_string();

    camera_configuration_publisher =
    Node::create_publisher<interfaces::msg::CameraConfigurationMessage>(
    "camera_data/camera_configuration_data",
    1);

    camera_image_publisher =
    Node::create_publisher<sensor_msgs::msg::Image>(
    "camera_data/camera_image_data",
    100);
}

void Camera_publisher::camera_image_handler(Mat image, int fps)
{
    //RCLCPP_INFO(Node::get_logger(), "Publishing Camera Image Data");
    //RCLCPP_INFO(Node::get_logger(), "Mat rows: %i", image.rows);
    //RCLCPP_INFO(Node::get_logger(), "Mat columns: %i", image.cols);
    //RCLCPP_INFO(Node::get_logger(), "Mat size: %i", image.rows * image.cols);
    //RCLCPP_INFO(Node::get_logger(), "Mat type: %i", image.type());

    auto buffer_length = image.cols * image.rows * sizeof(uint8_t) * 3;
    vector<uint8_t> vector_buffer(image.ptr(0), image.ptr(0) + buffer_length);

    //RCLCPP_INFO(Node::get_logger(), "Image buffer size: %li", buffer_length);

    if (!configuration_sent) {
        auto config_message = interfaces::msg::CameraConfigurationMessage();
        config_message.width = image.cols;
        config_message.height = image.rows;
        config_message.channels = image.channels();
        config_message.fps = fps;
        camera_configuration_publisher->publish(config_message);
        configuration_sent = true;
    }

    auto message = sensor_msgs::msg::Image();
    message.header = std_msgs::msg::Header();
    message.header.stamp = node->get_clock()->now();

    message.height = image.rows;
    message.width = image.cols;
    message.encoding = "8UC3";
    message.is_bigendian = true;
    message.step = image.step;
    message.data = std::move(vector_buffer);

    camera_image_publisher->publish(message);
}