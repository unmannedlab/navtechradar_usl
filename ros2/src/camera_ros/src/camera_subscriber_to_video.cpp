#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <opencv2/opencv.hpp>

#include "interfaces/msg/camera_image_message.hpp"
#include "camera_subscriber_to_video.h"

using namespace std;
using namespace rclcpp;
using namespace cv;

namespace{
    bool config_data_received { false };
    VideoWriter video_writer {};
}

Camera_subscriber_to_video::Camera_subscriber_to_video() :
rclcpp::Node{ "camera_subscriber_to_video" }
{
    using std::placeholders::_1;

    rclcpp::QoS qos_camera_configuration_subscriber(1);
    qos_camera_configuration_subscriber.reliable();

    camera_configuration_subscriber =
    Node::create_subscription<interfaces::msg::CameraConfigurationMessage>(
    "camera_data/camera_configuration_data",
    qos_camera_configuration_subscriber,
    std::bind(&Camera_subscriber_to_video::configuration_data_callback, this, _1));

    rclcpp::QoS qos_camera_image_subscriber(100);
    qos_camera_image_subscriber.reliable();

    camera_data_subscriber =
    Node::create_subscription<sensor_msgs::msg::Image>(
    "camera_data/camera_image_data",
    qos_camera_image_subscriber,
    std::bind(&Camera_subscriber_to_video::camera_image_callback, this, _1));
}

void Camera_subscriber_to_video::configuration_data_callback(const interfaces::msg::CameraConfigurationMessage::SharedPtr data) const
{
    RCLCPP_INFO(Node::get_logger(), "Camera Configuration received");
    RCLCPP_INFO(Node::get_logger(), "Image Width: %i", data->width);
    RCLCPP_INFO(Node::get_logger(), "Image Height: %i", data->height);
    RCLCPP_INFO(Node::get_logger(), "Image Channels: %i", data->channels);
    RCLCPP_INFO(Node::get_logger(), "Video FPS: %i", data->fps);

    video_writer.open("output_videos/camera_output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), data->fps, Size(data->width, data->height), true);
    config_data_received = true;
}

void Camera_subscriber_to_video::camera_image_callback(const sensor_msgs::msg::Image::SharedPtr data) const
{
    if (!config_data_received) {
        RCLCPP_INFO(Node::get_logger(), "No Camera Configuration received");
        return;
    }

    Mat camera_image = Mat{ data->height, data->width, CV_8UC3, data->data.data() }.clone();
    video_writer.write(camera_image);
}