#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include "interfaces/msg/camera_image_message.hpp"
#include "camera_subscriber_to_video.h"

using namespace std;
using namespace rclcpp;
using namespace cv;

namespace{
    bool first_frame { true };
    VideoWriter video_writer {};
}

Camera_subscriber_to_video::Camera_subscriber_to_video() :
    rclcpp::Node{ "camera_subscriber_to_video" }{

    using std::placeholders::_1;

    camera_data_subscriber =
        Node::create_subscription<interfaces::msg::CameraImageMessage>(
        "camera_data/image_data",
            100,
            std::bind(&Camera_subscriber_to_video::camera_image_callback, this, _1));
}

void Camera_subscriber_to_video::camera_image_callback(const interfaces::msg::CameraImageMessage::SharedPtr data) const{
    if (first_frame) {
        RCLCPP_INFO(Node::get_logger(), "Camera Data received");
        RCLCPP_INFO(Node::get_logger(), "Image Rows: %i", data->image_rows);
        RCLCPP_INFO(Node::get_logger(), "Image Cols: %i", data->image_cols);
        RCLCPP_INFO(Node::get_logger(), "Image Channels: %i", data->image_channels);
        RCLCPP_INFO(Node::get_logger(), "Image FPS: %i", data->image_fps);

        video_writer.open("output_videos/camera_output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), data->image_fps, Size(data->image_cols, data->image_rows), true);

        first_frame = false;
    }

    auto dataType = CV_8UC3;
    if (data->image_channels == 1) {
        dataType = CV_8UC1;
    }
    Mat camera_image = Mat{ data->image_rows, data->image_cols, dataType, data->image_data.data() }.clone();
    video_writer.write(camera_image);
}