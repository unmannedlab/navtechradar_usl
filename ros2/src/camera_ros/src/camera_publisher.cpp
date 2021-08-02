#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/camera_image_message.hpp"
#include "opencv2/opencv.hpp"
#include "iostream"

using namespace std;
using namespace rclcpp;
using namespace cv;
using namespace chrono;

Publisher<interfaces::msg::CameraImageMessage>::SharedPtr camera_image_publisher;

string camera_url;

class Camera_publisher : public rclcpp::Node
{
public:
    Camera_publisher() : Node{ "camera_publisher" }
    {
        declare_parameter("camera_url", "");

        camera_url = get_parameter("camera_url").as_string();

        camera_image_publisher = Node::create_publisher<interfaces::msg::CameraImageMessage>("camera_data/image_data", 100);
    }

    void camera_image_handler(Mat image, int fps)
    {
        //RCLCPP_INFO(Node::get_logger(), "Publishing Camera Image Data");
        //RCLCPP_INFO(Node::get_logger(), "Mat rows: %i", image.rows);
        //RCLCPP_INFO(Node::get_logger(), "Mat columns: %i", image.cols);
        //RCLCPP_INFO(Node::get_logger(), "Mat size: %i", image.rows * image.cols);
        //RCLCPP_INFO(Node::get_logger(), "Mat type: %i", image.type());

        auto buffer_length = image.cols * image.rows * sizeof(uint8_t) * 3;
        auto image_buffer = new uint8_t[buffer_length];
        memcpy(image_buffer, image.ptr(0), image.cols * image.rows * sizeof(uint8_t) * 3);
        const unsigned char* charBuffer = (unsigned char*)image_buffer;
        vector<unsigned char> vectorBuffer(charBuffer, charBuffer + buffer_length);

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
};

std::shared_ptr<Camera_publisher> node;

int main(int argc, char* argv[])
{
    init(argc, argv);
    node = std::make_shared<Camera_publisher>();

    RCLCPP_INFO(node->get_logger(), "Starting camera publisher");

    VideoCapture capture(camera_url);

    if (!capture.isOpened()) {
        RCLCPP_INFO(node->get_logger(), "Unable to connect to camera");
    }
    else {
        RCLCPP_INFO(node->get_logger(), "Camera connected");
        RCLCPP_INFO(node->get_logger(), "Width: %f", capture.get(CAP_PROP_FRAME_WIDTH));
        RCLCPP_INFO(node->get_logger(), "Height: %f", capture.get(CAP_PROP_FRAME_HEIGHT)); 
        RCLCPP_INFO(node->get_logger(), "FPS: %f", capture.get(CAP_PROP_FPS));
    }

    Mat image;
    while (ok()) {
        capture >> image;
        node->camera_image_handler(image, capture.get(CAP_PROP_FPS));
        spin_some(node);
    }

    shutdown();
    RCLCPP_INFO(node->get_logger(), "Stopped camera publisher");
}
