#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/camera_image_message.hpp"

using namespace std;
using namespace rclcpp;

Publisher<interfaces::msg::CameraImageMessage>::SharedPtr camera_image_publisher;

string camera_url;

class Camera_publisher : public rclcpp::Node
{
public:
    Camera_publisher() : Node{ "camera_publisher" }
    {
        declare_parameter("camera_url");

        camera_url = get_parameter("camera_url").as_string();

        camera_image_publisher = Node::create_publisher<interfaces::msg::CameraImageMessage>("camera_data/image_data", 100);
    }

    void camera_image_handler()
    {
        RCLCPP_INFO(Node::get_logger(), "Publishing Camera Image Data");

        auto message = interfaces::msg::CameraImageMessage();

        camera_image_publisher->publish(message);
    }
};

std::shared_ptr<Camera_publisher> node;

int main(int argc, char* argv[])
{
    init(argc, argv);
    node = std::make_shared<Camera_publisher>();

    RCLCPP_INFO(node->get_logger(), "Starting camera publisher");

    while (ok()) {
        spin(node);
    }

    shutdown();
    RCLCPP_INFO(node->get_logger(), "Stopped camera publisher");
}
