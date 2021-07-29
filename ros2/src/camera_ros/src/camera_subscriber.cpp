#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/camera_image_message.hpp"

using namespace std;
using namespace rclcpp;

class Camera_subscriber : public Node
{
public:
    Camera_subscriber() : Node{ "camera_subscriber" }
    {
        using std::placeholders::_1;

        camera_data_subscriber = Node::create_subscription<interfaces::msg::CameraImageMessage>(
            "camera_data/image_data", 100, std::bind(&Camera_subscriber::camera_image_callback, this, _1));
    }

private:
    void camera_image_callback(const interfaces::msg::CameraImageMessage::SharedPtr data) const
    {
        RCLCPP_INFO(Node::get_logger(), "Camera Data received");
        RCLCPP_INFO(Node::get_logger(), "Data Length: %i", data->data_length);
        RCLCPP_INFO(Node::get_logger(), "NTP Seconds: %i", data->ntp_seconds);
        RCLCPP_INFO(Node::get_logger(), "NTP Split Seconds: %i", data->ntp_split_seconds);
    }

    rclcpp::Subscription<interfaces::msg::CameraImageMessage>::SharedPtr camera_data_subscriber;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Camera_subscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
