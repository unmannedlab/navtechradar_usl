#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Colossus_subscriber : public rclcpp::Node
{
public:
    Colossus_subscriber() : Node{ "colossus_subscriber" }
    {
        using std::placeholders::_1;

        Configuration_data_subscriber = Node::create_subscription<std_msgs::msg::String>(
            "configuration_data", 5, std::bind(&Colossus_subscriber::configuration_data_callback, this, _1));
        Fft_data_subscriber = Node::create_subscription<std_msgs::msg::String>(
            "fft_data", 400, std::bind(&Colossus_subscriber::fft_data_callback, this, _1));
    }

private:
    void configuration_data_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(Node::get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    void fft_data_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(Node::get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Configuration_data_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Fft_data_subscriber;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Colossus_subscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
