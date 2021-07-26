#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Colossus_subscriber : public rclcpp::Node
{
public:
    Colossus_subscriber()
        : Node{ "colossus_subscriber" }
  {
    using std::placeholders:: _1;

    subscription = Node::create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&Colossus_subscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(Node::get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Colossus_subscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
