#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "radarclient.h"

using namespace std::chrono_literals;
using namespace Navtech;

RadarClientPtr_t radarClient;

class Colossus_publisher : public rclcpp::Node
{
public:
    Colossus_publisher()
        : Node{"colossus_publisher"}, count(0)
  {
    publisher = Node::create_publisher<std_msgs::msg::String>("topic", 10);
    timer = Node::create_wall_timer(
      500ms, std::bind(&Colossus_publisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Colossus Publisher: " + std::to_string(count++);
    RCLCPP_INFO(Node::get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
  size_t count{ 0 };
};

void FFTDataHandler(const FFTDataPtr_t& data)
{
}

void ConfigurationDataHandler(const ConfigurationDataPtr_t& data)
{
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Colossus_publisher>();

  RCLCPP_INFO(node->get_logger(), "Starting radar client");
  //radarClient = std::make_shared<Navtech::RadarClient>("192.168.0.1");
  //radarClient->SetFFTDataCallback(std::bind(&FFTDataHandler, std::placeholders::_1));
  //radarClient->SetConfigurationDataCallback(std::bind(&ConfigurationDataHandler, std::placeholders::_1));
  //radarClient->Start();

  rclcpp::spin(node);
  rclcpp::shutdown();
}
