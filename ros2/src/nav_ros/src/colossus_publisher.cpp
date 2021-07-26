#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "radarclient.h"

using namespace std::chrono_literals;
using namespace Navtech;

RadarClientPtr_t _radarClient;

class ColossusPublisher : public rclcpp::Node
{
public:
    ColossusPublisher()
  : Node("colossus_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&ColossusPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Colossus Publisher! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
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
  auto node = std::make_shared<ColossusPublisher>();

  RCLCPP_INFO(node->get_logger(), "Starting radar client");
  //_radarClient = std::make_shared<Navtech::RadarClient>("192.168.0.1");
  //_radarClient->SetFFTDataCallback(std::bind(&FFTDataHandler, std::placeholders::_1));
  //_radarClient->SetConfigurationDataCallback(std::bind(&ConfigurationDataHandler, std::placeholders::_1));
  //_radarClient->Start();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
