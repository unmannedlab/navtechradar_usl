#include <rclcpp/rclcpp.hpp>

#include "interfaces/msg/configuration_data_message.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "laser_scan_subscriber.h"

using namespace std;
using namespace rclcpp;

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Laser_scan_subscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}