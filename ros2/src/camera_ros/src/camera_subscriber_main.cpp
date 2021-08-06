#include <rclcpp/rclcpp.hpp>

#include "camera_subscriber.h"

using namespace std;
using namespace rclcpp;

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Camera_subscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
