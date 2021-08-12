#include <rclcpp/rclcpp.hpp>

#include "interfaces/msg/configuration_data_message.hpp"
#include "interfaces/msg/fft_data_message.hpp"
#include "interfaces/msg/camera_image_message.hpp"
#include "colossus_and_camera_subscriber_to_video.h"

using namespace std;
using namespace rclcpp;

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Colossus_and_camera_subscriber_to_video>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}