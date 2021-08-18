#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "interfaces/msg/configuration_data_message.hpp"
#include "interfaces/msg/fft_data_message.hpp"
#include "interfaces/msg/camera_image_message.hpp"
#include "radar_client.h"
#include "colossus_and_camera_publisher.h"

using namespace std;
using namespace Navtech;
using namespace rclcpp;
using namespace cv;
using namespace chrono;

int main(int argc, char* argv[]){
    init(argc, argv);
    node = std::make_shared<Colossus_and_camera_publisher>();

    RCLCPP_INFO(node->get_logger(), "Starting radar client");
    radar_client = allocate_owned<Radar_client>(radar_ip, radar_port);
    radar_client->set_fft_data_callback(std::bind(&Colossus_and_camera_publisher::fft_data_handler, node.get(), std::placeholders::_1));
    radar_client->set_configuration_data_callback(std::bind(&Colossus_and_camera_publisher::configuration_data_handler, node.get(), std::placeholders::_1));
    radar_client->start();
    RCLCPP_INFO(node->get_logger(), "Radar client started");

    RCLCPP_INFO(node->get_logger(), "Starting camera publisher");
    RCLCPP_INFO(node->get_logger(), "URL: %s", camera_url.c_str());
    VideoCapture capture{ camera_url };
	
    if (!capture.isOpened()) {
        RCLCPP_INFO(node->get_logger(), "Unable to connect to camera");
        node->cleanup_and_shutdown();
    }
    else {
        RCLCPP_INFO(node->get_logger(), "Camera connected");
        RCLCPP_INFO(node->get_logger(), "Width: %f", capture.get(CAP_PROP_FRAME_WIDTH));
        RCLCPP_INFO(node->get_logger(), "Height: %f", capture.get(CAP_PROP_FRAME_HEIGHT));
        RCLCPP_INFO(node->get_logger(), "FPS: %f", capture.get(CAP_PROP_FPS));
    }

    Mat image{ };
    while (ok()) {
        capture >> image;
        node->camera_image_handler(image, capture.get(CAP_PROP_FPS));
        spin_some(node);
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down");
    node->cleanup_and_shutdown();
}
