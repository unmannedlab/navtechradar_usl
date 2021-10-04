#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "interfaces/msg/configuration_data_message.hpp"
#include "interfaces/msg/fft_data_message.hpp"
#include "interfaces/msg/camera_configuration_message.hpp"
#include "radar_client.h"
#include "colossus_and_camera_publisher.h"
#include "../../camera_ros/src/video_capture_manager.h"

std::shared_ptr<Colossus_and_camera_publisher> node{};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    node = std::make_shared<Colossus_and_camera_publisher>();

    RCLCPP_INFO(node->get_logger(), "Starting radar client");
    node->radar_client = Navtech::allocate_owned<Navtech::Radar_client>(Navtech::Utility::IP_address { node->radar_ip }, node->radar_port);
    node->radar_client->set_fft_data_callback(std::bind(&Colossus_and_camera_publisher::fft_data_handler, node.get(), std::placeholders::_1));
    node->radar_client->set_configuration_data_callback(std::bind(&Colossus_and_camera_publisher::configuration_data_handler, node.get(), std::placeholders::_1));
    node->radar_client->start();
    RCLCPP_INFO(node->get_logger(), "Radar client started");

    RCLCPP_INFO(node->get_logger(), "Starting camera publisher");
    RCLCPP_INFO(node->get_logger(), "URL: %s", node->camera_url.c_str());

    std::shared_ptr<Video_capture_manager> vid_cap_manager = std::make_shared<Video_capture_manager>();

    auto ret = vid_cap_manager->connect_to_camera(node->camera_url);

    if (ret) {

        cv::Mat image{ };

        while (rclcpp::ok()) {

            image = vid_cap_manager->get_latest_frame();
            node->camera_image_handler(image);
            spin_some(node);
        }
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down");
    vid_cap_manager->disconnect_from_camera();
    node->cleanup_and_shutdown();
}
