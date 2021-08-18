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
#include "../../camera_ros/src/video_capture_manager.h"

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

    std::shared_ptr<Video_capture_manager> vid_cap_manager = std::make_shared<Video_capture_manager>();

    auto ret = vid_cap_manager->connect_to_camera(camera_url);

    if (ret) {

        Mat image{ };

        while (ok()) {

            image = vid_cap_manager->get_latest_frame();
            node->camera_image_handler(image, vid_cap_manager->capture.get(CAP_PROP_FPS));
            spin_some(node);
        }
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down");
    vid_cap_manager->disconnect_from_camera();
    node->cleanup_and_shutdown();
}
