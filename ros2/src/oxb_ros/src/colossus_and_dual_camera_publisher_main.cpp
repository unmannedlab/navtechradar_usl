#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "interfaces/msg/configuration_data_message.hpp"
#include "interfaces/msg/fft_data_message.hpp"
#include "interfaces/msg/camera_image_message.hpp"
#include "radar_client.h"
#include "colossus_and_dual_camera_publisher.h"
#include "../../camera_ros/src/video_capture_manager.h"

using namespace std;
using namespace Navtech;
using namespace rclcpp;
using namespace cv;
using namespace chrono;

int main(int argc, char* argv[]){
    init(argc, argv);
    node = std::make_shared<Colossus_and_dual_camera_publisher>();

    RCLCPP_INFO(node->get_logger(), "Starting radar client");
    radar_client = allocate_owned<Radar_client>(radar_ip, radar_port);
    radar_client->set_fft_data_callback(std::bind(&Colossus_and_dual_camera_publisher::fft_data_handler, node.get(), std::placeholders::_1));
    radar_client->set_configuration_data_callback(std::bind(&Colossus_and_dual_camera_publisher::configuration_data_handler, node.get(), std::placeholders::_1));
    radar_client->start();
    RCLCPP_INFO(node->get_logger(), "Radar client started");

    RCLCPP_INFO(node->get_logger(), "Starting camera publisher");
    RCLCPP_INFO(node->get_logger(), "URL left: %s", camera_left_url.c_str());
    RCLCPP_INFO(node->get_logger(), "URL right: %s", camera_right_url.c_str());

    std::shared_ptr<Video_capture_manager> vid_cap_manager_left = std::make_shared<Video_capture_manager>();
    std::shared_ptr<Video_capture_manager> vid_cap_manager_right = std::make_shared<Video_capture_manager>();

    auto ret_left = vid_cap_manager_left->connect_to_camera(camera_left_url);
    auto ret_right = vid_cap_manager_left->connect_to_camera(camera_right_url);

    if (ret_left && ret_right) {

        Mat image_left{ };
        Mat image_right{ };

        while (ok()) {

            image_left = vid_cap_manager_left->get_latest_frame();
            image_right = vid_cap_manager_right->get_latest_frame();
            node->camera_image_handler(image_left, image_right, vid_cap_manager_left->capture.get(CAP_PROP_FPS), vid_cap_manager_right->capture.get(CAP_PROP_FPS));
            spin_some(node);
        }
    }

    RCLCPP_INFO(node->get_logger(), "Shutting down");
    vid_cap_manager_left->disconnect_from_camera();
    vid_cap_manager_right->disconnect_from_camera();
    node->cleanup_and_shutdown();
}
