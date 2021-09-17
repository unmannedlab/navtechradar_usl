#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include "interfaces/msg/configuration_data_message.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "laser_scan_subscriber_to_video.h"

using namespace std;
using namespace rclcpp;
using namespace cv;

std::shared_ptr<Laser_scan_subscriber_to_video> node{};

Laser_scan_subscriber_to_video::Laser_scan_subscriber_to_video() :Node{ "laser_scan_subscriber_to_video" }
{
    using std::placeholders::_1;

    rclcpp::QoS qos_radar_configuration_subscriber(radar_configuration_queue_size);
    qos_radar_configuration_subscriber.reliable();

    configuration_data_subscriber =
    Node::create_subscription<interfaces::msg::ConfigurationDataMessage>(
    "radar_data/configuration_data",
    qos_radar_configuration_subscriber,
    std::bind(&Laser_scan_subscriber_to_video::configuration_data_callback, this, _1));

    rclcpp::QoS qos_radar_laser_scan_subscriber(radar_laser_scan_queue_size);
    qos_radar_laser_scan_subscriber.reliable();

    laser_scan_subscriber =
    Node::create_subscription<sensor_msgs::msg::LaserScan>(
    "radar_data/laser_scan",
    qos_radar_laser_scan_subscriber,
    std::bind(&Laser_scan_subscriber_to_video::laser_scan_callback, this, _1));
}

void Laser_scan_subscriber_to_video::configuration_data_callback(const interfaces::msg::ConfigurationDataMessage::SharedPtr msg) const
{
    RCLCPP_INFO(Node::get_logger(), "Configuration Data recieved");
    RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", msg->azimuth_samples);
    node->azimuth_samples = msg->azimuth_samples;
    RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", msg->encoder_size);
    node->encoder_size = msg->encoder_size;
    RCLCPP_INFO(Node::get_logger(), "Bin Size: %f", msg->bin_size);
    RCLCPP_INFO(Node::get_logger(), "Range In Bins: %i", msg->range_in_bins);
    RCLCPP_INFO(Node::get_logger(), "Expected Rotation Rate: %i", msg->expected_rotation_rate);

    node->video_width = msg->range_in_bins;
    node->video_height = msg->azimuth_samples;
    node->video_writer.open("output_videos/laser_scan_output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), msg->expected_rotation_rate, Size(azimuth_samples, azimuth_samples), true);
    node->config_data_received = true;
}
void Laser_scan_subscriber_to_video::laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
{
    if (!node->config_data_received) {
        RCLCPP_INFO(Node::get_logger(), "No Configuration Data Received");
        return;
    }

    RCLCPP_INFO(Node::get_logger(), "Laser Scan Received");
    time_t epoch = msg->header.stamp.sec;
    RCLCPP_INFO(Node::get_logger(), "Timestamp: %s", asctime(gmtime(&epoch)));
    RCLCPP_INFO(Node::get_logger(), "Start angle: %f", msg->angle_min * (180 / M_PI));
    RCLCPP_INFO(Node::get_logger(), "End angle: %f", msg->angle_max * (180 / M_PI));
    RCLCPP_INFO(Node::get_logger(), "Angle increment: %f", msg->angle_increment * (180 / M_PI));
    RCLCPP_INFO(Node::get_logger(), "Range min: %f", msg->range_min);
    RCLCPP_INFO(Node::get_logger(), "Range max: %f", msg->range_max);
    RCLCPP_INFO(Node::get_logger(), "Time increment: %f", msg->time_increment);
    RCLCPP_INFO(Node::get_logger(), "Scan time: %f", msg->scan_time);
    RCLCPP_INFO(Node::get_logger(), "Ranges: %i", msg->ranges.size());
    RCLCPP_INFO(Node::get_logger(), "Intensities: %i", msg->intensities.size());

    Mat laser_scan_image{ Size(azimuth_samples, azimuth_samples), CV_8UC1, Scalar(0, 0) };
    for (int r = 0; r < msg->ranges.size(); r++){
        auto index = int(msg->ranges[r] / 0.175);
        auto intensity = int(msg->intensities[r]);
        if (index < azimuth_samples) {
            // Note - these points have been enhanced for visual purposes
            circle(laser_scan_image, Point(msg->ranges[r], r), 2 ,Scalar(msg->intensities[r], msg->intensities[r], msg->intensities[r]), FILLED, 1);
        }
    }

    Mat recovered_lin_polar_img;
    Point2f center((float)laser_scan_image.cols / 2, (float)laser_scan_image.rows / 2);
    double max_radius = min(center.y, center.x);
    linearPolar(laser_scan_image, recovered_lin_polar_img, center, max_radius, INTER_LINEAR + WARP_FILL_OUTLIERS + WARP_INVERSE_MAP);
    Mat normalised_image(Size(azimuth_samples, azimuth_samples), CV_8UC1, Scalar(0, 0));
    normalize(recovered_lin_polar_img, normalised_image, 0, 255, NORM_MINMAX);
    Mat rotated_image(Size(azimuth_samples, azimuth_samples), CV_8UC1, Scalar(0, 0));
    rotate(normalised_image, rotated_image, ROTATE_90_COUNTERCLOCKWISE);
    Mat channels[3] = { blank_image, rotated_image, blank_image };
    Mat merged_data;
    merge(channels, 3, merged_data);
    node->video_writer.write(merged_data);
}