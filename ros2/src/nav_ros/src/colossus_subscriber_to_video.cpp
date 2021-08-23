#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include "interfaces/msg/configuration_data_message.hpp"
#include "interfaces/msg/fft_data_message.hpp"
#include "colossus_subscriber_to_video.h"

using namespace std;
using namespace rclcpp;
using namespace cv;

auto config_data_received{ false };
VideoWriter video_writer{};
int encoder_size{ 0 };
int azimuth_samples{ 0 };
int video_width{ 0 };
int video_height{ 0 };
int current_bearing{ 0 };
Mat radar_image{ Size(400, 400), CV_8UC1, Scalar(0, 0) };
Mat blank_image{ Size(400, 400), CV_8UC1, Scalar(0, 0) };
uint8_t* image_ptr = { (uint8_t*)radar_image.data };
int last_azimuth{ 0 };

Colossus_subscriber_to_video::Colossus_subscriber_to_video() :Node{ "colossus_subscriber_to_video" } {
    using std::placeholders::_1;

    configuration_data_subscriber =
        Node::create_subscription<interfaces::msg::ConfigurationDataMessage>(
            "radar_data/configuration_data",
            5,
            std::bind(&Colossus_subscriber_to_video::configuration_data_callback, this, _1));
    fft_data_subscriber =
        Node::create_subscription<interfaces::msg::FftDataMessage>(
            "radar_data/fft_data",
            1600,
            std::bind(&Colossus_subscriber_to_video::fft_data_callback, this, _1));
}

void Colossus_subscriber_to_video::configuration_data_callback(const interfaces::msg::ConfigurationDataMessage::SharedPtr msg) const {
    RCLCPP_INFO(Node::get_logger(), "Configuration Data recieved");
    RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", msg->azimuth_samples);
    azimuth_samples = msg->azimuth_samples;
    RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", msg->encoder_size);
    encoder_size = msg->encoder_size;
    RCLCPP_INFO(Node::get_logger(), "Bin Size: %f", msg->bin_size);
    RCLCPP_INFO(Node::get_logger(), "Range In Bins: %i", msg->range_in_bins);
    RCLCPP_INFO(Node::get_logger(), "Expected Rotation Rate: %i", msg->expected_rotation_rate);

    video_width = msg->range_in_bins;
    video_height = msg->azimuth_samples;
    video_writer.open("output_videos/radar_output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), msg->expected_rotation_rate, Size(azimuth_samples, azimuth_samples), true);
    config_data_received = true;
}
void Colossus_subscriber_to_video::fft_data_callback(const interfaces::msg::FftDataMessage::SharedPtr msg) const {
    if (!config_data_received) {
        RCLCPP_INFO(Node::get_logger(), "No Configuration Data Received");
        return;
    }

    current_bearing = ((double)msg->azimuth / (double)encoder_size) * (double)azimuth_samples;

    int max_index = min((int)msg->data_length, (int)azimuth_samples);
    int matrix_max_index = radar_image.rows * radar_image.cols * radar_image.channels();
    for (int i = 0; i < max_index; i++) {
        int index = i * 1 + current_bearing * radar_image.step + 1;
        if (index < matrix_max_index) {
            image_ptr[index] = static_cast<int>(msg->data[i]);
        }
    }

    if (msg->azimuth < last_azimuth) {
        Mat recovered_lin_polar_img;
        Point2f center((float)radar_image.cols / 2, (float)radar_image.rows / 2);
        double max_radius = min(center.y, center.x);
        linearPolar(radar_image, recovered_lin_polar_img, center, max_radius, INTER_LINEAR + WARP_FILL_OUTLIERS + WARP_INVERSE_MAP);
        Mat normalised_image(Size(azimuth_samples, azimuth_samples), CV_8UC1, Scalar(0, 0));
        normalize(recovered_lin_polar_img, normalised_image, 0, 255, NORM_MINMAX);
        Mat rotated_image(Size(azimuth_samples, azimuth_samples), CV_8UC1, Scalar(0, 0));
        rotate(normalised_image, rotated_image, ROTATE_90_COUNTERCLOCKWISE);
        Mat channels[3] = { blank_image, rotated_image, blank_image };
        Mat merged_data;
        merge(channels, 3, merged_data);
        video_writer.write(merged_data);
    }
    last_azimuth = msg->azimuth;

    //RCLCPP_INFO(Node::get_logger(), "FFT Data Received");
    //RCLCPP_INFO(Node::get_logger(), "Angle: %f", msg->angle);
    //RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", encoder_size);
    //RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", azimuth_samples);
    //RCLCPP_INFO(Node::get_logger(), "Azimuth: %i", msg->azimuth);
    //RCLCPP_INFO(Node::get_logger(), "Current Bearing: %i", current_bearing);
    //RCLCPP_INFO(Node::get_logger(), "Sweep Counter: %i", msg->sweep_counter);
    //RCLCPP_INFO(Node::get_logger(), "NTP Seconds: : %i", msg->ntp_seconds);
    //RCLCPP_INFO(Node::get_logger(), "NTP Split Seconds: %i", msg->ntp_split_seconds);
}