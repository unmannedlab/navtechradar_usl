#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

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

Colossus_and_camera_publisher::Colossus_and_camera_publisher():Node{ "colossus_and_camera_publisher" }
{
    declare_parameter("radar_ip", "");
    declare_parameter("radar_port", 0);
    declare_parameter("camera_url", "");

    radar_ip = get_parameter("radar_ip").as_string();
    radar_port = get_parameter("radar_port").as_int();
    camera_url = get_parameter("camera_url").as_string();

    configuration_data_publisher =
    Node::create_publisher<interfaces::msg::ConfigurationDataMessage>(
    "radar_data/configuration_data",
    1);

    fft_data_publisher =
    Node::create_publisher<interfaces::msg::FftDataMessage>(
    "radar_data/fft_data",
     10);

    camera_image_publisher =
    Node::create_publisher<interfaces::msg::CameraImageMessage>(
    "camera_data/image_data",
    4);
}

void Colossus_and_camera_publisher::fft_data_handler(const Fft_data::Pointer& data)
{
    //RCLCPP_INFO(Node::get_logger(), "Publishing FFT Data");

    auto message = interfaces::msg::FftDataMessage();
    message.angle = data->angle;
    message.azimuth = data->azimuth;
    message.sweep_counter = data->sweep_counter;
    message.ntp_seconds = data->ntp_seconds;
    message.ntp_split_seconds = data->ntp_split_seconds;
    message.data = data->data;
    message.data_length = data->data.size();

    //RCLCPP_INFO(Node::get_logger(), "Data 0: %u", static_cast<int>(data->Data[0]));
    //RCLCPP_INFO(Node::get_logger(), "Data 1: %u", static_cast<int>(data->Data[1]));
    //RCLCPP_INFO(Node::get_logger(), "Data 2: %u", static_cast<int>(data->Data[2]));
    //RCLCPP_INFO(Node::get_logger(), "Data 3: %u", static_cast<int>(data->Data[3]));
    //RCLCPP_INFO(Node::get_logger(), "Data 4: %u", static_cast<int>(data->Data[4]));

	if (camera_message.image_rows <= 0 || camera_message.image_cols <= 0 || camera_message.image_channels <= 0 || camera_message.image_fps <= 0)
	{
        return;
	}

    // On every radar rotation, send out the latest camera frame
    if (data->azimuth < last_azimuth) {
        rotated_once = true;
        camera_image_publisher->publish(camera_message);
    }
    last_azimuth = data->azimuth;

    if (!rotated_once) {
        return;
    }
	
    fft_data_publisher->publish(message);
    //RCLCPP_INFO(Node::get_logger(), "Azimuth: %i", data->azimuth);
}

void Colossus_and_camera_publisher::configuration_data_handler(const Configuration_data::Pointer& data)
{
    RCLCPP_INFO(Node::get_logger(), "Configuration Data Received");
    RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", data->azimuth_samples);
    RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", data->encoder_size);
    RCLCPP_INFO(Node::get_logger(), "Bin Size: %f", data->bin_size);
    RCLCPP_INFO(Node::get_logger(), "Range In Bins: : %i", data->range_in_bins);
    RCLCPP_INFO(Node::get_logger(), "Expected Rotation Rate: %i", data->expected_rotation_rate);
    RCLCPP_INFO(Node::get_logger(), "Publishing Configuration Data");

    auto message = interfaces::msg::ConfigurationDataMessage();
    azimuth_samples = data->azimuth_samples;
    message.azimuth_samples = data->azimuth_samples;
    message.encoder_size = data->encoder_size;
    message.bin_size = data->bin_size;
    message.range_in_bins = data->range_in_bins;
    message.expected_rotation_rate = data->expected_rotation_rate;
    fps = data->expected_rotation_rate;
    configuration_data_publisher->publish(message);

    radar_client->start_fft_data();
}

void Colossus_and_camera_publisher::camera_image_handler(Mat image)
{
    //RCLCPP_INFO(Node::get_logger(), "Publishing Camera Image Data");
    //RCLCPP_INFO(Node::get_logger(), "Mat rows: %i", image.rows);
    //RCLCPP_INFO(Node::get_logger(), "Mat columns: %i", image.cols);
    //RCLCPP_INFO(Node::get_logger(), "Mat size: %i", image.rows * image.cols);
    //RCLCPP_INFO(Node::get_logger(), "Mat type: %i", image.type());

    auto buffer_length = image.cols * image.rows * sizeof(uint8_t) * 3;
    vector<uint8_t> vectorBuffer(image.ptr(0), image.ptr(0) + buffer_length);

    auto nanosec_since_epoch = duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
    auto sec_since_epoch = duration_cast<seconds>(steady_clock::now().time_since_epoch()).count();

    //RCLCPP_INFO(Node::get_logger(), "Image buffer size: %li", buffer_length);

    camera_message.image_data = std::move(vectorBuffer);
    camera_message.image_rows = image.rows;
    camera_message.image_cols = image.cols;
    camera_message.image_channels = image.channels();
    camera_message.image_fps = fps;
    camera_message.ntp_seconds = sec_since_epoch;
    camera_message.ntp_split_seconds = nanosec_since_epoch - (sec_since_epoch * 1000000000);
}

void Colossus_and_camera_publisher::cleanup_and_shutdown()
{
    radar_client->stop_fft_data();
    radar_client->set_configuration_data_callback();
    radar_client->set_fft_data_callback();
    radar_client->stop();
    shutdown();
    RCLCPP_INFO(Node::get_logger(), "Stopped radar client");
    RCLCPP_INFO(Node::get_logger(), "Stopped camera publisher");
}