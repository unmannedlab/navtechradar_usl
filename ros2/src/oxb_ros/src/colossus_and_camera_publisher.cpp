#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

#include "interfaces/msg/configuration_data_message.hpp"
#include "interfaces/msg/fft_data_message.hpp"
#include "interfaces/msg/camera_configuration_message.hpp"
#include "sensor_msgs/msg/image.hpp"
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

    rclcpp::QoS qos_radar_configuration_publisher(1);
    qos_radar_configuration_publisher.reliable();

    configuration_data_publisher =
    Node::create_publisher<interfaces::msg::ConfigurationDataMessage>(
    "radar_data/configuration_data",
    qos_radar_configuration_publisher);

    rclcpp::QoS qos_radar_fft_publisher(400);
    qos_radar_fft_publisher.reliable();

    fft_data_publisher =
    Node::create_publisher<interfaces::msg::FftDataMessage>(
    "radar_data/fft_data",
    qos_radar_fft_publisher);

    rclcpp::QoS qos_camera_configuration_publisher(1);
    qos_camera_configuration_publisher.reliable();

    camera_configuration_publisher =
    Node::create_publisher<interfaces::msg::CameraConfigurationMessage>(
    "camera_data/camera_configuration_data",
    qos_camera_configuration_publisher);

    rclcpp::QoS qos_camera_image_publisher(25);
    qos_camera_image_publisher.reliable();

    camera_image_publisher =
    Node::create_publisher<sensor_msgs::msg::Image>(
    "camera_data/camera_image_data",
    qos_camera_image_publisher);
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

	if (camera_message.height <= 0 || camera_message.width <= 0)
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
    vector<uint8_t> vector_buffer(image.ptr(0), image.ptr(0) + buffer_length);

    //RCLCPP_INFO(Node::get_logger(), "Image buffer size: %li", buffer_length);

    if (!configuration_sent) {
        auto config_message = interfaces::msg::CameraConfigurationMessage();
        config_message.width = image.cols;
        config_message.height = image.rows;
        config_message.channels = image.channels();
        config_message.fps = fps;
        camera_configuration_publisher->publish(config_message);
        configuration_sent = true;
    }

    camera_message.header = std_msgs::msg::Header();
    camera_message.header.stamp = node->get_clock()->now();

    camera_message.height = image.rows;
    camera_message.width = image.cols;
    camera_message.encoding = "8UC3";
    camera_message.is_bigendian = true;
    camera_message.step = image.step;
    camera_message.data = std::move(vector_buffer);
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