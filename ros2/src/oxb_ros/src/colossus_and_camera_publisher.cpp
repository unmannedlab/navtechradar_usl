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
#include "net_conversion.h"

using namespace Navtech::Utility;
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

    rclcpp::QoS qos_radar_configuration_publisher(radar_configuration_queue_size);
    qos_radar_configuration_publisher.reliable();

    configuration_data_publisher =
    Node::create_publisher<interfaces::msg::ConfigurationDataMessage>(
    "radar_data/configuration_data",
    qos_radar_configuration_publisher);

    rclcpp::QoS qos_radar_fft_publisher(radar_fft_queue_size);
    qos_radar_fft_publisher.reliable();

    fft_data_publisher =
    Node::create_publisher<interfaces::msg::FftDataMessage>(
    "radar_data/fft_data",
    qos_radar_fft_publisher);

    rclcpp::QoS qos_camera_configuration_publisher(camera_configuration_queue_size);
    qos_camera_configuration_publisher.reliable();

    camera_configuration_publisher =
    Node::create_publisher<interfaces::msg::CameraConfigurationMessage>(
    "camera_data/camera_configuration_data",
    qos_camera_configuration_publisher);

    rclcpp::QoS qos_camera_image_publisher(camera_image_queue_size);
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
    message.angle = to_vector(to_uint64_host(data->angle));
    message.azimuth = to_vector(to_uint16_network(data->azimuth));
    message.sweep_counter = to_vector(to_uint16_network(data->sweep_counter));
    message.ntp_seconds = to_vector(to_uint32_network(data->ntp_seconds));
    message.ntp_split_seconds = to_vector(to_uint32_network(data->ntp_split_seconds));
    message.data = data->data;
    message.data_length = to_vector(to_uint16_network(data->data.size()));

	if (camera_message.height <= 0 || camera_message.width <= 0)
	{
        return;
	}

    // On every radar rotation, send out the latest camera frame
    if (data->azimuth < last_azimuth) {
        rotation_count++;
        rotated_once = true;
        camera_image_publisher->publish(camera_message);
    }
    last_azimuth = data->azimuth;

    if (rotation_count >= config_publish_count) {
        configuration_data_publisher->publish(config_message);
        rotation_count = 0;
    }

    if (!rotated_once) {
        return;
    }
	
    fft_data_publisher->publish(message);
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

    azimuth_samples = data->azimuth_samples;
    config_message.azimuth_samples = to_vector(to_uint16_network(data->azimuth_samples));
    config_message.encoder_size = to_vector(to_uint16_network(data->encoder_size));
    config_message.bin_size = to_vector(to_uint64_host(data->bin_size));
    config_message.range_in_bins = to_vector(to_uint16_network(data->range_in_bins));
    config_message.expected_rotation_rate = to_vector(to_uint16_network(data->expected_rotation_rate));
    fps = data->expected_rotation_rate;
    configuration_data_publisher->publish(config_message);

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

    if ((!configuration_sent) || (frame_count >= config_publish_count)) {
        auto config_message = interfaces::msg::CameraConfigurationMessage();
        config_message.width = image.cols;
        config_message.height = image.rows;
        config_message.channels = image.channels();
        config_message.fps = fps;
        camera_configuration_publisher->publish(config_message);
        configuration_sent = true;
        frame_count = 0;
    }

    frame_count++;

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