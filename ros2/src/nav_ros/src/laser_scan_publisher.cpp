#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <math.h>

#include "interfaces/msg/configuration_data_message.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "radar_client.h"
#include "laser_scan_publisher.h"
#include "net_conversion.h"

Laser_scan_publisher::Laser_scan_publisher():Node{ "laser_scan_publisher" }
{
    declare_parameter("radar_ip", "");
    declare_parameter("radar_port", 0);
    declare_parameter("start_azimuth", 0);
    declare_parameter("end_azimuth", 0);
    declare_parameter("start_bin", 0);
    declare_parameter("end_bin", 0);
    declare_parameter("power_threshold", 0);
    declare_parameter("azimuth_offset", 0);

    radar_ip = get_parameter("radar_ip").as_string();
    radar_port = get_parameter("radar_port").as_int();
    start_azimuth = get_parameter("start_azimuth").as_int();
    end_azimuth = get_parameter("end_azimuth").as_int();
    start_bin = get_parameter("start_bin").as_int();
    end_bin = get_parameter("end_bin").as_int();
    power_threshold = get_parameter("power_threshold").as_int();
    azimuth_offset = get_parameter("azimuth_offset").as_int();

    rclcpp::QoS qos_radar_configuration_publisher(radar_configuration_queue_size);
    qos_radar_configuration_publisher.reliable();

    configuration_data_publisher =
    Node::create_publisher<interfaces::msg::ConfigurationDataMessage>(
        "radar_data/configuration_data",
        qos_radar_configuration_publisher);

    rclcpp::QoS qos_laser_scan_publisher(radar_laser_scan_queue_size);
    qos_laser_scan_publisher.reliable();

    laser_scan_publisher =
    Node::create_publisher<sensor_msgs::msg::LaserScan>(
        "radar_data/laser_scan",
        qos_laser_scan_publisher);
}

void Laser_scan_publisher::publish_laser_scan(const Navtech::Fft_data::Pointer& data)
{
    auto message = sensor_msgs::msg::LaserScan();

    message.header = std_msgs::msg::Header();
    message.header.stamp.sec = data->ntp_seconds;
    message.header.stamp.nanosec = data->ntp_split_seconds;
    message.header.frame_id = "laser_frame";

    message.angle_min = M_PI / 180 * 360 / azimuth_samples * start_azimuth;
    message.angle_max = M_PI / 180 * 360 / azimuth_samples * end_azimuth;
    message.angle_increment = M_PI / 180 * 360 / azimuth_samples;
    message.time_increment = 1.0 / expected_rotation_rate / azimuth_samples;
    message.scan_time = 1.0 / expected_rotation_rate;
    message.range_min = 1.0 * bin_size;
    message.range_max = range_in_bins * bin_size;
    message.ranges.resize(azimuth_samples);
    message.ranges = range_values;
    message.intensities.resize(azimuth_samples);
    message.intensities = intensity_values;
    laser_scan_publisher->publish(message);
}

struct more_than {
    more_than(int limit) : _limit(limit) {}

    bool operator()(int val) {
        return val > _limit;
    }

    int _limit;
};

void Laser_scan_publisher::fft_data_handler(const Navtech::Fft_data::Pointer& data)
{
    auto itr = find_if(data->data.begin(), data->data.end(), more_than(power_threshold));
    auto first_peak_bin_index = distance(data->data.begin(), itr);
    if (itr == data->data.end()) {
        first_peak_bin_index = std::distance(data->data.begin(), itr - 1);
    }
    float range = bin_size * first_peak_bin_index;
    float intensity = data->data[first_peak_bin_index];
    int azimuth_index = static_cast<int>(data->angle / (360.0 / azimuth_samples));

    // To adjust radar start azimuth, for sake of visualisation
    // Note - this value will be different for every setup!
    // Values based on 0 angle of radar, and surrounding landscape
    int adjusted_azimuth_index = azimuth_index + azimuth_offset;
    if (adjusted_azimuth_index >= azimuth_samples) {
        adjusted_azimuth_index = adjusted_azimuth_index - azimuth_samples;
    }

    if ((azimuth_index >= start_azimuth) && (azimuth_index < end_azimuth)) {
        if ((first_peak_bin_index >= start_bin) && (first_peak_bin_index < end_bin)) {
            range_values[adjusted_azimuth_index] = range;
            intensity_values[adjusted_azimuth_index] = intensity;
        }
        else {
            range_values[adjusted_azimuth_index] = 0;
            intensity_values[adjusted_azimuth_index] = 0;
        }
    }
    else{
        range_values[adjusted_azimuth_index] = 0;
        intensity_values[adjusted_azimuth_index] = 0;
    }

    if (data->azimuth < last_azimuth) {
        rotation_count++;
        rotated_once = true;
        Laser_scan_publisher::publish_laser_scan(data);
    }
    last_azimuth = data->azimuth;

    if (rotation_count >= config_publish_count) {
        azimuth_offset = get_parameter("azimuth_offset").as_int();
        power_threshold = get_parameter("power_threshold").as_int();
        start_azimuth = get_parameter("start_azimuth").as_int();
        end_azimuth = get_parameter("end_azimuth").as_int();
        start_bin = get_parameter("start_bin").as_int();
        end_bin = get_parameter("end_bin").as_int();
        configuration_data_publisher->publish(config_message);
        rotation_count = 0;
    }

    if (!rotated_once) {
        return;
    }
}

void Laser_scan_publisher::configuration_data_handler(const Navtech::Configuration_data::Pointer& data){
    RCLCPP_INFO(Node::get_logger(), "Configuration Data Received");
    RCLCPP_INFO(Node::get_logger(), "Azimuth Samples: %i", data->azimuth_samples);
    RCLCPP_INFO(Node::get_logger(), "Encoder Size: %i", data->encoder_size);
    RCLCPP_INFO(Node::get_logger(), "Bin Size: %f", data->bin_size);
    RCLCPP_INFO(Node::get_logger(), "Range In Bins: : %i", data->range_in_bins);
    RCLCPP_INFO(Node::get_logger(), "Expected Rotation Rate: %i", data->expected_rotation_rate);
    RCLCPP_INFO(Node::get_logger(), "Publishing Configuration Data");

    azimuth_samples = data->azimuth_samples;
    bin_size = data->bin_size;
    range_in_bins = data->range_in_bins;
    expected_rotation_rate = data->expected_rotation_rate;
    config_message.azimuth_samples = Navtech::Utility::to_vector(Navtech::Utility::to_uint16_network(data->azimuth_samples));
    config_message.encoder_size = Navtech::Utility::to_vector(Navtech::Utility::to_uint16_network(data->encoder_size));
    config_message.bin_size = Navtech::Utility::to_vector(Navtech::Utility::to_uint64_host(data->bin_size));
    config_message.range_in_bins = Navtech::Utility::to_vector(Navtech::Utility::to_uint16_network(data->range_in_bins));
    config_message.expected_rotation_rate = Navtech::Utility::to_vector(Navtech::Utility::to_uint16_network(data->expected_rotation_rate));
    configuration_data_publisher->publish(config_message);

    range_values.resize(azimuth_samples);
    intensity_values.resize(azimuth_samples);

    RCLCPP_INFO(Node::get_logger(), "Starting laser scan publisher");
    RCLCPP_INFO(Node::get_logger(), "Start azimuth: %i", start_azimuth);
    RCLCPP_INFO(Node::get_logger(), "End azimuth: %i", end_azimuth);
    RCLCPP_INFO(Node::get_logger(), "Start bin: %i", start_bin);
    RCLCPP_INFO(Node::get_logger(), "End bin: %i", end_bin);
    RCLCPP_INFO(Node::get_logger(), "Power threshold: %i", power_threshold);
    RCLCPP_INFO(Node::get_logger(), "Azimuth offset: %i", azimuth_offset);

    radar_client->start_fft_data();
}