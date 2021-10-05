#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <math.h>

#include "interfaces/msg/configuration_data_message.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "radar_client.h"
#include "point_cloud_publisher.h"
#include "net_conversion.h"

Point_cloud_publisher::Point_cloud_publisher():Node{ "point_cloud_publisher" }
{
    declare_parameter("radar_ip", "");
    declare_parameter("radar_port", 0);
    declare_parameter("start_azimuth", 0);
    declare_parameter("end_azimuth", 0);
    declare_parameter("start_bin", 0);
    declare_parameter("end_bin", 0);
    declare_parameter("power_threshold", 0);

    radar_ip = get_parameter("radar_ip").as_string();
    radar_port = get_parameter("radar_port").as_int();
    start_azimuth = get_parameter("start_azimuth").as_int();
    end_azimuth = get_parameter("end_azimuth").as_int();
    start_bin = get_parameter("start_bin").as_int();
    end_bin = get_parameter("end_bin").as_int();
    power_threshold = get_parameter("power_threshold").as_int();

    rclcpp::QoS qos_radar_configuration_publisher(radar_configuration_queue_size);
    qos_radar_configuration_publisher.reliable();

    configuration_data_publisher =
    Node::create_publisher<interfaces::msg::ConfigurationDataMessage>(
        "radar_data/configuration_data",
        qos_radar_configuration_publisher);

    rclcpp::QoS qos_point_cloud_publisher(radar_point_cloud_queue_size);
    qos_point_cloud_publisher.reliable();

    point_cloud_publisher =
    Node::create_publisher<sensor_msgs::msg::PointCloud2>(
        "radar_data/point_cloud",
        qos_point_cloud_publisher);
}

void Point_cloud_publisher::publish_point_cloud(const Navtech::Fft_data::Pointer& data)
{
    auto message = sensor_msgs::msg::PointCloud2();
    message.header = std_msgs::msg::Header();
    message.header.stamp.sec = data->ntp_seconds;
    message.header.stamp.nanosec = data->ntp_split_seconds;
    message.header.frame_id = "point_cloud";

    message.height = 1;
    message.width = 3;
    uint8_t data_type = 7;

    auto x_field = sensor_msgs::msg::PointField();
    x_field.name = "x";
    x_field.offset = 0 * 4;
    x_field.datatype = data_type;
    x_field.count = message.width;

    auto y_field = sensor_msgs::msg::PointField();
    y_field.name = "y";
    y_field.offset = 1 * 4;
    y_field.datatype = data_type;
    y_field.count = message.width;

    auto z_field = sensor_msgs::msg::PointField();
    z_field.name = "z";
    z_field.offset = 2 * 4;
    z_field.datatype = data_type;
    z_field.count = message.width;

    message.fields = std::vector<sensor_msgs::msg::PointField>{x_field, y_field, z_field};

    message.is_bigendian = false;
    message.point_step = 3 * 4;
    message.row_step = message.point_step * message.width;


    float five = 5.0;
    uint8_t* five_char = reinterpret_cast<uint8_t*>(&five);

    float ten = 10.0;
    uint8_t* ten_char = reinterpret_cast<uint8_t*>(&ten);

    float fifteen = 15.0;
    uint8_t* fifteen_char = reinterpret_cast<uint8_t*>(&fifteen);


    message.data = std::vector<uint8_t>{ five_char[0],five_char[1],five_char[2],five_char[3],
        five_char[0],five_char[1],five_char[2],five_char[3],
        five_char[0],five_char[1],five_char[2],five_char[3],
    
    ten_char[0],ten_char[1],ten_char[2],ten_char[3],
        ten_char[0],ten_char[1],ten_char[2],ten_char[3],
        ten_char[0],ten_char[1],ten_char[2],ten_char[3],
    
    fifteen_char[0],fifteen_char[1],fifteen_char[2],fifteen_char[3],
        fifteen_char[0],fifteen_char[1],fifteen_char[2],fifteen_char[3],
        fifteen_char[0],fifteen_char[1],fifteen_char[2],fifteen_char[3] };
    message.is_dense = true;

    point_cloud_publisher->publish(message);
}

struct more_than {
    more_than(int limit) : _limit(limit) {}

    bool operator()(int val) {
        return val > _limit;
    }

    int _limit;
};

void Point_cloud_publisher::fft_data_handler(const Navtech::Fft_data::Pointer& data)
{
    auto itr = find_if(data->data.begin(), data->data.end(), more_than(power_threshold));
    auto first_peak = distance(data->data.begin(), itr);
    if (itr == data->data.end()) {
        first_peak = std::distance(data->data.begin(), itr - 1);
    }
    float range = bin_size * first_peak;
    float intensity = data->data[first_peak];
    int azimuth_index = static_cast<int>(data->angle / (360.0 / azimuth_samples));
    azimuth_index = azimuth_samples - azimuth_index;
    if ((azimuth_index >= start_azimuth) && (azimuth_index < end_azimuth)) {
        range_values[azimuth_index] = range;
        intensity_values[azimuth_index] = intensity;
    }
    else{
        range_values[azimuth_index] = 0;
        intensity_values[azimuth_index] = 0;
    }

    if (data->azimuth < last_azimuth) {
        rotation_count++;
        rotated_once = true;
        Point_cloud_publisher::publish_point_cloud(data);
    }
    last_azimuth = data->azimuth;

    if (rotation_count >= config_publish_count) {
        power_threshold = get_parameter("power_threshold").as_int();
        start_azimuth = get_parameter("start_azimuth").as_int();
        end_azimuth = get_parameter("end_azimuth").as_int();
        configuration_data_publisher->publish(config_message);
        rotation_count = 0;
    }

    if (!rotated_once) {
        return;
    }
}

void Point_cloud_publisher::configuration_data_handler(const Navtech::Configuration_data::Pointer& data){
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

    range_values.resize(end_azimuth - start_azimuth);
    intensity_values.resize(end_azimuth - start_azimuth);

    RCLCPP_INFO(Node::get_logger(), "Starting laser scan publisher");
    RCLCPP_INFO(Node::get_logger(), "Start azimuth: %i", start_azimuth);
    RCLCPP_INFO(Node::get_logger(), "End azimuth: %i", end_azimuth);
    RCLCPP_INFO(Node::get_logger(), "Start bin: %i", start_bin);
    RCLCPP_INFO(Node::get_logger(), "End bin: %i", end_bin);
    RCLCPP_INFO(Node::get_logger(), "Power threshold: %i", power_threshold);

    radar_client->start_fft_data();
}