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

    message.height = range_in_bins;
    message.width = azimuth_samples;
    const uint8_t data_type = 7;
    const uint8_t num_bytes = 4; //float32 as bytes

    auto x_field = sensor_msgs::msg::PointField();
    x_field.name = "x";
    x_field.offset = 0 * num_bytes;
    x_field.datatype = data_type;
    x_field.count = message.height * message.width;

    auto y_field = sensor_msgs::msg::PointField();
    y_field.name = "y";
    y_field.offset = 1 * num_bytes;
    y_field.datatype = data_type;
    y_field.count = message.height * message.width;

    auto z_field = sensor_msgs::msg::PointField();
    z_field.name = "z";
    z_field.offset = 2 * num_bytes;
    z_field.datatype = data_type;
    z_field.count = message.height * message.width;

    auto intensity_field = sensor_msgs::msg::PointField();
    intensity_field.name = "intensity";
    intensity_field.offset = 3 * num_bytes;
    intensity_field.datatype = data_type;
    intensity_field.count = message.height * message.width;

    message.fields = std::vector<sensor_msgs::msg::PointField>{x_field, y_field, z_field, intensity_field};

    message.is_bigendian = false;
    message.point_step = 4 * num_bytes;
    message.row_step = message.point_step * message.width;

    std::vector<uint8_t> data_vector;
    data_vector.reserve(message.height * message.row_step);
    for (int i = 0; i < message.height * message.width; i++) {
        auto vec = Point_cloud_publisher::floats_to_uint8_t_vector(bin_values[i], azimuth_values[i], 0, intensity_values[i]);
        data_vector.insert(data_vector.end(), vec.begin(), vec.end());
    }
    message.data = data_vector;
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

std::vector<uint8_t> Point_cloud_publisher::floats_to_uint8_t_vector(float x, float y, float z, float intensity) {
    uint8_t* chars_x = reinterpret_cast<uint8_t*>(&x);
    uint8_t* chars_y = reinterpret_cast<uint8_t*>(&y);
    uint8_t* chars_z = reinterpret_cast<uint8_t*>(&z);
    uint8_t* chars_intensity = reinterpret_cast<uint8_t*>(&intensity);
    return std::vector<uint8_t>{chars_x[0], chars_x[1], chars_x[2], chars_x[3],
        chars_y[0], chars_y[1], chars_y[2], chars_y[3],
        chars_z[0], chars_z[1], chars_z[2], chars_z[3],
        chars_intensity[0], chars_intensity[1], chars_intensity[2], chars_intensity[3]};
}

void Point_cloud_publisher::fft_data_handler(const Navtech::Fft_data::Pointer& data){

    int azimuth_index = static_cast<int>(data->angle / (360.0 / azimuth_samples));

    // To adjust radar start azimuth, for sake of visualisation
    // Note - this value will be different for every setup!
    // Values based on 0 angle of radar, and surrounding landscape
    int adjusted_azimuth_index = azimuth_index + 200;
    if (adjusted_azimuth_index >= azimuth_samples) {
        adjusted_azimuth_index = adjusted_azimuth_index - 400;
    }

    if ((azimuth_index >= start_azimuth) && (azimuth_index < end_azimuth)) {

        for (int bin_index = 0; bin_index < data->data.size(); bin_index++) {
            if (data->data[bin_index] > power_threshold) {
                azimuth_values[(adjusted_azimuth_index * range_in_bins) + bin_index] = adjusted_azimuth_index;
                bin_values[(adjusted_azimuth_index * range_in_bins) + bin_index] = bin_index;
                intensity_values[(adjusted_azimuth_index * range_in_bins) + bin_index] = data->data[bin_index];
            }
            else {
                azimuth_values[(adjusted_azimuth_index * range_in_bins) + bin_index] = adjusted_azimuth_index;
                bin_values[(adjusted_azimuth_index * range_in_bins) + bin_index] = bin_index;
                intensity_values[(adjusted_azimuth_index * range_in_bins) + bin_index] = 0;
            }
        }
    }
    else {
        for (int bin_index = 0; bin_index < data->data.size(); bin_index++) {
            azimuth_values[(adjusted_azimuth_index * range_in_bins) + bin_index] = adjusted_azimuth_index;
            bin_values[(adjusted_azimuth_index * range_in_bins) + bin_index] = bin_index;
            intensity_values[(adjusted_azimuth_index * range_in_bins) + bin_index] = 0;
        }
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

    azimuth_values.resize((end_azimuth - start_azimuth) * (end_bin - start_bin));
    bin_values.resize((end_azimuth - start_azimuth) * (end_bin - start_bin));
    intensity_values.resize((end_azimuth - start_azimuth) * (end_bin - start_bin));

    RCLCPP_INFO(Node::get_logger(), "Starting laser scan publisher");
    RCLCPP_INFO(Node::get_logger(), "Start azimuth: %i", start_azimuth);
    RCLCPP_INFO(Node::get_logger(), "End azimuth: %i", end_azimuth);
    RCLCPP_INFO(Node::get_logger(), "Start bin: %i", start_bin);
    RCLCPP_INFO(Node::get_logger(), "End bin: %i", end_bin);
    RCLCPP_INFO(Node::get_logger(), "Power threshold: %i", power_threshold);

    radar_client->start_fft_data();
}