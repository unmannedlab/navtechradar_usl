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
#include "navigation/peak_finder.h"
#include "navigation_mode_point_cloud_publisher.h"
#include "net_conversion.h"

Navigation_mode_point_cloud_publisher::Navigation_mode_point_cloud_publisher() :Node{ "navigation_mode_point_cloud_publisher" }
{
    declare_parameter("radar_ip", "");
    declare_parameter("radar_port", 0);
    declare_parameter("start_azimuth", 0);
    declare_parameter("end_azimuth", 0);
    declare_parameter("start_bin", 0);
    declare_parameter("end_bin", 0);
    declare_parameter("azimuth_offset", 0);

    declare_parameter("bins_to_operate_on", 0);
    declare_parameter("min_bin", 0);
    declare_parameter("power_threshold", 0.0);
    declare_parameter("max_peaks_per_azimuth", 0);
    declare_parameter("process_locally", false);

    radar_ip = get_parameter("radar_ip").as_string();
    radar_port = get_parameter("radar_port").as_int();
    start_azimuth = get_parameter("start_azimuth").as_int();
    end_azimuth = get_parameter("end_azimuth").as_int();
    start_bin = get_parameter("start_bin").as_int();
    end_bin = get_parameter("end_bin").as_int();
    azimuth_offset = get_parameter("azimuth_offset").as_int();

    bins_to_operate_on = get_parameter("bins_to_operate_on").as_int();
    min_bin = get_parameter("min_bin").as_int();
    power_threshold = get_parameter("power_threshold").as_double();
    max_peaks_per_azimuth = get_parameter("max_peaks_per_azimuth").as_int();
    process_locally = get_parameter("process_locally").as_bool();

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

void Navigation_mode_point_cloud_publisher::publish_point_cloud(const Navtech::Navigation_data::Pointer& data)
{
    auto message = sensor_msgs::msg::PointCloud2();
    message.header = std_msgs::msg::Header();
    message.header.stamp.sec = data->ntp_seconds;
    message.header.stamp.nanosec = data->ntp_split_seconds;
    message.header.frame_id = "point_cloud";

    message.height = 1;
    message.width = intensity_values.size();
    constexpr uint8_t data_type = 7;
    constexpr uint8_t num_bytes = 4; //float32 as bytes

    auto x_field = sensor_msgs::msg::PointField();
    x_field.name = "x";
    x_field.offset = 0 * num_bytes;
    x_field.datatype = data_type;
    x_field.count = intensity_values.size();

    auto y_field = sensor_msgs::msg::PointField();
    y_field.name = "y";
    y_field.offset = 1 * num_bytes;
    y_field.datatype = data_type;
    y_field.count = intensity_values.size();

    auto z_field = sensor_msgs::msg::PointField();
    z_field.name = "z";
    z_field.offset = 2 * num_bytes;
    z_field.datatype = data_type;
    z_field.count = intensity_values.size();

    auto intensity_field = sensor_msgs::msg::PointField();
    intensity_field.name = "intensity";
    intensity_field.offset = 3 * num_bytes;
    intensity_field.datatype = data_type;
    intensity_field.count = intensity_values.size();

    message.fields = std::vector<sensor_msgs::msg::PointField>{ x_field, y_field, z_field, intensity_field };

    message.is_bigendian = false;
    message.point_step = 4 * num_bytes;
    message.row_step = message.point_step * message.width;

    std::vector<uint8_t> data_vector;
    data_vector.reserve(intensity_values.size());
    for (int i = 0; i < intensity_values.size(); i++) {

        float current_azimuth = (azimuth_values[i] * 0.9) * (M_PI / 180.0);
        float point_x = bin_values[i] * cos(current_azimuth);
        float point_y = bin_values[i] * sin(current_azimuth);

        auto vec = Navigation_mode_point_cloud_publisher::floats_to_uint8_t_vector(point_x, point_y, 0, intensity_values[i]);
        data_vector.insert(data_vector.end(), vec.begin(), vec.end());
    }
    message.data = std::move(data_vector);
    message.is_dense = true;

    point_cloud_publisher->publish(message);
}

std::vector<uint8_t> Navigation_mode_point_cloud_publisher::floats_to_uint8_t_vector(float x, float y, float z, float intensity) {
    uint8_t* chars_x = reinterpret_cast<uint8_t*>(&x);
    uint8_t* chars_y = reinterpret_cast<uint8_t*>(&y);
    uint8_t* chars_z = reinterpret_cast<uint8_t*>(&z);
    uint8_t* chars_intensity = reinterpret_cast<uint8_t*>(&intensity);
    return std::vector<uint8_t>{chars_x[0], chars_x[1], chars_x[2], chars_x[3],
        chars_y[0], chars_y[1], chars_y[2], chars_y[3],
        chars_z[0], chars_z[1], chars_z[2], chars_z[3],
        chars_intensity[0], chars_intensity[1], chars_intensity[2], chars_intensity[3]};
}

void Navigation_mode_point_cloud_publisher::update_navigation_config() {
    RCLCPP_INFO(Node::get_logger(), "Updating navigation config on radar");
    auto navigation_config = Navtech::Navigation_config();
    navigation_config.bins_to_operate_on = bins_to_operate_on;
    navigation_config.min_bin = min_bin;
    navigation_config.navigation_threshold = power_threshold;
    navigation_config.max_peaks_per_azimuth = max_peaks_per_azimuth;
    radar_client->set_navigation_configuration(navigation_config);
}

void Navigation_mode_point_cloud_publisher::navigation_config_data_handler(const Navtech::Navigation_config::Pointer& data) {
    RCLCPP_INFO(Node::get_logger(), "Received navigation config from radar");
    RCLCPP_INFO(Node::get_logger(), "Bins to operate on: %i", data->bins_to_operate_on);
    RCLCPP_INFO(Node::get_logger(), "Min bin: %i", data->min_bin);
    RCLCPP_INFO(Node::get_logger(), "Power threshold: %f", data->navigation_threshold);
    RCLCPP_INFO(Node::get_logger(), "Max peaks per azimuth: %i", data->max_peaks_per_azimuth);
}

void Navigation_mode_point_cloud_publisher::fft_data_handler(const Navtech::Fft_data::Pointer& data) {
    peak_finder->fft_data_handler(data);
}

void Navigation_mode_point_cloud_publisher::target_data_handler(const Navtech::Azimuth_target& target_data) {

    auto data = std::make_shared<Navtech::Navigation_data>();

    data->angle = target_data.angle;
    data->azimuth = target_data.azimuth;
    data->ntp_seconds = target_data.ntp_seconds;
    data->ntp_split_seconds = target_data.ntp_split_seconds;

    for (int t = 0; t < target_data.targets.size(); t++) {
        std::tuple<float, std::uint16_t> peak(target_data.targets[t].range, (std::uint16_t)(target_data.targets[t].power * 10.0));
        data->peaks.push_back(peak);
    }

    navigation_data_handler(data);
}

void Navigation_mode_point_cloud_publisher::navigation_data_handler(const Navtech::Navigation_data::Pointer& data) {

    int azimuth_index = static_cast<int>(data->angle / (360.0 / azimuth_samples));

    // To adjust radar start azimuth, for sake of visualisation
    // Note - this value will be different for every setup!
    // Values based on 0 angle of radar, and surrounding landscape
    int adjusted_azimuth_index = azimuth_index + azimuth_offset;
    if (adjusted_azimuth_index >= azimuth_samples) {
        adjusted_azimuth_index = adjusted_azimuth_index - azimuth_samples;
    }

    if ((azimuth_index >= start_azimuth) && (azimuth_index < end_azimuth)) {

        for (int peak_index = 0; peak_index < data->peaks.size(); peak_index++) {
            float target_range = std::get<float>(data->peaks[peak_index]);
            int bin_index = (int)(target_range / bin_size);
            // TODO - use float not uint16_t, divide power by 10
            uint16_t target_power = std::get<uint16_t>(data->peaks[peak_index]);
            if ((bin_index >= start_bin) && (bin_index < end_bin)) {
                    azimuth_values.push_back(adjusted_azimuth_index);
                    bin_values.push_back(bin_index);
                    intensity_values.push_back(target_power);
            }
        }
    }

    if (data->azimuth < last_azimuth) {
        rotation_count++;
        rotated_once = true;
        Navigation_mode_point_cloud_publisher::publish_point_cloud(data);
        bin_values.clear();
        azimuth_values.clear();
        intensity_values.clear();
    }
    last_azimuth = data->azimuth;

    if (rotation_count >= config_publish_count) {

        int temp_azimuth_offset = get_parameter("azimuth_offset").as_int();
        if (temp_azimuth_offset > azimuth_samples) {
            RCLCPP_INFO(Node::get_logger(), "Azimuth offset of %i is invalid, must be less than or equal to %i", temp_azimuth_offset, azimuth_samples);
            RCLCPP_INFO(Node::get_logger(), "Setting azimuth offset to %i", azimuth_samples);
            set_parameter(rclcpp::Parameter("azimuth_offset", azimuth_samples));
        }
        else {
            azimuth_offset = temp_azimuth_offset;
        }

        int temp_start_azimuth = get_parameter("start_azimuth").as_int();
        if (temp_start_azimuth < 0 || temp_start_azimuth > azimuth_samples) {
            RCLCPP_INFO(Node::get_logger(), "Start azimuth of %i is invalid, must be between 0 and %i", temp_start_azimuth, azimuth_samples);
            RCLCPP_INFO(Node::get_logger(), "Setting start azimuth to %i", 0);
            set_parameter(rclcpp::Parameter("start_azimuth", 0));
        }
        else {
            start_azimuth = temp_start_azimuth;
        }

        int temp_end_azimuth = get_parameter("end_azimuth").as_int();
        if (temp_end_azimuth < 0 || temp_end_azimuth > azimuth_samples) {
            RCLCPP_INFO(Node::get_logger(), "End azimuth of %i is invalid, must be between 0 and %i", temp_end_azimuth, azimuth_samples);
            RCLCPP_INFO(Node::get_logger(), "Setting end azimuth to %i", azimuth_samples);
            set_parameter(rclcpp::Parameter("end_azimuth", azimuth_samples));
        }
        else {
            end_azimuth = temp_end_azimuth;
        }

        int temp_start_bin = get_parameter("start_bin").as_int();
        if (temp_start_bin < 0 || temp_start_bin > range_in_bins) {
            RCLCPP_INFO(Node::get_logger(), "Start bin of %i is invalid, must be between 0 and %i", temp_start_bin, range_in_bins);
            RCLCPP_INFO(Node::get_logger(), "Setting start bin to %i", 0);
            set_parameter(rclcpp::Parameter("start_bin", 0));
        }
        else {
            start_bin = temp_start_bin;
        }

        int temp_end_bin = get_parameter("end_bin").as_int();
        if (temp_end_bin < 0 || temp_end_bin > range_in_bins) {
            RCLCPP_INFO(Node::get_logger(), "End bin of %i is invalid, must be between 0 and %i", temp_end_bin, range_in_bins);
            RCLCPP_INFO(Node::get_logger(), "Setting end bin to %i", range_in_bins);
            set_parameter(rclcpp::Parameter("end_bin", range_in_bins));
        }
        else {
            end_bin = temp_end_azimuth;
        }

        // These params mean a new config has to be sent to the radar
        bool update_radar_navigation_config = false;
        if (get_parameter("bins_to_operate_on").as_int() != bins_to_operate_on)
        {
            int temp_bins_to_operate_on = get_parameter("bins_to_operate_on").as_int();
            if (temp_bins_to_operate_on < 0 || temp_bins_to_operate_on > 10) {
                RCLCPP_INFO(Node::get_logger(), "Bins to operate on of %i is invalid, must be between 0 and %i", temp_bins_to_operate_on, 10);
                RCLCPP_INFO(Node::get_logger(), "Setting bins to operate on to %i", 10);
                set_parameter(rclcpp::Parameter("bins_to_operate_on", bins_to_operate_on));
            }
            else {
                bins_to_operate_on = temp_bins_to_operate_on;
            }
            update_radar_navigation_config = true;
        }
        if (get_parameter("min_bin").as_int() != min_bin)
        {
            int temp_min_bin = get_parameter("min_bin").as_int();
            if (temp_min_bin < 0 || temp_min_bin > range_in_bins) {
                RCLCPP_INFO(Node::get_logger(), "Min bin of %i is invalid, must be between 0 and %i", temp_min_bin, range_in_bins);
                RCLCPP_INFO(Node::get_logger(), "Setting min bin to %i", range_in_bins);
                set_parameter(rclcpp::Parameter("min_bin", min_bin));
            }
            else {
                min_bin = temp_min_bin;
            }
            update_radar_navigation_config = true;
        }

        if (get_parameter("power_threshold").as_double() != power_threshold)
        {
            double temp_power_threshold = get_parameter("power_threshold").as_double();
            if (temp_power_threshold < 0 || temp_power_threshold > std::numeric_limits<uint8_t>::max() * 10) {
                RCLCPP_INFO(Node::get_logger(), "Power threshold of %f is invalid, must be between 0 and %i", temp_power_threshold, std::numeric_limits<uint8_t>::max() * 10);
                RCLCPP_INFO(Node::get_logger(), "Setting power threshold to %i", std::numeric_limits<uint8_t>::max() * 10 / 2);
                set_parameter(rclcpp::Parameter("power_threshold", std::numeric_limits<uint8_t>::max() * 10 / 2));
            }
            else {
                power_threshold = temp_power_threshold;
            }
            update_radar_navigation_config = true;
        }
        if (get_parameter("max_peaks_per_azimuth").as_int() != max_peaks_per_azimuth)
        {
            int temp_max_peaks_per_azimuth = get_parameter("max_peaks_per_azimuth").as_int();
            if (temp_max_peaks_per_azimuth < 0 || temp_max_peaks_per_azimuth > 5) {
                RCLCPP_INFO(Node::get_logger(), "Max peaks per azimuth of %i is invalid, must be between 0 and %i", temp_max_peaks_per_azimuth, 5);
                RCLCPP_INFO(Node::get_logger(), "Setting max peaks per azimuth to %i", 5);
                set_parameter(rclcpp::Parameter("max_peaks_per_azimuth", 5));
            }
            else {
                max_peaks_per_azimuth = temp_max_peaks_per_azimuth;
            }
            update_radar_navigation_config = true;
        }

        configuration_data_publisher->publish(config_message);
        rotation_count = 0;

        if (update_radar_navigation_config) {
            Navigation_mode_point_cloud_publisher::update_navigation_config();
            radar_client->request_navigation_configuration();
        }
    }

    if (!rotated_once) {
        return;
    }
}

void Navigation_mode_point_cloud_publisher::configuration_data_handler(const Navtech::Configuration_data::Pointer& data, const Navtech::Configuration_data::ProtobufPointer& protobuf_data) {
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

    RCLCPP_INFO(Node::get_logger(), "Starting navigation mode point cloud publisher");
    RCLCPP_INFO(Node::get_logger(), "Start azimuth: %i", start_azimuth);
    RCLCPP_INFO(Node::get_logger(), "End azimuth: %i", end_azimuth);
    RCLCPP_INFO(Node::get_logger(), "Start bin: %i", start_bin);
    RCLCPP_INFO(Node::get_logger(), "End bin: %i", end_bin);
    RCLCPP_INFO(Node::get_logger(), "Azimuth offset: %i", azimuth_offset);
    RCLCPP_INFO(Node::get_logger(), "Processing locally: %s", process_locally ? "true" : "false");

    if (process_locally) {

        peak_finder->configure(data,
            protobuf_data,
            power_threshold,
            bins_to_operate_on,
            min_bin,
            Navtech::BufferModes::off,
            10,
            max_peaks_per_azimuth);

        radar_client->start_non_contour_fft_data();
    }
    else {
        Navigation_mode_point_cloud_publisher::update_navigation_config();
        radar_client->start_navigation_data();
    }
}