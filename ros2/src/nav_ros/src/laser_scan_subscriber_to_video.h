#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

class Laser_scan_subscriber_to_video : public ::rclcpp::Node {
public:
    Laser_scan_subscriber_to_video();

private:
    constexpr static int radar_configuration_queue_size{ 1 };
    constexpr static int radar_laser_scan_queue_size{ 400 };

    bool config_data_received{ false };
    cv::VideoWriter video_writer{};
    int encoder_size{ 0 };
    int azimuth_samples{ 0 };
    int video_width{ 0 };
    int video_height{ 0 };
    int current_bearing{ 0 };
    cv::Mat radar_image{ cv::Size(400, 400), CV_8UC1, cv::Scalar(0, 0) };
    cv::Mat blank_image{ cv::Size(400, 400), CV_8UC1, cv::Scalar(0, 0) };
    uint8_t* image_ptr = { (uint8_t*)radar_image.data };
    int last_azimuth{ 0 };

    void configuration_data_callback(const interfaces::msg::ConfigurationDataMessage::SharedPtr msg) const;
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const;

    rclcpp::Subscription<interfaces::msg::ConfigurationDataMessage>::SharedPtr configuration_data_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscriber;
};

extern std::shared_ptr<Laser_scan_subscriber_to_video> node;