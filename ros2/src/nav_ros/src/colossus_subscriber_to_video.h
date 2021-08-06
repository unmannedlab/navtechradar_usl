#include "rclcpp/rclcpp.hpp"

using namespace rclcpp;

class Colossus_subscriber_to_video : public ::rclcpp::Node {
public:
    Colossus_subscriber_to_video();

    void configuration_data_callback(const interfaces::msg::ConfigurationDataMessage::SharedPtr msg) const;

    void fft_data_callback(const interfaces::msg::FftDataMessage::SharedPtr msg) const;

    rclcpp::Subscription<interfaces::msg::ConfigurationDataMessage>::SharedPtr configuration_data_subscriber;
    rclcpp::Subscription<interfaces::msg::FftDataMessage>::SharedPtr fft_data_subscriber;
};