#include <rclcpp/rclcpp.hpp>

class Colossus_subscriber : public ::rclcpp::Node
{
public:
    Colossus_subscriber();

    void configuration_data_callback(const messages::msg::ConfigurationDataMessage::SharedPtr msg) const;
    void fft_data_callback(const messages::msg::FftDataMessage::SharedPtr msg) const;

    rclcpp::Subscription<messages::msg::ConfigurationDataMessage>::SharedPtr configuration_data_subscriber;
    rclcpp::Subscription<messages::msg::FftDataMessage>::SharedPtr fft_data_subscriber;

private:
    constexpr static int radar_configuration_queue_size{ 1 };
    constexpr static int radar_fft_queue_size{ 400 };
};