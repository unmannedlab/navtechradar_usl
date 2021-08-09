#include <rclcpp/rclcpp.hpp>
#include <radar_client.h>

class Colossus_publisher : public ::rclcpp::Node {
public:
    Colossus_publisher();

    void fft_data_handler(const Navtech::Fft_data::Pointer& data);

    void configuration_data_handler(const Navtech::Configuration_data::Pointer& data);
};