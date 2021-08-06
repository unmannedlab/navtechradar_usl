#include <rclcpp/rclcpp.hpp>
#include <radarclient.h>

class Colossus_publisher : public ::rclcpp::Node {
public:
    Colossus_publisher();

    void fft_data_handler(const Navtech::FFTDataPtr_t& data);

    void configuration_data_handler(const Navtech::ConfigurationDataPtr_t& data);
};