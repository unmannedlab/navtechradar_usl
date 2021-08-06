#include <rclcpp/rclcpp.hpp>
#include <radarclient.h>

using namespace Navtech;
using namespace rclcpp;

class Colossus_publisher : public ::rclcpp::Node {
public:
    Colossus_publisher();

    void fft_data_handler(const FFTDataPtr_t& data);

    void configuration_data_handler(const ConfigurationDataPtr_t& data);
};