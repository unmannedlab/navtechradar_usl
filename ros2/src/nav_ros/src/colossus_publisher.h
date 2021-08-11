#include <rclcpp/rclcpp.hpp>
#include "radar_client.h"

extern std::shared_ptr<Navtech::Radar_client> radar_client;
extern std::string radar_ip;
extern uint16_t radar_port;

class Colossus_publisher : public ::rclcpp::Node {
public:
    Colossus_publisher();

    void fft_data_handler(const Navtech::Fft_data::Pointer& data);

    void configuration_data_handler(const Navtech::Configuration_data::Pointer& data);
};