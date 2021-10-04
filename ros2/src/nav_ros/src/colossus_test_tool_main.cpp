#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "../../../cpp_17/network/radar_client.h"
#include "colossus_test_tool.h"

int main(int argc, char* argv[])
{
    auto test_tool = new Colossus_test_tool();

    std::cout << "Starting radar client" << std::endl;
    test_tool->radar_client = Navtech::allocate_owned<Navtech::Radar_client>(Navtech::Utility::IP_address { test_tool->radar_ip }, test_tool->radar_port);
    test_tool->radar_client->set_fft_data_callback(std::bind(&Colossus_test_tool::fft_data_handler, test_tool, std::placeholders::_1));
    test_tool->radar_client->set_configuration_data_callback(std::bind(&Colossus_test_tool::configuration_data_handler, test_tool, std::placeholders::_1));

    test_tool->radar_client->start();

    std::this_thread::sleep_for(std::chrono::seconds(10));

    test_tool->radar_client->stop_fft_data();
    test_tool->radar_client->set_configuration_data_callback();
    test_tool->radar_client->set_fft_data_callback();
    test_tool->radar_client->stop();
    std::cout << "Stopped radar client" << std::endl;
}
