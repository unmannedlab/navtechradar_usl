#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "../../../cpp_17/network/radar_client.h"
#include "colossus_test_tool.h"

using namespace std;
using namespace Navtech;

int main(int argc, char* argv[])
{
    auto test_tool = new Colossus_test_tool();

    cout << "Starting radar client" << endl;
    test_tool->radar_client = allocate_owned<Navtech::Radar_client>(test_tool->radar_ip, test_tool->radar_port);
    test_tool->radar_client->set_fft_data_callback(std::bind(&Colossus_test_tool::fft_data_handler, test_tool, std::placeholders::_1));
    test_tool->radar_client->set_configuration_data_callback(std::bind(&Colossus_test_tool::configuration_data_handler, test_tool, std::placeholders::_1));

    test_tool->radar_client->start();

    this_thread::sleep_for(std::chrono::seconds(10));

    test_tool->radar_client->stop_fft_data();
    test_tool->radar_client->set_configuration_data_callback();
    test_tool->radar_client->set_fft_data_callback();
    test_tool->radar_client->stop();
    cout << "Stopped radar client" << endl;
}
