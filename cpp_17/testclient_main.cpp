// Copyright 2016 Navtech Radar Limited
// This file is part of iasdk which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//

#include "common.h"
#include "network/radar_client.h"
#include "utility/Pointer_types.h"

#ifdef _WIN32
#include <WinSock2.h>
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Rpcrt4.lib")
#pragma comment(lib, "Iphlpapi.lib")
#endif

using namespace Navtech;

uint16_t last_azimuth        = 0;
uint16_t packet_count        = 0;
uint64_t last_rotation_reset = Helpers::Now();
bool rotated_once            = false;
Owner_of<Radar_client> radar_client;

void fft_data_handler(const Fft_data::Pointer& data)
{
    packet_count++;
    if (data->azimuth < last_azimuth) {
        auto diff = Helpers::Now() - last_rotation_reset;
        if (rotated_once) {
            Helpers::Log("fft_data_handler - Rotating @ [" + std::to_string(1000.0 / diff) + "Hz] Packets [" +
                         std::to_string(packet_count) + "]");
        }
        last_rotation_reset = Helpers::Now();
        packet_count        = 0;
        rotated_once        = true;
    }
    last_azimuth = data->azimuth;
}

void configuration_data_handler(const Configuration_data::Pointer& data)
{
    Helpers::Log("configuration_data_handler - Expected Rotation Rate [" +
                 std::to_string(data->expected_rotation_rate) + "Hz]");
    Helpers::Log("configuration_data_handler - Range In Bins [" + std::to_string(data->range_in_bins) + "]");
    Helpers::Log("configuration_data_handler - Bin Size [" + std::to_string(data->bin_size / 10000.0) + "cm]");
    Helpers::Log("configuration_data_handler - Range In Metres [" +
                 std::to_string((data->bin_size / 10000.0) * data->range_in_bins) + "m]");
    Helpers::Log("configuration_data_handler - Azimuth Samples [" + std::to_string(data->azimuth_samples) + "]");

    packet_count = 0;
    last_azimuth = 0;

    radar_client->start_fft_data();
}

int32_t main(int32_t argc, char** argv)
{
#ifdef _WIN32
    WSADATA wsaData;
    auto err = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (err != 0) {
        Helpers::Log("Tracker - WSAStartup failed with error [" + std::to_string(err) + "]");
        return EXIT_FAILURE;
    }
#endif

    Helpers::Log("Test Client Starting");

    radar_client = allocate_owned<Radar_client>("10.77.2.211");
    radar_client->set_fft_data_callback(std::bind(&fft_data_handler, std::placeholders::_1));
    radar_client->set_configuration_data_callback(std::bind(&configuration_data_handler, std::placeholders::_1));
    radar_client->start();

    std::this_thread::sleep_for(std::chrono::milliseconds(6000));

    Helpers::Log("Test Client Stopping");
    radar_client->stop_navigation_data();
    radar_client->stop_fft_data();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    radar_client->set_configuration_data_callback();
    radar_client->set_fft_data_callback();

    radar_client->stop();

    Helpers::Log("Test Client Stopped");

    return 0;
}
