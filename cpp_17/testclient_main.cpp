// Copyright 2016 Navtech Radar Limited
// This file is part of iasdk which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//

#include <chrono>
#include <cstdint>

#include "common.h"
#include "radar_client.h"
#include "pointer_types.h"
#include "sector_blanking.h"

#ifdef _WIN32
#include <WinSock2.h>
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Rpcrt4.lib")
#pragma comment(lib, "Iphlpapi.lib")
#endif

using namespace Navtech;
using namespace Navtech::Utility;

std::uint64_t Now()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
        .count();
}

std::uint16_t last_azimuth        = 0;
std::uint16_t packet_count        = 0;
std::uint64_t last_rotation_reset = Now();
bool rotated_once                 = false;
Owner_of<Radar_client> radar_client;

void fft_data_handler(const Fft_data::Pointer& data)
{
    packet_count++;
    if (data->azimuth < last_azimuth) {
        auto diff         = Now() - last_rotation_reset;
        auto packate_rate = packet_count / (diff / 1000.0);
        if (rotated_once) {
            Log("fft_data_handler - Rotating @ [" + std::to_string(1000.0 / diff) + "Hz] Packets [" +
                std::to_string(packet_count) + "] [" + std::to_string(packate_rate) + "]");
        }
        last_rotation_reset = Now();
        packet_count        = 0;
        rotated_once        = true;
    }
    last_azimuth = data->azimuth;
}


void configuration_data_handler(const Configuration_data::Pointer& data,
                                const Configuration_data::ProtobufPointer& protobuf_configuration)
{
    Log("configuration_data_handler - Expected Rotation Rate [" + std::to_string(data->expected_rotation_rate) +
        " Hz]");
    Log("configuration_data_handler - Range In Bins [" + std::to_string(data->range_in_bins) + "]");
    Log("configuration_data_handler - Range In Metres [" +
        std::to_string((data->bin_size / 10000.0) * data->range_in_bins) + " m]");
    Log("configuration_data_handler - Azimuth Samples [" + std::to_string(data->azimuth_samples) + "]");
    Log("configuration_data_handler - Bin Size [" + std::to_string(data->bin_size / 10000.0) + " m]");
    Log("configuration_data_handler - Range Resolution [" +
        std::to_string(protobuf_configuration->rangeresolutionmetres()) + " m]");
    Log("configuration_data_handler - Resolution Hz [" + std::to_string(protobuf_configuration->rangeresolutionhz()) +
        " Hz]");

    packet_count = 0;
    last_azimuth = 0;

    // radar_client->start_fft_data();
    radar_client->request_navigation_configuration();

    // Set-up blanking sectors
    //
    Blanking_sector_list blanking_sectors {
        Sector { 45.0, 60.0 },
        Sector { 65.0, 80.0 },
        Sector { 85.0, 100.0 },
        Sector { 105.0, 120.0 }
    };

    radar_client->set_blanking_sectors(blanking_sectors);
}


void navigation_config_handler(const Navigation_config::Pointer& cfg)
{
    using std::to_string;

    Navigation_config nav_config { *cfg };

    Log("Navigation_config_handler - Bins to operate on [" + to_string(nav_config.bins_to_operate_on) + "]");
    Log("Navigation_config_handler - Minimum bin [" + to_string(nav_config.min_bin) + "]");
    Log("Navigation_config_handler - Navigation threshold [" + to_string(nav_config.navigation_threshold) + "]");
    Log("Navigation_config_handler - Maximum peaks per azimuth [" + to_string(nav_config.max_peaks_per_azimuth) + "]");

    // Now, change the configuration and send back to the
    // radar.
    //
    nav_config.bins_to_operate_on   = 10;
    nav_config.min_bin              = 100;
    nav_config.navigation_threshold = 75.6;

    radar_client->set_navigation_configuration(nav_config);
}


int main(int argc, char** argv)
{
#ifdef _WIN32
    WSADATA wsaData;
    auto err = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (err != 0) {
        Log("Tracker - WSAStartup failed with error [" + std::to_string(err) + "]");
        return EXIT_FAILURE;
    }
#endif

    Log("Test Client Starting");

    radar_client = allocate_owned<Radar_client>("127.0.0.1"_ipv4);

    radar_client->set_fft_data_callback(fft_data_handler);
    radar_client->set_configuration_data_callback(configuration_data_handler);
    radar_client->set_navigation_config_callback(navigation_config_handler);

    radar_client->start();

    std::this_thread::sleep_for(std::chrono::milliseconds(10'000));




    Log("Test Client Stopping");
    radar_client->stop_navigation_data();
    radar_client->stop_fft_data();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    radar_client->set_configuration_data_callback();
    radar_client->set_fft_data_callback();
    radar_client->set_navigation_config_callback();

    radar_client->stop();

    Log("Test Client Stopped");
}
