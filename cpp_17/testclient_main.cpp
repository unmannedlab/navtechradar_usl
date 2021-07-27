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

uint16_t _lastAzimuth       = 0;
uint16_t _packetCount       = 0;
uint64_t _lastRotationReset = Helpers::Now();
Owner_of<Radar_client> _radarClient;

void FFTDataHandler(const Fft_data::Pointer& data)
{
    _packetCount++;
    if (data->Azimuth < _lastAzimuth) {
        auto diff = Helpers::Now() - _lastRotationReset;
        Helpers::Log("FFTDataHandler - Rotating @ [" + std::to_string(1000.0 / diff) + "Hz] Packets [" +
                     std::to_string(_packetCount) + "]");
        _lastRotationReset = Helpers::Now();
        _packetCount       = 0;
    }
    // std::cout << data->Azimuth << "\n";
    _lastAzimuth = data->Azimuth;
}

void ConfigurationDataHandler(const Configuration_data::Pointer& data)
{
    Helpers::Log("ConfigurationDataHandler - Expected Rotation Rate [" + std::to_string(data->Expected_rotation_rate) +
                 "Hz]");
    Helpers::Log("ConfigurationDataHandler - Range In Bins [" + std::to_string(data->Range_in_bins) + "]");
    Helpers::Log("ConfigurationDataHandler - Bin Size [" + std::to_string(data->Bin_size / 10000.0) + "cm]");
    Helpers::Log("ConfigurationDataHandler - Range In Metres [" +
                 std::to_string((data->Bin_size / 10000.0) * data->Range_in_bins) + "m]");
    Helpers::Log("ConfigurationDataHandler - Azimuth Samples [" + std::to_string(data->Azimuth_samples) + "]");

    _packetCount = 0;
    _lastAzimuth = 0;

    _radarClient->Start_fft_data();
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

    _radarClient = make_owned<Radar_client>("10.77.2.211");
    _radarClient->Set_fft_data_callback(std::bind(&FFTDataHandler, std::placeholders::_1));
    _radarClient->Set_configuration_data_callback(std::bind(&ConfigurationDataHandler, std::placeholders::_1));
    _radarClient->Start();

    std::this_thread::sleep_for(std::chrono::milliseconds(6000));

    Helpers::Log("Test Client Stopping");
    _radarClient->Stop_navigation_data();
    _radarClient->Stop_fft_data();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    _radarClient->Set_configuration_data_callback();
    _radarClient->Set_fft_data_callback();

    _radarClient->Stop();

    Helpers::Log("Test Client Stopped");

    return 0;
}
