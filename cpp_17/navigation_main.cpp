// Copyright 2016 Navtech Radar Limited
// This file is part of iasdk which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//

#include <functional>
#include <thread>

#ifdef _WIN32
#include <WinSock2.h>
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Rpcrt4.lib")
#pragma comment(lib, "Iphlpapi.lib")
#endif

#include "common.h"
#include "navigation/peak_finder.h"
#include "network/radar_client.h"
#include "utility/Pointer_types.h"

using namespace Navtech;

Radar_client radar_client { "192.168.0.1" };
Peak_finder peak_finder {};
bool running { true };

#ifdef __linux__
#include "utility/signal_handler.h"
Signal_handler _signalHandler;
void HandleSigInt(std::int32_t signal, std::int32_t info)
#elif _WIN32
void HandleSigInt(int signal)
#endif
{
    switch (signal) {
        case 49:
        case SIGINT:
        case SIGTERM:
            running = false;
            break;
        default:
            break;
    }
}


void fft_data_handler(const Fft_data::Pointer& data)
{
    peak_finder.fft_data_handler(data);
}


void navigation_data_handler(const Azimuth_target& target_data)
{
    Log("navigation_data_handler - Found Targets in Angle [" + std::to_string(target_data.angle) + "] Target Count [" +
        std::to_string(target_data.targets.size()) + "]");
    for (auto t : target_data.targets) {
        Log("\t Range [" + std::to_string(t.range) + "] Power [" + std::to_string(t.power) + "]");
    }
}


void configuration_data_handler(const Configuration_data::Pointer& configuration,
                                const Configuration_data::ProtobufPointer& protobuf_configuration)
{
    double threshold                    = 80.0;             // Threshold in dB
    std::uint8_t bins_to_operate_on     = 4;                // Radar bins window size to search for peaks in
    std::uint16_t start_bin             = 50;               // Start Bin
    BufferModes buffer_mode             = BufferModes::off; // Buffer mode should only be used with a staring radar
    std::size_t buffer_length           = 10;               // Buffer Length
    std::uint32_t max_peaks_per_azimuth = 10;               // Maximum number of peaks to find in a single azimuth

    peak_finder.configure(configuration,
                          protobuf_configuration,
                          threshold,
                          bins_to_operate_on,
                          start_bin,
                          buffer_mode,
                          buffer_length,
                          max_peaks_per_azimuth);

    radar_client.start_fft_data();
    // radar_client.start_health_data(); //Uncomment to receive health messages
}


void health_data_handler(const Shared_owner<Colossus::Protobuf::Health>& health)
{
    Log("Health Received Module MAC Address: [" + health->macaddress() + "]");
}


int main(int argc, char** argv)
{
#ifdef __linux__
    _signalHandler.RegisterHandler(SIGINT, &HandleSigInt);
    _signalHandler.RegisterHandler(SIGTERM, &HandleSigInt);
#elif _WIN32
    signal(SIGINT, HandleSigInt);
    signal(SIGTERM, HandleSigInt);
#endif

#ifdef _WIN32
    WSADATA wsaData;
    auto err = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (err != 0) {
        Log("WSAStartup failed with error [" + std::to_string(err) + "]");
        return EXIT_FAILURE;
    }
#endif

    Log("Navigation Client Starting");

    peak_finder.set_target_callback(std::bind(&navigation_data_handler, std::placeholders::_1));
    radar_client.set_fft_data_callback(std::bind(&fft_data_handler, std::placeholders::_1));
    radar_client.set_configuration_data_callback(
        std::bind(&configuration_data_handler, std::placeholders::_1, std::placeholders::_2));
    radar_client.set_health_data_callback(std::bind(&health_data_handler, std::placeholders::_1));
    radar_client.start();

    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    Log("Navigation Client Stopping");

    radar_client.stop_fft_data();
    peak_finder.set_target_callback();
    radar_client.set_fft_data_callback();
    radar_client.set_configuration_data_callback();
    radar_client.stop();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

#ifdef __linux__
    _signalHandler.UnRegisterHandler(SIGINT);
    _signalHandler.UnRegisterHandler(SIGTERM);
#endif

    Log("Navigation Client Stopped");
}
