// Copyright 2016 Navtech Radar Limited
// This file is part of iasdk which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//

#ifndef RADAR_CLIENT_H
#define RADAR_CLIENT_H

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "../utility/Pointer_types.h"
#include "tcp_radar_client.h"

namespace Navtech {
    constexpr uint32_t RANGEMULTIPLIER     = 1000000;
    constexpr float RANGEMULTIPLIERFLOAT   = 1000000.0f;
    constexpr uint32_t NAVDATARECORDLENGTH = (sizeof(uint32_t) + sizeof(uint16_t));

    class Fft_data {
    public:
        using Pointer = Shared_owner<Fft_data>;
        float Angle { 0.0f };
        uint16_t Azimuth { 0 };
        uint16_t Sweep_counter { 0 };
        uint32_t Ntp_seconds { 0 };
        uint32_t Ntp_split_seconds { 0 };
        std::vector<uint8_t> Data;
    };

    class Navigation_data {
    public:
        using Pointer = Shared_owner<Navigation_data>;
        float Angle { 0.0f };
        uint16_t Azimuth { 0 };
        uint32_t Ntp_seconds { 0 };
        uint32_t Ntp_split_seconds { 0 };
        std::vector<std::tuple<float, uint16_t>> Peaks;
    };

    class Configuration_data {
    public:
        using Pointer = Shared_owner<Configuration_data>;
        uint16_t Azimuth_samples { 0 };
        uint16_t EncoderSize { 0 };
        uint16_t Bin_size { 0 };
        uint16_t Range_in_bins { 0 };
        uint16_t Expected_rotation_rate { 0 };
    };

    class Radar_client {
    public:
        explicit Radar_client(const std::string& radarAddress, const uint16_t& port = 6317);
        explicit Radar_client(const Radar_client&) = delete;
        Radar_client& operator=(const Radar_client&) = delete;

        void Start();
        void Stop();

        void Update_contour_map(const std::vector<uint8_t>& contourData);
        void Start_fft_data();
        void Stop_fft_data();
        void Start_navigation_data();
        void Stop_navigation_data();
        void Set_navigation_threshold(const uint16_t& threshold);
        void Set_navigation_gain_and_offset(const float& gain, const float& offset);
        void Set_fft_data_callback(std::function<void(const Fft_data::Pointer&)> fn = nullptr);
        void Set_navigation_data_callback(std::function<void(const Navigation_data::Pointer&)> fn = nullptr);
        void Set_configuration_data_callback(std::function<void(const Configuration_data::Pointer&)> fn = nullptr);

    private:
        Tcp_radar_client radar_client;
        std::atomic_bool running;
        std::atomic_bool send_radar_data;
        std::mutex _callbackMutex;
        std::function<void(const Fft_data::Pointer&)> fft_data_callback                     = nullptr;
        std::function<void(const Navigation_data::Pointer&)> navigation_data_callback       = nullptr;
        std::function<void(const Configuration_data::Pointer&)> configuration_data_callback = nullptr;

        uint16_t encoder_size           = 0;
        uint16_t bin_size               = 0;
        uint16_t range_in_bins          = 0;
        uint16_t azimuth_amples         = 0;
        uint16_t expected_rotation_rate = 0;

        void Handle_data(const CNDPDataMessagePtr_t& message);
        void Handle_configuration_message(const CNDPDataMessagePtr_t& configMessage);
        void Handle_fft_data_message(const CNDPDataMessagePtr_t& fftDataMessage);
        void Handle_health_message(const CNDPDataMessagePtr_t& fftDataMessage);
        void Handle_navigation_data_message(const CNDPDataMessagePtr_t& navigationMessage);
        void Send_simple_network_message(const CNDPNetworkDataMessageType& type);
    };

} // namespace Navtech

#endif // RADAR_CLIENT_H
