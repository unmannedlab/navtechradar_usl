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

#include "tcp_radar_client.h"
#include <Pointer_types.h>

namespace Navtech {
    constexpr uint32_t RANGE_MULTIPLIER       = 1000000;
    constexpr float RANGE_MULTIPLIER_FLOAT    = 1000000.0f;
    constexpr uint32_t NAV_DATA_RECORD_LENGTH = (sizeof(uint32_t) + sizeof(uint16_t));

    class Fft_data {
    public:
        using Pointer = Shared_owner<Fft_data>;
        float angle { 0.0f };
        uint16_t azimuth { 0 };
        uint16_t sweep_counter { 0 };
        uint32_t ntp_seconds { 0 };
        uint32_t ntp_split_seconds { 0 };
        std::vector<uint8_t> data;
    };

    class Navigation_data {
    public:
        using Pointer = Shared_owner<Navigation_data>;
        float angle { 0.0f };
        uint16_t azimuth { 0 };
        uint32_t ntp_seconds { 0 };
        uint32_t ntp_split_seconds { 0 };
        std::vector<std::tuple<float, uint16_t>> peaks;
    };

    class Configuration_data {
    public:
        using Pointer = Shared_owner<Configuration_data>;
        uint16_t azimuth_samples { 0 };
        uint16_t encoder_size { 0 };
        uint16_t bin_size { 0 };
        uint16_t range_in_bins { 0 };
        uint16_t expected_rotation_rate { 0 };
    };

    class Radar_client {
    public:
        explicit Radar_client(const std::string& radarAddress, const uint16_t& port = 6317);
        explicit Radar_client(const Radar_client&) = delete;
        Radar_client& operator=(const Radar_client&) = delete;

        void start();
        void stop();

        void update_contour_map(const std::vector<uint8_t>& contourData);
        void start_fft_data();
        void stop_fft_data();
        void Start_navigation_data();
        void stop_navigation_data();
        void set_navigation_threshold(const uint16_t& threshold);
        void set_navigation_gain_and_offset(const float& gain, const float& offset);
        void set_fft_data_callback(std::function<void(const Fft_data::Pointer&)> fn = nullptr);
        void set_navigation_data_callback(std::function<void(const Navigation_data::Pointer&)> fn = nullptr);
        void set_configuration_data_callback(std::function<void(const Configuration_data::Pointer&)> fn = nullptr);

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
        uint16_t azimuth_samples        = 0;
        uint16_t expected_rotation_rate = 0;

        void handle_data(std::vector<std::uint8_t>&& data);
        void handle_configuration_message(const std::vector<std::uint8_t>& data);
        void handle_fft_data_message(const std::vector<std::uint8_t>& data);
        void handle_health_message(const std::vector<std::uint8_t>& data);
        void handle_navigation_data_message(const std::vector<std::uint8_t>& data);
        void send_simple_network_message(const CNDPNetworkDataMessageType& type);
    };

} // namespace Navtech

#endif // RADAR_CLIENT_H
