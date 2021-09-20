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
#include "colossus_network_message.h"
#include "tcp_radar_client.h"
#include <configurationdata.pb.h>
#include <health.pb.h>

namespace Navtech {
    constexpr std::uint32_t range_multiplier       = 1000000;
    constexpr float range_multiplier_float         = 1000000.0f;
    constexpr std::uint32_t nav_data_record_length = (sizeof(std::uint32_t) + sizeof(std::uint16_t));

    struct Fft_data
    {
        using Pointer = Shared_owner<Fft_data>;
        double angle { 0.0 };
        std::uint16_t azimuth { 0 };
        std::uint16_t sweep_counter { 0 };
        std::uint32_t ntp_seconds { 0 };
        std::uint32_t ntp_split_seconds { 0 };
        std::vector<std::uint8_t> data;
    };

    struct Navigation_data
    {
        using Pointer = Shared_owner<Navigation_data>;
        double angle { 0.0 };
        std::uint16_t azimuth { 0 };
        std::uint32_t ntp_seconds { 0 };
        std::uint32_t ntp_split_seconds { 0 };
        std::vector<std::tuple<float, std::uint16_t>> peaks;
    };

    struct Configuration_data
    {
        using Pointer         = Shared_owner<Configuration_data>;
        using ProtobufPointer = Shared_owner<Colossus::Protobuf::ConfigurationData>;
        std::uint16_t azimuth_samples { 0 };
        std::uint16_t encoder_size { 0 };
        double bin_size { 0 };
        std::uint16_t range_in_bins { 0 };
        std::uint16_t expected_rotation_rate { 0 };
        float range_gain { 0.0f };
        float range_offset { 0.0f };
    };

    class Radar_client {
    public:
        explicit Radar_client(const std::string& radarAddress, const std::uint16_t& port = 6317);
        Radar_client(const Radar_client&) = delete;
        Radar_client(Radar_client&&)      = delete;
        Radar_client& operator=(const Radar_client&) = delete;
        Radar_client& operator=(Radar_client&&) = delete;

        void start();
        void stop();

        void update_contour_map(const std::vector<std::uint8_t>& contourData);
        void start_fft_data();
        void stop_fft_data();
        void start_health_data();
        void stop_health_data();
        void Start_navigation_data();
        void stop_navigation_data();
        void set_navigation_threshold(std::uint16_t threshold);
        void set_navigation_gain_and_offset(float gain, float offset);
        void set_fft_data_callback(std::function<void(const Fft_data::Pointer&)> fn = nullptr);
        void set_raw_fft_data_callback(std::function<void(const std::vector<uint8_t>&)> fn = nullptr);
        void set_navigation_data_callback(std::function<void(const Navigation_data::Pointer&)> fn = nullptr);
        void set_configuration_data_callback(
            std::function<void(const Configuration_data::Pointer&, const Configuration_data::ProtobufPointer&)> fn =
                nullptr);
        void set_raw_configuration_data_callback(std::function<void(const std::vector<uint8_t>&)> fn = nullptr);
        void set_health_data_callback(
            std::function<void(const Shared_owner<Colossus::Protobuf::Health>&)> fn = nullptr);

    private:
        Tcp_radar_client radar_client;
        std::atomic_bool running;
        std::atomic_bool send_radar_data;
        std::mutex callback_mutex;
        std::function<void(const std::vector<uint8_t>&)> raw_fft_data_callback        = nullptr;
        std::function<void(const Fft_data::Pointer&)> fft_data_callback               = nullptr;
        std::function<void(const Navigation_data::Pointer&)> navigation_data_callback = nullptr;
        std::function<void(const Configuration_data::Pointer&, const Configuration_data::ProtobufPointer&)>
            configuration_data_callback                                                           = nullptr;
        std::function<void(const std::vector<uint8_t>&)> raw_configuration_data_callback          = nullptr;
        std::function<void(const Shared_owner<Colossus::Protobuf::Health>&)> health_data_callback = nullptr;

        std::uint16_t encoder_size = 0;
        double bin_size            = 0;

        void handle_data(std::vector<std::uint8_t>&& data);
        void handle_configuration_message(Colossus_network_protocol::Message& msg);
        void handle_fft_data_message(Colossus_network_protocol::Message& msg);
        void handle_health_message(Colossus_network_protocol::Message& data);
        void handle_navigation_data_message(Colossus_network_protocol::Message& data);
        void send_simple_network_message(const Colossus_network_protocol::Message::Type& type);
    };

} // namespace Navtech

#endif // RADAR_CLIENT_H
