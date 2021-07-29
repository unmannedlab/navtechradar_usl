// Copyright 2016 Navtech Radar Limited
// This file is part of iasdk which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//

#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>

#ifdef _WIN32
#include <WinSock2.h>
#else
#include <netinet/in.h>
#endif

#include "../common.h"
#include "colossus_messages.h"
#include "colossus_network_message.h"
#include "radar_client.h"

namespace Navtech {
    Radar_client::Radar_client(const std::string& radarAddress, const std::uint16_t& port) :
        radar_client { radarAddress, port }, running { false }, send_radar_data { false }
    { }

    void Radar_client::set_fft_data_callback(std::function<void(const Fft_data::Pointer&)> fn)
    {
        std::lock_guard<std::mutex> lock(_callbackMutex);
        fft_data_callback = std::move(fn);
    }

    void Radar_client::set_navigation_data_callback(std::function<void(const Navigation_data::Pointer&)> fn)
    {
        std::lock_guard<std::mutex> lock(_callbackMutex);
        navigation_data_callback = std::move(fn);
    }

    void Radar_client::set_configuration_data_callback(
        std::function<void(const Configuration_data::Pointer&, const Configuration_data::ProtobufPointer&)> fn)
    {
        std::lock_guard<std::mutex> lock(_callbackMutex);
        configuration_data_callback = std::move(fn);
    }

    void Radar_client::start()
    {
        if (running) return;
        Helpers::Log("Radar_client - Starting");

        radar_client.set_receive_data_callback(std::bind(&Radar_client::handle_data, this, std::placeholders::_1));
        radar_client.start();
        running = true;

        Helpers::Log("Radar_client - Started");
    }

    void Radar_client::stop()
    {
        if (!running) return;

        running = false;
        radar_client.stop();
        Helpers::Log("Radar_client - Stopped");
    }

    void Radar_client::start_fft_data()
    {
        Helpers::Log("Radar_client - Start FFT Data");
        send_simple_network_message(Colossus_network_protocol::Message::Type::start_fft_data);
        send_radar_data = true;
    }

    void Radar_client::stop_fft_data()
    {
        Helpers::Log("Radar_client - Stop FFT Data");
        send_simple_network_message(Colossus_network_protocol::Message::Type::stop_fft_data);
        send_radar_data = false;
    }

    void Radar_client::Start_navigation_data()
    {
        Helpers::Log("Radar_client - Start Navigation Data");
        send_simple_network_message(Colossus_network_protocol::Message::Type::start_nav_data);
    }

    void Radar_client::stop_navigation_data()
    {
        Helpers::Log("Radar_client - Stop Navigation Data");
        send_simple_network_message(Colossus_network_protocol::Message::Type::stop_nav_data);
    }

    void Radar_client::set_navigation_threshold(const std::uint16_t& threshold)
    {
        if (radar_client.get_connection_state() != Connection_state::Connected) return;

        std::vector<std::uint8_t> buffer(sizeof(threshold));
        auto network_threshold = ntohs(threshold);
        std::memcpy(buffer.data(), &network_threshold, sizeof(network_threshold));

        Colossus_network_protocol::Message msg {};
        msg.type(Colossus_network_protocol::Message::Type::set_nav_threshold);
        msg.payload().replace(buffer);
        auto d = msg.relinquish();
        radar_client.send(msg.relinquish());
    }

    void Radar_client::set_navigation_gain_and_offset(const float& gain, const float& offset)
    {
        if (radar_client.get_connection_state() != Connection_state::Connected) return;

        std::vector<std::uint8_t> buffer(sizeof(std::uint32_t) * 2);
        uint32_t net_gain   = htonl(static_cast<std::uint32_t>(gain * RANGE_MULTIPLIER));
        uint32_t net_offset = htonl(static_cast<std::uint32_t>(offset * RANGE_MULTIPLIER));
        std::memcpy(buffer.data(), &net_gain, sizeof(net_gain));
        std::memcpy(&buffer[sizeof(net_gain)], &net_offset, sizeof(net_offset));

        Colossus_network_protocol::Message msg {};
        msg.type(Colossus_network_protocol::Message::Type::set_nav_range_offset_and_gain);
        msg.payload().replace(buffer);
        auto d = msg.relinquish();
        radar_client.send(msg.relinquish());
    }

    void Radar_client::send_simple_network_message(const Colossus_network_protocol::Message::Type& type)
    {
        if (radar_client.get_connection_state() != Connection_state::Connected) return;

        Colossus_network_protocol::Message msg {};
        msg.type(type);
        auto d = msg.relinquish();
        radar_client.send(d);
    }

    void Radar_client::update_contour_map(const std::vector<std::uint8_t>& contour_data)
    {
        if (radar_client.get_connection_state() != Connection_state::Connected) return;

        auto contour = std::vector<std::uint8_t>();
        if (contour_data.size() == 720) {
            contour.resize(contour_data.size());
            auto resolution = bin_size / 10000.0;
            for (std::size_t i = 0; i < contour_data.size(); i += 2) {
                auto result    = (((contour_data[i]) << 8) | ((contour_data[i + 1])));
                result         = (std::uint16_t)std::ceil(result / resolution);
                contour[i]     = (result & 0xff00) >> 8;
                contour[i + 1] = result & 0x00ff;
            }
        }

        Colossus_network_protocol::Message msg {};
        msg.type(Colossus_network_protocol::Message::Type::contour_update);
        msg.payload().replace(contour);
        auto d = msg.relinquish();
        radar_client.send(msg.relinquish());
    }

    void Radar_client::handle_data(std::vector<std::uint8_t>&& data)
    {
        Colossus_network_protocol::Message msg { std::move(data) };

        switch (msg.type()) {
            case Colossus_network_protocol::Message::Type::configuration:
                handle_configuration_message(msg.relinquish());
                break;
            case Colossus_network_protocol::Message::Type::fft_data:
                handle_fft_data_message(msg.relinquish());
                break;
            case Colossus_network_protocol::Message::Type::navigation_data:
                handle_navigation_data_message(msg.relinquish());
                break;
            case Colossus_network_protocol::Message::Type::navigation_alarm_data:
                break;
            case Colossus_network_protocol::Message::Type::health:
                handle_health_message(msg.relinquish());
                break;
            default:
                Helpers::Log("Radar_client - Unhandled Message [" + std::to_string(static_cast<uint32_t>(msg.type())) +
                             "]");
                break;
        }
    }

    void Radar_client::handle_configuration_message(const std::vector<std::uint8_t>& data)
    {
        Helpers::Log("Radar_client - Handle Configuration Message");

        _callbackMutex.lock();
        auto configuration_fn = configuration_data_callback;
        _callbackMutex.unlock();
        if (configuration_fn == nullptr) return;

        Colossus_network_protocol::Message msg { data };
        auto config                 = msg.payload().as<Colossus_network_protocol::Configuration>();
        auto protobuf_configuration = allocate_shared<Colossus::Protobuf::ConfigurationData>();
        protobuf_configuration->ParseFromString(config.to_string());

        azimuth_samples        = config.azimuth_samples();
        bin_size               = config.bin_size();
        range_in_bins          = config.range_in_bins();
        encoder_size           = config.encoder_size();
        expected_rotation_rate = config.rotation_speed() / 1000;

        if (send_radar_data) send_simple_network_message(Colossus_network_protocol::Message::Type::start_fft_data);

        auto configuration_data                    = allocate_shared<Configuration_data>();
        configuration_data->azimuth_samples        = azimuth_samples;
        configuration_data->bin_size               = bin_size;
        configuration_data->range_in_bins          = range_in_bins;
        configuration_data->encoder_size           = encoder_size;
        configuration_data->expected_rotation_rate = expected_rotation_rate;

        configuration_fn(configuration_data, protobuf_configuration);
    }

    void Radar_client::handle_health_message(const std::vector<std::uint8_t>& health_message) { }

    void Radar_client::handle_fft_data_message(const std::vector<std::uint8_t>& data)
    {
        _callbackMutex.lock();
        auto fft_data_fn = fft_data_callback;
        _callbackMutex.unlock();
        if (fft_data_fn == nullptr) return;

        Colossus_network_protocol::Message msg { data };
        auto fft_data = msg.payload().as<Colossus_network_protocol::Fft_data>();

        auto fftData               = allocate_shared<Fft_data>();
        fftData->azimuth           = fft_data.azimuth();
        fftData->angle             = (fft_data.azimuth() * 360.0f) / (float)encoder_size;
        fftData->sweep_counter     = fft_data.sweep_counter();
        fftData->ntp_seconds       = fft_data.ntp_seconds();
        fftData->ntp_split_seconds = fft_data.ntp_split_seconds();
        fftData->data              = fft_data.fft_data();

        fft_data_fn(fftData);
    }

    void Radar_client::handle_navigation_data_message(const std::vector<std::uint8_t>& data)
    {
        _callbackMutex.lock();
        auto navigation_data_fn = navigation_data_callback;
        _callbackMutex.unlock();
        if (navigation_data_fn == nullptr) return;

        Colossus_network_protocol::Message msg { data };
        auto nav_data = msg.payload().as<Colossus_network_protocol::Navigation_data>();
        auto targets  = nav_data.nav_data();

        auto navigation_data               = allocate_shared<Navigation_data>();
        navigation_data->azimuth           = nav_data.azimuth();
        navigation_data->ntp_seconds       = nav_data.ntp_seconds();
        navigation_data->ntp_split_seconds = nav_data.ntp_split_seconds();
        navigation_data->angle             = (nav_data.azimuth() * 360.0f) / (float)encoder_size;

        auto peaks_count = targets.size() / NAV_DATA_RECORD_LENGTH;
        for (auto i = 0; i < (10 + (peaks_count * NAV_DATA_RECORD_LENGTH)); i += NAV_DATA_RECORD_LENGTH) {
            uint32_t peak_resolve = 0;
            std::memcpy(&peak_resolve, &targets[i], sizeof(peak_resolve));
            uint16_t power = 0;
            std::memcpy(&power, &targets[i + sizeof(peak_resolve)], sizeof(power));
            navigation_data->peaks.push_back(
                std::make_tuple<float, uint16_t>(htonl(peak_resolve) / RANGE_MULTIPLIER_FLOAT, htons(power)));
        }

        navigation_data_fn(navigation_data);
    }

} // namespace Navtech
