// Copyright 2016 Navtech Radar Limited
// This file is part of iasdk which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//

#include <cmath>
#include <cstring>

#ifdef _WIN32
#include <WinSock2.h>
#else
#include <netinet/in.h>
#endif

#include "cndp_configuration_message.h"
#include "cndp_fft_data_message.h"
#include "cndp_network_data_message.h"
#include "radar_client.h"
#include <common.h>
#include <configurationdata.pb.h>
#include <utility/Protobuf_helpers.h>

namespace Navtech {
    Radar_client::Radar_client(const std::string& radarAddress, const uint16_t& port) :
        radar_client { radarAddress, port }, running { false }, send_radar_data { false }
    { }

    void Radar_client::Set_fft_data_callback(std::function<void(const Fft_data::Pointer&)> fn)
    {
        std::lock_guard<std::mutex> lock(_callbackMutex);
        fft_data_callback = std::move(fn);
    }

    void Radar_client::Set_navigation_data_callback(std::function<void(const Navigation_data::Pointer&)> fn)
    {
        std::lock_guard<std::mutex> lock(_callbackMutex);
        navigation_data_callback = std::move(fn);
    }

    void Radar_client::Set_configuration_data_callback(std::function<void(const Configuration_data::Pointer&)> fn)
    {
        std::lock_guard<std::mutex> lock(_callbackMutex);
        configuration_data_callback = std::move(fn);
    }

    void Radar_client::Start()
    {
        if (running) return;
        Helpers::Log("Radar_client - Starting");

        radar_client.Set_receive_data_callback(std::bind(&Radar_client::Handle_data, this, std::placeholders::_1));
        radar_client.Start();
        running = true;

        Helpers::Log("Radar_client - Started");
    }

    void Radar_client::Stop()
    {
        if (!running) return;

        running = false;
        radar_client.Stop();
        Helpers::Log("Radar_client - Stopped");
    }

    void Radar_client::Start_fft_data()
    {
        Helpers::Log("Radar_client - Start FFT Data");
        Send_simple_network_message(CNDPNetworkDataMessageType::StartFFTData);
        send_radar_data = true;
    }

    void Radar_client::Stop_fft_data()
    {
        Helpers::Log("Radar_client - Stop FFT Data");
        Send_simple_network_message(CNDPNetworkDataMessageType::StopFFTData);
        send_radar_data = false;
    }

    void Radar_client::Start_navigation_data()
    {
        Helpers::Log("Radar_client - Start Navigation Data");
        Send_simple_network_message(CNDPNetworkDataMessageType::StartNavData);
    }

    void Radar_client::Stop_navigation_data()
    {
        Helpers::Log("Radar_client - Stop Navigation Data");
        Send_simple_network_message(CNDPNetworkDataMessageType::StopNavData);
    }

    void Radar_client::Set_navigation_threshold(const uint16_t& threshold)
    {
        if (radar_client.Get_connection_state() != Connection_state::Connected) return;

        std::vector<uint8_t> buffer(sizeof(threshold));
        auto network_threshold = ntohs(threshold);
        std::memcpy(buffer.data(), &network_threshold, sizeof(network_threshold));

        CNDPNetworkDataHeader header;
        header.Init();
        header.Set_message_id(CNDPNetworkDataMessageType::SetNavThreshold);
        header.Set_payload_length(sizeof(threshold));
        auto message = make_shared_owner<Network_data_message>(header, buffer);

        Helpers::Log("Set_navigation_threshold - Message Length [" + std::to_string(message->Message_data().size()) +
                     "]");
        radar_client.Send(message->Message_data());
    }

    void Radar_client::Set_navigation_gain_and_offset(const float& gain, const float& offset)
    {
        if (radar_client.Get_connection_state() != Connection_state::Connected) return;

        std::vector<uint8_t> buffer(sizeof(uint32_t) * 2);
        uint32_t net_gain   = htonl(static_cast<uint32_t>(gain * RANGEMULTIPLIER));
        uint32_t net_offset = htonl(static_cast<uint32_t>(offset * RANGEMULTIPLIER));
        std::memcpy(buffer.data(), &net_gain, sizeof(net_gain));
        std::memcpy(&buffer[sizeof(net_gain)], &net_offset, sizeof(net_offset));

        CNDPNetworkDataHeader header {};
        header.Init();
        header.Set_message_id(CNDPNetworkDataMessageType::SetNavRangeOffsetAndGain);
        header.Set_payload_length(sizeof(uint32_t) + sizeof(uint32_t));
        auto message = make_shared_owner<Network_data_message>(header, buffer);
        radar_client.Send(message->Message_data());
    }

    void Radar_client::Send_simple_network_message(const CNDPNetworkDataMessageType& type)
    {
        if (radar_client.Get_connection_state() != Connection_state::Connected) return;

        CNDPNetworkDataHeader header {};
        header.Init();
        header.Set_message_id(type);
        auto message = make_shared_owner<Network_data_message>(header);
        radar_client.Send(message->Message_data());
    }

    void Radar_client::Update_contour_map(const std::vector<uint8_t>& contour_data)
    {
        if (radar_client.Get_connection_state() != Connection_state::Connected) return;

        auto contour = std::vector<uint8_t>();
        if (contour_data.size() == 720) {
            contour.resize(contour_data.size());
            auto resolution = bin_size / 10000.0;
            for (std::size_t i = 0; i < contour_data.size(); i += 2) {
                auto result    = (((contour_data[i]) << 8) | ((contour_data[i + 1])));
                result         = (uint16_t)std::ceil(result / resolution);
                contour[i]     = (result & 0xff00) >> 8;
                contour[i + 1] = result & 0x00ff;
            }
        }

        CNDPNetworkDataHeader header {};
        header.Init();
        header.Set_message_id(CNDPNetworkDataMessageType::ContourUpdate);
        header.Set_payload_length(static_cast<uint32_t>(contour.size()));
        auto message = make_shared_owner<Network_data_message>(header, contour);
        radar_client.Send(message->Message_data());
    }

    void Radar_client::Handle_data(const CNDPDataMessagePtr_t& message)
    {
        switch (message->Message_id()) {
            case CNDPNetworkDataMessageType::Configuration:
                Handle_configuration_message(message);
                break;
            case CNDPNetworkDataMessageType::FFTData:
                Handle_fft_data_message(message);
                break;
            case CNDPNetworkDataMessageType::NavigationData:
                Handle_navigation_data_message(message);
                break;
            case CNDPNetworkDataMessageType::NavigationAlarmData:
                break;
            case CNDPNetworkDataMessageType::Health:
                Handle_health_message(message);
                break;
            default:
                Helpers::Log("Radar_client - Unhandled Message [" +
                             std::to_string(static_cast<uint32_t>(message->Message_id())) + "]");
                break;
        }
    }

    void Radar_client::Handle_configuration_message(const CNDPDataMessagePtr_t& configMessage)
    {
        Helpers::Log("Radar_client - Handle Configuration Message");

        _callbackMutex.lock();
        auto configuration_fn = configuration_data_callback;
        _callbackMutex.unlock();
        if (configuration_fn == nullptr) return;

        CNDPConfigurationHeaderStruct configHeader {};
        std::memset(&configHeader, 0, sizeof(configHeader));
        std::memcpy(&configHeader,
                    configMessage->Message_data().data(),
                    configHeader.HeaderLength() + configHeader.header.Header_length());

        azimuth_amples         = ntohs(configHeader.azimuth_samples);
        bin_size               = ntohs(configHeader.bin_size);
        range_in_bins          = ntohs(configHeader.range_in_bins);
        encoder_size           = ntohs(configHeader.encoder_size);
        expected_rotation_rate = ntohs(configHeader.rotation_speed) / 1000;

        if (send_radar_data) Send_simple_network_message(CNDPNetworkDataMessageType::StartFFTData);

        auto configurationData                    = make_shared_owner<Configuration_data>();
        configurationData->Azimuth_samples        = azimuth_amples;
        configurationData->Bin_size               = bin_size;
        configurationData->Range_in_bins          = range_in_bins;
        configurationData->EncoderSize            = encoder_size;
        configurationData->Expected_rotation_rate = expected_rotation_rate;

        configuration_fn(configurationData);
    }

    void Radar_client::Handle_health_message(const CNDPDataMessagePtr_t& health_message) { }

    void Radar_client::Handle_fft_data_message(const CNDPDataMessagePtr_t& fft_data_message)
    {
        _callbackMutex.lock();
        auto fft_data_fn = fft_data_callback;
        _callbackMutex.unlock();
        if (fft_data_fn == nullptr) return;

        CNDPNetworkDataFftDataHeaderStruct fft_data_header {};
        auto message_data = fft_data_message->Message_data();

        std::memcpy(&fft_data_header,
                    message_data.data(),
                    fft_data_header.HeaderLength() + fft_data_header.header.Header_length());

        auto fftData               = make_shared_owner<Fft_data>();
        fftData->Azimuth           = ntohs(fft_data_header.azimuth);
        fftData->Angle             = (fftData->Azimuth * 360.0f) / (float)encoder_size;
        fftData->Sweep_counter     = ntohs(fft_data_header.sweep_counter);
        fftData->Ntp_seconds       = fft_data_header.seconds;
        fftData->Ntp_split_seconds = fft_data_header.split_seconds;
        fftData->Data.resize(fft_data_header.header.Payload_length() - fft_data_header.HeaderLength());
        std::memcpy(fftData->Data.data(),
                    &message_data[fft_data_header.header.Header_length() + fft_data_header.HeaderLength()],
                    fftData->Data.size());

        fft_data_fn(fftData);
    }

    void Radar_client::Handle_navigation_data_message(const CNDPDataMessagePtr_t& navigation_message)
    {
        _callbackMutex.lock();
        auto navigation_data_fn = navigation_data_callback;
        _callbackMutex.unlock();
        if (navigation_data_fn == nullptr) return;

        uint16_t net_azimuth = 0;
        std::memcpy(&net_azimuth, &navigation_message->Payload()[0], sizeof(net_azimuth));
        uint32_t net_seconds = 0;
        std::memcpy(&net_seconds, &navigation_message->Payload()[sizeof(net_azimuth)], sizeof(net_seconds));
        uint32_t net_split_seconds = 0;
        std::memcpy(&net_split_seconds,
                    &navigation_message->Payload()[sizeof(net_azimuth) + sizeof(net_seconds)],
                    sizeof(net_split_seconds));

        auto navigation_data               = make_shared_owner<Navigation_data>();
        navigation_data->Azimuth           = ntohs(net_azimuth);
        navigation_data->Ntp_seconds       = ntohl(net_seconds);
        navigation_data->Ntp_split_seconds = ntohl(net_split_seconds);
        navigation_data->Angle             = (navigation_data->Azimuth * 360.0f) / (float)encoder_size;

        auto offset      = sizeof(net_azimuth) + sizeof(net_seconds) + sizeof(net_split_seconds);
        auto peaks_count = (navigation_message->Payload_size() - offset) / NAVDATARECORDLENGTH;
        for (auto i = offset; i < (10 + (peaks_count * NAVDATARECORDLENGTH)); i += NAVDATARECORDLENGTH) {
            uint32_t peak_resolve = 0;
            std::memcpy(&peak_resolve, &navigation_message->Payload()[i], sizeof(peak_resolve));
            uint16_t power = 0;
            std::memcpy(&power, &navigation_message->Payload()[i + sizeof(peak_resolve)], sizeof(power));
            navigation_data->Peaks.push_back(
                std::make_tuple<float, uint16_t>(htonl(peak_resolve) / RANGEMULTIPLIERFLOAT, htons(power)));
        }

        navigation_data_fn(navigation_data);
    }

} // namespace Navtech
