// Copyright 2016 Navtech Radar Limited
// This file is part of iasdk which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//

#ifndef CNDP_CONFIGURATION_MESSAGE_H
#define CNDP_CONFIGURATION_MESSAGE_H

#include "cndp_network_data_message.h"

#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

namespace Navtech {

    constexpr uint8_t TCPNDMAZIMUTHSAMPLESOFFSET = 0;
    constexpr uint8_t TCPNDMAZIMUTHSAMPLESLENGTH = sizeof(uint16_t);
    constexpr uint8_t TCPNDMBINSIZEOFFSET        = TCPNDMAZIMUTHSAMPLESOFFSET + TCPNDMAZIMUTHSAMPLESLENGTH;
    constexpr uint8_t TCPNDMBINSIZELENGTH        = sizeof(uint16_t);
    constexpr uint8_t TCPNDMRANGEINBINSOFFSET    = TCPNDMBINSIZEOFFSET + TCPNDMBINSIZELENGTH;
    constexpr uint8_t TCPNDMRANGEINBINSLENGTH    = sizeof(uint16_t);
    constexpr uint8_t TCPNDMENCODERSIZEOFFSET    = TCPNDMRANGEINBINSOFFSET + TCPNDMRANGEINBINSLENGTH;
    constexpr uint8_t TCPNDMENCODERSIZELENGTH    = sizeof(uint16_t);
    constexpr uint8_t TCPNDMROTATIONSPEEDOFFSET  = TCPNDMENCODERSIZEOFFSET + TCPNDMENCODERSIZELENGTH;
    constexpr uint8_t TCPNDMROTATIONSPEEDLENGTH  = sizeof(uint16_t);
    constexpr uint8_t TCPNDMPACKETRATEOFFSET     = TCPNDMROTATIONSPEEDOFFSET + TCPNDMROTATIONSPEEDLENGTH;
    constexpr uint8_t TCPNDMPACKETRATELENGTH     = sizeof(uint16_t);
    constexpr uint8_t TCPNDMRANGEGAINOFFSET      = TCPNDMPACKETRATEOFFSET + TCPNDMPACKETRATELENGTH;
    constexpr uint8_t TCPNDMRANGEGAINLENGTH      = sizeof(uint32_t);
    constexpr uint8_t TCPNDMRANGEOFFSETOFFSET    = TCPNDMRANGEGAINOFFSET + TCPNDMRANGEGAINLENGTH;
    constexpr uint8_t TCPNDMRANGEOFFSETLENGTH    = sizeof(uint32_t);
    constexpr uint8_t TCPNDMCONFIGURATIONHEADERLENGTH =
        TCPNDMAZIMUTHSAMPLESLENGTH + TCPNDMBINSIZELENGTH + TCPNDMRANGEINBINSLENGTH + TCPNDMENCODERSIZELENGTH +
        TCPNDMROTATIONSPEEDLENGTH + TCPNDMPACKETRATELENGTH + TCPNDMRANGEGAINLENGTH + TCPNDMRANGEOFFSETLENGTH;

#pragma pack(push)
#pragma pack(1)
    typedef struct CNDPConfigurationHeader
    {
        CNDPNetworkDataHeaderStruct header;
        uint16_t azimuth_samples;
        uint16_t bin_size;
        uint16_t range_in_bins;
        uint16_t encoder_size;
        uint16_t rotation_speed;
        uint16_t packet_rate;
        uint32_t range_gain;
        uint32_t range_offset;

        void Init()
        {
            header.Init();
            header.message_id = static_cast<uint8_t>(CNDPNetworkDataMessageType::Configuration);
            azimuth_samples   = 0;
            bin_size          = 0;
            range_in_bins     = 0;
            encoder_size      = 0;
            rotation_speed    = 0;
            packet_rate       = 0;
            range_gain        = 1;
            range_offset      = 0;
        }
        constexpr const uint32_t HeaderLength() const { return TCPNDMCONFIGURATIONHEADERLENGTH; }
        void SetProtocolBufferLength(uint32_t length)
        {
            header.Set_payload_length(TCPNDMCONFIGURATIONHEADERLENGTH + length);
        }
        void SetAzimuthSamples(std::uint16_t azimuthSamples) { azimuth_samples = htons(azimuthSamples); }
        void SetBinSize(std::uint16_t binSize) { bin_size = htons(binSize); }
        void SetRangeInBins(std::uint16_t rangeInBins) { range_in_bins = htons(rangeInBins); }
        void SetEncoderSize(std::uint16_t encoderSize) { encoder_size = htons(encoderSize); }
        void SetRotationSpeed(std::uint16_t rotationSpeed) { rotation_speed = htons(rotationSpeed); }
        void SetPacketRate(std::uint16_t packetRate) { packet_rate = htons(packetRate); }
        void SetRangeGain(float rangeGain)
        {
            union v
            {
                float f;
                uint32_t i;
            };
            v testValue;
            testValue.f = rangeGain;
            range_gain  = htonl(testValue.i);
        }
        void SetRangeOffset(float rangeOffset)
        {
            union v
            {
                float f;
                uint32_t i;
            };
            v testValue;
            testValue.f  = rangeOffset;
            range_offset = htonl(testValue.i);
        }
        const std::vector<uint8_t> ToData() const
        {
            std::vector<uint8_t> data(HeaderLength());
            std::memcpy(&data[TCPNDMAZIMUTHSAMPLESOFFSET], &azimuth_samples, sizeof(azimuth_samples));
            std::memcpy(&data[TCPNDMBINSIZEOFFSET], &bin_size, sizeof(bin_size));
            std::memcpy(&data[TCPNDMRANGEINBINSOFFSET], &range_in_bins, sizeof(range_in_bins));
            std::memcpy(&data[TCPNDMENCODERSIZEOFFSET], &encoder_size, sizeof(encoder_size));
            std::memcpy(&data[TCPNDMROTATIONSPEEDOFFSET], &rotation_speed, sizeof(rotation_speed));
            std::memcpy(&data[TCPNDMPACKETRATEOFFSET], &packet_rate, sizeof(packet_rate));
            std::memcpy(&data[TCPNDMRANGEGAINOFFSET], &range_gain, sizeof(range_gain));
            std::memcpy(&data[TCPNDMRANGEOFFSETOFFSET], &range_offset, sizeof(range_offset));
            return data;
        }
    } CNDPConfigurationHeaderStruct;
#pragma pack(pop)

} // namespace Navtech

#endif // CNDP_CONFIGURATION_MESSAGE_H
