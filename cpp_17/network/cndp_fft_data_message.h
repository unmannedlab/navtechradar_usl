// Copyright 2016 Navtech Radar Limited
// This file is part of iasdk which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//

#ifndef CNDP_FFT_DATA_MESSAGE_H
#define CNDP_FFT_DATA_MESSAGE_H

#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include "cndp_network_data_message.h"

namespace Navtech {

    constexpr uint8_t TCPNDMFFTDATAOFFSETOFFSET = 0;
    constexpr uint8_t TCPNDMFFTDATAOFFSETLENGTH = sizeof(uint16_t);
    constexpr uint8_t TCPNDMSWEEPCOUNTEROFFSET  = TCPNDMFFTDATAOFFSETOFFSET + TCPNDMFFTDATAOFFSETLENGTH;
    constexpr uint8_t TCPNDMSWEEPCOUNTERLENGTH  = sizeof(uint16_t);
    constexpr uint8_t TCPNDMAZIMUTHOFFSET       = TCPNDMSWEEPCOUNTEROFFSET + TCPNDMSWEEPCOUNTERLENGTH;
    constexpr uint8_t TCPNDMAZIMUTHLENGTH       = sizeof(uint16_t);
    constexpr uint8_t TCPNDMSECONDSOFFSET       = TCPNDMAZIMUTHOFFSET + TCPNDMAZIMUTHLENGTH;
    constexpr uint8_t TCPNDMSECONDSLENGTH       = sizeof(uint32_t);
    constexpr uint8_t TCPNDMSPLITSECONDSOFFSET  = TCPNDMSECONDSOFFSET + TCPNDMSECONDSLENGTH;
    constexpr uint8_t TCPNDMSPLITSECONDSLENGTH  = sizeof(uint32_t);
    constexpr uint8_t TCPNDMFFTDATAHEADERLENGTH = TCPNDMFFTDATAOFFSETLENGTH + TCPNDMSWEEPCOUNTERLENGTH +
                                                  TCPNDMAZIMUTHLENGTH + TCPNDMSECONDSLENGTH + TCPNDMSPLITSECONDSLENGTH;

#pragma pack(push)
#pragma pack(1)
    typedef struct CNDPNetworkDataFftDataHeader
    {
        CNDPNetworkDataHeaderStruct header;
        uint16_t fft_data_offset { 0 };
        uint16_t sweep_counter { 0 };
        uint16_t azimuth { 0 };
        uint32_t seconds { 0 };
        uint32_t split_seconds { 0 };
        void Init()
        {
            header.Init();
            header.message_id = static_cast<uint8_t>(CNDPNetworkDataMessageType::FFTData);
            fft_data_offset   = htons(TCPNDMFFTDATAHEADERLENGTH);
        }
        constexpr const uint32_t HeaderLength() const { return TCPNDMFFTDATAHEADERLENGTH; }
        void set_fft_data_length(uint32_t length) { header.Set_payload_length(TCPNDMFFTDATAHEADERLENGTH + length); }
        void set_sweep_counter(std::uint16_t counter) { sweep_counter = htons(counter); }
        void set_azimuth(std::uint16_t azi) { azimuth = htons(azi); }
        void set_seconds(uint32_t secondsToSet) { seconds = htonl(secondsToSet); }
        void set_split_seconds(uint32_t splitSeconds) { split_seconds = ntohl(splitSeconds); }
        const std::vector<uint8_t> to_data() const
        {
            std::vector<uint8_t> data(header.Payload_length());
            std::memcpy(&data[TCPNDMFFTDATAOFFSETOFFSET], &fft_data_offset, sizeof(fft_data_offset));
            std::memcpy(&data[TCPNDMSWEEPCOUNTEROFFSET], &sweep_counter, sizeof(sweep_counter));
            std::memcpy(&data[TCPNDMAZIMUTHOFFSET], &azimuth, sizeof(azimuth));
            std::memcpy(&data[TCPNDMSECONDSOFFSET], &seconds, sizeof(seconds));
            std::memcpy(&data[TCPNDMSPLITSECONDSOFFSET], &split_seconds, sizeof(split_seconds));
            return data;
        }
    } CNDPNetworkDataFftDataHeaderStruct;
#pragma pack(pop)

} // namespace Navtech

#endif // CNDP_FFT_DATA_MESSAGE_H
