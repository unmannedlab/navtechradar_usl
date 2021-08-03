#ifndef CP_MESSAGES_H
#define CP_MESSAGES_H

// DEBUG ONLY
//
#include <iomanip>
#include <iostream>

#include <arpa/inet.h>
#include <cstdint>

#include "colossus_message_base.h"

namespace Navtech::Colossus_network_protocol {

// ---------------------------------------------------------------------------------------------
// Actual message types must inherit from one of the Message_base templates
// passing their own type as the template parameter.  This is an application
// of the Curiously Recurring Template Pattern (CRTP)
//
#pragma pack(1)
    class Configuration : public Message_base::Protocol_buffer<Configuration> {
        union float_uint32_map
        {
            float f;
            std::uint32_t i;
        };

    public:
        // Accessor/mutator API; or, you could make the attributes public
        // (but be careful of endianness issues!)
        //
        std::uint16_t azimuth_samples() const { return ntohs(azi_samples); }
        void azimuth_samples(std::uint16_t val) { azi_samples = htons(val); }

        std::uint16_t bin_size() const { return ntohs(bin_sz); }
        void bin_size(std::uint16_t val) { bin_sz = htons(val); }

        std::uint16_t range_in_bins() const { return ntohs(range_bins); }
        void range_in_bins(std::uint16_t val) { range_bins = htons(val); }

        std::uint16_t encoder_size() const { return ntohs(encoder_sz); }
        void encoder_size(std::uint16_t val) { encoder_sz = htons(val); }

        std::uint16_t rotation_speed() const { return ntohs(rotation_spd); }
        void rotation_speed(std::uint16_t val) { rotation_spd = htons(val); }

        std::uint16_t packet_rate() const { return ntohs(pckt_rate); }
        void packet_rate(std::uint16_t val) { pckt_rate = htons(val); }

        float range_gain() const
        {
            float_uint32_map float_value { 0 };
            float_value.i = ntohl(gain);
            return float_value.f;
        }

        void range_gain(float val)
        {
            float_uint32_map float_value { 0 };
            float_value.f = val;
            gain          = htonl(float_value.i);
        }

        float range_offset() const
        {
            float_uint32_map float_value { 0 };
            float_value.i = ntohl(offset);
            return float_value.f;
        }

        void range_offset(float val)
        {
            float_uint32_map float_value { 0 };
            float_value.f = val;
            offset        = htonl(float_value.i);
        }

        // If your message has a header you MUST provide this function
        //
        std::size_t size() const { return (6 * sizeof(std::uint16_t) + 2 * sizeof(std::uint32_t)); }

    private:
        // Attribute order MUST match the actual message header, as
        // this is a memory overlay.
        //
        std::uint16_t azi_samples;
        std::uint16_t bin_sz;
        std::uint16_t range_bins;
        std::uint16_t encoder_sz;
        std::uint16_t rotation_spd;
        std::uint16_t pckt_rate;
        std::uint32_t gain;
        std::uint32_t offset;
    };
#pragma pack()

#pragma pack(1)
    class Fft_data : public Message_base::Protocol_buffer<Fft_data> {
    public:
        // Accessor/mutator API; or, you could make the attributes public
        // (but be careful of endianness issues!)
        //
        std::uint16_t fft_data_offset() const { return ntohs(data_offset); }
        void fft_data_offset(std::uint16_t val) { data_offset = htons(val); }

        std::uint16_t sweep_counter() const { return ntohs(sweep); }
        void sweep_counter(std::uint16_t val) { sweep = htons(val); }

        std::uint16_t azimuth() const { return ntohs(azi); }
        void azimuth(std::uint16_t val) { azi = htons(val); }

        std::uint32_t ntp_seconds() const { return ntohl(seconds); }
        void ntp_seconds(std::uint32_t val) { seconds = htonl(val); }

        std::uint32_t ntp_split_seconds() const { return ntohl(split_seconds); }
        void ntp_split_seconds(std::uint32_t val) { split_seconds = htonl(val); }

        std::vector<std::uint8_t> fft_data() const { return std::vector<std::uint8_t>(payload_begin() + fft_data_offset(), payload_end()); }

        // If your message has a header you MUST provide this function
        //
        std::size_t size() const { return (3 * sizeof(std::uint16_t) + 2 * sizeof(std::uint32_t)); }

    private:
        // Attribute order MUST match the actual message header, as
        // this is a memory overlay.
        //
        std::uint16_t data_offset;
        std::uint16_t sweep;
        std::uint16_t azi;
        std::uint32_t seconds;
        std::uint32_t split_seconds;
    };
#pragma pack()

#pragma pack(1)
    class Navigation_data : public Message_base::Protocol_buffer<Navigation_data> {
    public:
        // Accessor/mutator API; or, you could make the attributes public
        // (but be careful of endianness issues!)
        //
        std::uint16_t azimuth() const { return ntohs(net_azimuth); }
        void azimuth(std::uint16_t val) { net_azimuth = htons(val); }

        std::uint32_t ntp_seconds() const { return ntohl(seconds); }
        void ntp_seconds(std::uint32_t val) { seconds = htonl(val); }

        std::uint32_t ntp_split_seconds() const { return ntohl(split_seconds); }
        void ntp_split_seconds(std::uint32_t val) { split_seconds = htonl(val); }

        std::vector<std::uint8_t> nav_data() const { return std::vector<std::uint8_t>(payload_begin(), payload_end()); }


        // If your message has a header you MUST provide this function
        //
        std::size_t size() const { return (sizeof(std::uint16_t) + 2 * sizeof(std::uint32_t)); }

    private:
        // Attribute order MUST match the actual message header, as
        // this is a memory overlay.
        //
        std::uint16_t net_azimuth;
        std::uint32_t seconds;
        std::uint32_t split_seconds;
    };
#pragma pack()

} // namespace Navtech::Colossus_network_protocol


#endif // CP_MESSAGES_H
