#ifndef CP_MESSAGES_H
#define CP_MESSAGES_H

// DEBUG ONLY
//
#include <iomanip>
#include <iostream>

#ifdef __linux
#include <arpa/inet.h>
#else
#include "winsock.h"
#endif
#include <cstdint>

#include "colossus_message_base.h"

namespace {

    // Conversion helpers
    //
    union float_uint32_map
    {
        float f;
        std::uint32_t i;
    };

    inline std::uint32_t to_uint32_host(float in)
    {
        float_uint32_map value {};
        value.f = in;
        return value.i;
    }

    inline std::uint32_t to_uint32_network(float in)
    {
        float_uint32_map value {};
        value.f = in;
        return htonl(value.i);
    }

    inline float from_uint32_host(std::uint32_t in)
    {
        float_uint32_map value {};
        value.i = in;
        return value.f;
    }

    inline float from_uint32_network(std::uint32_t in)
    {
        float_uint32_map value {};
        value.i = ntohl(in);
        return value.f;
    }
} // namespace

namespace Navtech::Network::Colossus_protocol {

// DO NOT REMOVE - 
// This ensures correct alignment for all 
// Colossus messages
//
#pragma pack(1)

    // ---------------------------------------------------------------------------------------------
    // Actual message types must inherit from one of the Message_base templates
    // passing their own type as the template parameter.  This is an application
    // of the Curiously Recurring Template Pattern (CRTP)
    //
    class Configuration : public Message_base::Protocol_buffer<Configuration> {
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

        float range_gain() const { return from_uint32_network(gain); }
        void range_gain(float val) { gain = to_uint32_network(val); }

        float range_offset() const { return from_uint32_network(offset); }
        void range_offset(float val) { offset = to_uint32_network(val); }

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

        std::uint32_t ntp_seconds() const { return seconds; }
        void ntp_seconds(std::uint32_t val) { seconds = htonl(val); }

        std::uint32_t ntp_split_seconds() const { return split_seconds; }
        void ntp_split_seconds(std::uint32_t val) { split_seconds = htonl(val); }

        std::vector<std::uint8_t> fft_data() const
        {
            return to_vector();
        }

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


    class Health : public Message_base::Protocol_buffer<Health> {
    public:
    };


    class Navigation_config : public Message_base::Header_only<Navigation_config> {
    public:
        std::size_t size() const
        {
            return (sizeof(operating_bins) + sizeof(min_bin) + sizeof(threshold) + sizeof(max_peaks));
        }

        void bins_to_operate_on(std::uint16_t bins) { operating_bins = htons(bins); }
        std::uint16_t bins_to_operate_on() const { return ntohs(operating_bins); }

        void min_bin_to_operate_on(std::uint16_t min) { min_bin = htons(min); }
        std::uint16_t min_bin_to_operate_on() const { return ntohs(min_bin); }

        void navigation_threshold(float level) { threshold = to_uint32_network(level); }
        float navigation_threshold() const { return from_uint32_network(threshold); }

        void max_peaks_per_azimuth(std::uint32_t peaks) { max_peaks = htons(peaks); }
        std::uint32_t max_peaks_per_azimuth() const { return ntohs(max_peaks); }

    private:
        // Attribute overlay
        //
        std::uint16_t operating_bins;
        std::uint16_t min_bin;
        std::uint32_t threshold;
        std::uint32_t max_peaks;
    };


// DO NOT REMOVE - 
// This ensures correct alignment for all 
// Colossus messages
//
#pragma pack()

} // namespace Navtech::Network::Colossus_protocol


#endif // CP_MESSAGES_H
