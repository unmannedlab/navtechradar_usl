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
#include "net_conversion.h"

using namespace Navtech::Utility;

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
    class Configuration : public Message_base::Header_and_payload<Configuration> {
    public:
        // Accessor/mutator API; or, you could make the attributes public
        // (but be careful of endianness issues!)
        //
        std::uint16_t azimuth_samples() const { return to_uint16_host(azi_samples); }
        void azimuth_samples(std::uint16_t val) { azi_samples = to_uint16_network(val); }

        std::uint16_t bin_size() const { return to_uint16_host(bin_sz); }
        void bin_size(std::uint16_t val) { bin_sz = to_uint16_network(val); }

        std::uint16_t range_in_bins() const { return to_uint16_host(range_bins); }
        void range_in_bins(std::uint16_t val) { range_bins = to_uint16_network(val); }

        std::uint16_t encoder_size() const { return to_uint16_host(encoder_sz); }
        void encoder_size(std::uint16_t val) { encoder_sz = to_uint16_network(val); }

        std::uint16_t rotation_speed() const { return to_uint16_host(rotation_spd); }
        void rotation_speed(std::uint16_t val) { rotation_spd = to_uint16_network(val); }

        std::uint16_t packet_rate() const { return to_uint16_host(pckt_rate); }
        void packet_rate(std::uint16_t val) { pckt_rate = to_uint16_network(val); }

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


    class Fft_data : public Message_base::Header_and_payload<Fft_data> {
    public:
        // Accessor/mutator API; or, you could make the attributes public
        // (but be careful of endianness issues!)
        //
        std::uint16_t fft_data_offset() const { return to_uint16_host(data_offset); }
        void fft_data_offset(std::uint16_t val) { data_offset = to_uint16_network(val); }

        std::uint16_t sweep_counter() const { return to_uint16_host(sweep); }
        void sweep_counter(std::uint16_t val) { sweep = to_uint16_network(val); }

        std::uint16_t azimuth() const { return to_uint16_host(azi); }
        void azimuth(std::uint16_t val) { azi = to_uint16_network(val); }

        std::uint32_t ntp_seconds() const { return seconds; }
        void ntp_seconds(std::uint32_t val) { seconds = to_uint32_network(val); }

        std::uint32_t ntp_split_seconds() const { return split_seconds; }
        void ntp_split_seconds(std::uint32_t val) { split_seconds = to_uint32_network(val); }

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


    class Navigation_data : public Message_base::Header_and_payload<Navigation_data> {
    public:
        // Accessor/mutator API; or, you could make the attributes public
        // (but be careful of endianness issues!)
        //
        std::uint16_t azimuth() const { return to_uint16_host(net_azimuth); }
        void azimuth(std::uint16_t val) { net_azimuth = to_uint16_network(val); }

        std::uint32_t ntp_seconds() const { return to_uint32_host(seconds); }
        void ntp_seconds(std::uint32_t val) { seconds = to_uint32_network(val); }

        std::uint32_t ntp_split_seconds() const { return to_uint32_host(split_seconds); }
        void ntp_split_seconds(std::uint32_t val) { split_seconds = to_uint32_network(val); }

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


    class Health : public Message_base::Payload_only<Health> {
    public:
    };


    class Navigation_config : public Message_base::Header_only<Navigation_config> {
    public:
        std::size_t size() const
        {
            return (sizeof(operating_bins) + sizeof(min_bin) + sizeof(threshold) + sizeof(max_peaks));
        }

        void bins_to_operate_on(std::uint16_t bins) { operating_bins = to_uint16_network(bins); }
        std::uint16_t bins_to_operate_on() const    { return to_uint16_host(operating_bins); }

        void min_bin_to_operate_on(std::uint16_t min) { min_bin = to_uint16_network(min); }
        std::uint16_t min_bin_to_operate_on() const   { return to_uint16_host(min_bin); }

        void navigation_threshold(float level) { threshold = to_uint32_network(level * 10.0F); }
        float navigation_threshold() const     { return (from_uint32_network(threshold) / 10.0F); }

        void max_peaks_per_azimuth(std::uint32_t peaks) { max_peaks = to_uint32_network(peaks); }
        std::uint32_t max_peaks_per_azimuth() const     { return to_uint32_host(max_peaks); }

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
