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
    class Configuration : public Message_base::Protocol_buffer<Configuration> {
    public:
        // Derived message types MUST provide a constructor in this form
        //
        Configuration(Message::Payload& parent) : Protocol_buffer { parent } { }


        // Accessor/mutator API; or, you could make the attributes public
        // (but be careful of endianness issues!)
        //
        std::uint16_t azimuth_samples() const { return ntohs(self()->azi_samples); }
        void azimuth_samples(std::uint16_t val) { self()->azi_samples = htons(val); }

        std::uint16_t bin_size() const { return ntohs(self()->bin_sz); }
        void bin_size(std::uint16_t val) { self()->bin_sz = htons(val); }

        std::uint16_t range_in_bins() const { return ntohs(self()->range_bins); }
        void range_in_bins(std::uint16_t val) { self()->range_bins = htons(val); }

        std::uint16_t encoder_size() const { return ntohs(self()->encoder_sz); }
        void encoder_size(std::uint16_t val) { self()->encoder_sz = htons(val); }

        std::uint16_t rotation_speed() const { return ntohs(self()->rotation_spd); }
        void rotation_speed(std::uint16_t val) { self()->rotation_spd = htons(val); }

        std::uint16_t packet_rate() const { return ntohs(self()->pckt_rate); }
        void packet_rate(std::uint16_t val) { self()->pckt_rate = htons(val); }

        float range_gain() const { return self()->gain; }
        void range_gain(float val) { self()->gain = val; }

        float range_offset() const { return self()->offset; }
        void range_offset(float val) { self()->offset = val; }


        // If your message has a header you MUST provide this function
        //
        std::size_t header_size() const { return (6 * sizeof(std::uint16_t) + 2 * sizeof(float)); }

    private:
// Attribute order MUST match the actual message header, as
// this is a memory overlay.
//
#pragma pack(1)
        std::uint16_t azi_samples;
        std::uint16_t bin_sz;
        std::uint16_t range_bins;
        std::uint16_t encoder_sz;
        std::uint16_t rotation_spd;
        std::uint16_t pckt_rate;
        float gain;
        float offset;
#pragma pack()
    };

    class Fft_data : public Message_base::Header_only<Fft_data> {
    public:
        // Derived message types MUST provide a constructor in this form
        //
        Fft_data(Message::Payload& parent) : Header_only { parent } { }


        // Accessor/mutator API; or, you could make the attributes public
        // (but be careful of endianness issues!)
        //
        std::uint16_t fft_data_offset() const { return ntohs(self()->data_offset); }
        void fft_data_offset(std::uint16_t val) { self()->data_offset = htons(val); }

        std::uint16_t sweep_counter() const { return ntohs(self()->sweep); }
        void sweep_counter(std::uint16_t val) { self()->sweep = htons(val); }

        std::uint16_t azimuth() const { return ntohs(self()->azi); }
        void azimuth(std::uint16_t val) { self()->azi = htons(val); }

        std::uint32_t ntp_seconds() const { return ntohl(self()->seconds); }
        void ntp_seconds(std::uint32_t val) { self()->seconds = htonl(val); }

        std::uint32_t ntp_split_seconds() const { return ntohl(self()->split_seconds); }
        void ntp_split_seconds(std::uint32_t val) { self()->split_seconds = htonl(val); }

        std::vector<uint8_t> fft_data() const
        {
            return std::vector<std::uint8_t>(payload()->begin() + fft_data_offset(), payload()->end());
        }

        // If your message has a header you MUST provide this function
        //
        std::size_t header_size() const { return (3 * sizeof(std::uint16_t) + 2 * sizeof(uint32_t)); }

    private:
// Attribute order MUST match the actual message header, as
// this is a memory overlay.
//
#pragma pack(1)
        uint16_t data_offset;
        uint16_t sweep;
        uint16_t azi;
        uint32_t seconds;
        uint32_t split_seconds;
#pragma pack()
    };

    class Navigation_data : public Message_base::Header_only<Navigation_data> {
    public:
        // Derived message types MUST provide a constructor in this form
        //
        Navigation_data(Message::Payload& parent) : Header_only { parent } { }


        // Accessor/mutator API; or, you could make the attributes public
        // (but be careful of endianness issues!)
        //
        std::uint16_t azimuth() const { return ntohs(self()->net_azimuth); }
        void azimuth(std::uint16_t val) { self()->net_azimuth = htons(val); }

        std::uint32_t ntp_seconds() const { return ntohl(self()->seconds); }
        void ntp_seconds(std::uint32_t val) { self()->seconds = htonl(val); }

        std::uint32_t ntp_split_seconds() const { return ntohl(self()->split_seconds); }
        void ntp_split_seconds(std::uint32_t val) { self()->split_seconds = htonl(val); }

        std::vector<uint8_t> nav_data() const
        {
            return std::vector<std::uint8_t>(payload()->begin(), payload()->end());
        }


        // If your message has a header you MUST provide this function
        //
        std::size_t header_size() const { return (sizeof(std::uint16_t) + 2 * sizeof(uint32_t)); }

    private:
// Attribute order MUST match the actual message header, as
// this is a memory overlay.
//
#pragma pack(1)
        std::uint16_t net_azimuth;
        std::uint32_t seconds;
        std::uint32_t split_seconds;
#pragma pack()
    };

} // namespace Navtech::Colossus_network_protocol


#endif // CP_MESSAGES_H
