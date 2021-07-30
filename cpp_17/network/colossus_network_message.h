#ifndef CP_NETWORK_MESSAGE_H
#define CP_NETWORK_MESSAGE_H

#include <cstdint>
#include <vector>


#include "../utility/Pointer_types.h"
#include "IP_address.h"

namespace Navtech::Colossus_network_protocol {

    constexpr std::array<uint8_t, 16> valid_signature {
        0x00, 0x01, 0x03, 0x03, 0x07, 0x07, 0x0F, 0x0F, 0x1F, 0x1F, 0x3F, 0x3F, 0x7F, 0x7F, 0xFE, 0xFE,
    };

    constexpr std::size_t signature_sz { valid_signature.size() };
    constexpr std::uint8_t version { 1 };

    // --------------------------------------------------------------------------------------------------
    // The Message class provides an interface for storing and accessing
    // CP Network messages.
    //
    class Message {
    public:
        enum class Type : std::uint8_t {
            invalid                        = 0,
            keep_alive                     = 1,
            configuration                  = 10,
            configuration_request          = 20,
            start_fft_data                 = 21,
            stop_fft_data                  = 22,
            starthealthmsgs                = 23,
            stop_health_msgs               = 24,
            recalibrate_rf_health          = 25,
            start_tracks                   = 26,
            stop_tracks                    = 27,
            transmit_on                    = 28,
            transmit_off                   = 29,
            fft_data                       = 30,
            high_precision_fft_data        = 31,
            health                         = 40,
            contour_update                 = 50,
            system_restart                 = 76,
            logging_levels                 = 90,
            logging_levels_request         = 100,
            set_auto_tune                  = 110,
            start_nav_data                 = 120,
            stop_nav_data                  = 121,
            set_nav_threshold              = 122,
            navigation_data                = 123,
            set_nav_range_offset_and_gain  = 124,
            calibrate_accelerometer        = 125,
            start_accelerometer            = 126,
            stop_accelerometer             = 127,
            accelerometer_data             = 128,
            start_non_contour_fft_data     = 140,
            set_nav_buffer_mode_and_length = 141,
            set_nav_bin_operation          = 142,
            navigation_alarm_data          = 143,
            set_nav_area_rules             = 144,
            nav_radar_reset                = 145,
            nav_radar_halt                 = 146
        };

        using ID             = std::uint32_t;
        using Iterator       = std::uint8_t*;
        using Const_iterator = const std::uint8_t*;

        // Payload provides an abstract interface into the payload
        // part of the message.  A Payload object is just an overlay
        // into the message's data; it contains no data itself.
        //
        class Payload {
        public:
            Payload(Message& parent);

            void append(const std::vector<std::uint8_t>& data);
            void append(std::vector<std::uint8_t>&& data);
            void append(Const_iterator start, std::size_t n);
            void append(const std::string& str);
            void append(std::string&& str);

            Payload& operator+=(const std::vector<std::uint8_t>& data);
            Payload& operator+=(std::vector<std::uint8_t>&& data);
            Payload& operator+=(const std::string& str);
            Payload& operator+=(std::string&& str);

            void replace(const std::vector<std::uint8_t>& data);
            void replace(std::vector<std::uint8_t>&& data);
            void replace(Const_iterator start, std::size_t n);
            void replace(const std::string& str);
            void replace(std::string&& str);

            Payload& operator=(const std::vector<std::uint8_t>& data);
            Payload& operator=(std::vector<std::uint8_t>&& data);
            Payload& operator=(const std::string& str);
            Payload& operator=(std::string&& str);

            std::vector<std::uint8_t> relinquish();

            std::uint32_t size() const;

            Iterator begin();
            Const_iterator begin() const;
            Iterator end();
            Const_iterator end() const;

            template<typename Accessor_Ty>
            Accessor_Ty as()
            {
                return Accessor_Ty { *this };
            }

        protected:
            void size(std::uint32_t sz);

        private:
            Association_to<Message> msg;
        };

        friend class Payload;


        // Empty, but initialized, messages; ready for adding payload, etc.
        //
        Message();
        Message(const std::string& ip_addr, ID id);

        // Initialize message with header information and payload
        //
        Message(const std::string& ip_addr, ID id, Type t, const std::vector<std::uint8_t>& payload_vector);
        Message(const std::string& ip_addr, ID id, Type t, std::vector<std::uint8_t>&& payload_vector);
        Message(const std::string& ip_addr, ID id, Type t, const std::string& payload_string);
        Message(const std::string& ip_addr, ID id, Type t, std::string&& payload_string);
        Message(const std::string& ip_addr, ID id, Type t, Const_iterator payload_start, std::size_t payload_sz);

        // Constructors to initialise from an (incoming) buffer of data; complete with
        // header information.
        //
        Message(const std::vector<std::uint8_t>& message_vector);
        Message(std::vector<std::uint8_t>&& message_vector);
        Message(Const_iterator message_start, std::size_t message_sz);
        Message(const std::string& ip_addr, ID id, const std::vector<std::uint8_t>& message_vector);
        Message(const std::string& ip_addr, ID id, std::vector<std::uint8_t>&& message_vector);
        Message(const std::string& ip_addr, ID id, Const_iterator message_start, std::size_t message_sz);

        ID id() const;
        void id(ID new_id);

        Type type() const;
        void type(Type t);

        const Utility::IP_address& ip_address() const;
        void ip_address(const std::string& ip_addr_str);

        // Network message interface - header + payload information
        //
        bool is_valid() const;
        std::size_t size() const;
        static constexpr std::size_t header_size() { return sizeof(Header); }

        void replace(const std::vector<std::uint8_t>& src);
        void replace(std::vector<std::uint8_t>&& src);
        void replace(Const_iterator src_start, std::size_t src_sz);
        std::vector<std::uint8_t> relinquish();

        Iterator begin();
        Const_iterator begin() const;
        Iterator end();
        Const_iterator end() const;

        Payload payload() { return Payload { *this }; }
        const Payload payload() const { return Payload { *(const_cast<Message*>(this)) }; }

    protected:
        void initialize();
        bool is_signature_valid() const;
        bool is_version_valid() const;
        void display();

        Iterator signature_begin();
        Const_iterator signature_begin() const;
        Iterator signature_end();
        Const_iterator signature_end() const;

    private:
// Header is for overlay only - header information is stored in
// data vector, contigous with the payload.
//
#pragma pack(1)
        struct Header
        {
            std::uint8_t signature[signature_sz];
            std::uint8_t version;
            Type id;
            std::uint32_t payload_size;

            static Header* overlay_onto(std::uint8_t* from) { return reinterpret_cast<Header*>(from); }
            static const Header* overlay_onto(const std::uint8_t* from) { return reinterpret_cast<const Header*>(from); }
        };
#pragma pack()

        static_assert(sizeof(Message::Header) == 22);

        Utility::IP_address address {};
        ID identity {};
        std::vector<std::uint8_t> data {};
    };

} // namespace Navtech::Colossus_network_protocol

#endif // CP_NETWORK_MESSAGE_H
