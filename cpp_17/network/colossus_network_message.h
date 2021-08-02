#ifndef CP_NETWORK_MESSAGE_H
#define CP_NETWORK_MESSAGE_H

#include <cstdint>
#include <vector>


#include "IP_address.h"
#include "Pointer_types.h"

namespace Navtech::Colossus_network_protocol {

    constexpr std::array<std::uint8_t, 16> valid_signature {
        0x00, 0x01, 0x03, 0x03, 0x07, 0x07, 0x0F, 0x0F, 0x1F, 0x1F, 0x3F, 0x3F, 0x7F, 0x7F, 0xFE, 0xFE,
    };

    static constexpr std::size_t signature_sz { valid_signature.size() };
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
        using Buffer         = std::vector<std::uint8_t>;


        // Constructors
        //
        Message();
        Message(const std::string& ip_addr, ID id);

        Message(const Buffer& message_vector);
        Message(Buffer&& message_vector);
        Message(Const_iterator message_start, std::size_t message_sz);

        Message(const std::string& ip_addr, ID id, const Buffer& message);
        Message(const std::string& ip_addr, ID id, Buffer&& message);
        Message(const std::string& ip_addr, ID id, Const_iterator message_start, std::size_t message_sz);

        // Colossus message interface
        //
        ID id() const;
        void id(ID new_id);

        Type type() const;
        void type(Type t);

        const Utility::IP_address& ip_address() const;
        void ip_address(const std::string& ip_addr_str);

        bool is_valid() const;

        // size() = header_size() + payload_size()
        //
        std::size_t size() const;
        std::size_t payload_size() const;
        static constexpr std::size_t header_size() { return sizeof(Header); }

        // Replace or retrieve the entire data contents of the message.
        // These functions will invalidate any views.
        //
        void replace(const Buffer& src);
        void replace(Buffer&& src);
        void replace(Const_iterator src_start, std::size_t src_sz);

        std::vector<std::uint8_t> relinquish();

        // Add a protocol buffer of data to the message
        // These functions will invalidate any views.
        //
        Message& append(const Buffer& protocol_buffer);
        Message& append(Buffer&& protocol_buffer);
        Message& append(const std::string& protocol_buffer);
        Message& append(std::string&& protocol_buffer);

        Message& operator<<(const Buffer& protocol_buffer);
        Message& operator<<(Buffer&& protocol_buffer);
        Message& operator<<(const std::string& protocol_buffer);
        Message& operator<<(std::string&& protocol_buffer);

        // Add a header.  The header is always inserted before any
        // protocol buffer.  In general, prefer to add the header
        // *before* adding the protocol buffer.
        // This function will invalidate any views.
        //
        template<typename Message_Ty>
        Message& append(const Message_Ty& header);

        template<typename Message_Ty>
        Message& operator<<(const Message_Ty& header);

        // Interpret the message contents as the provided message type.
        // Pre-conditions:
        // - Message is valid
        // - Message type is correct (matches return from type())
        //
        template<typename Message_Ty>
        Message_Ty* view_as();

    protected:
        void initialize();

        void set_view_iterators();
        void payload_size(std::uint32_t sz);
        Iterator payload_begin();
        Const_iterator payload_begin() const;
        Iterator payload_end();
        Const_iterator payload_end() const;

        void display();

        bool is_signature_valid() const;
        void add_signature();
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
            struct Iterators
            {
                Iterator begin;
                Iterator end;
            };

            union Signature_memory
            {
                std::uint8_t as_signature[signature_sz];
                Iterators as_iterators;
            };

            Signature_memory signature;
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
        bool has_protobuf {};
        Buffer data {};
    };


    template<typename Message_Ty>
    Message& Message::append(const Message_Ty& header)
    {
        using std::begin;
        using std::copy_backward;
        using std::end;

        data.resize(size() + header.header_size());

        if (has_protobuf) {
            auto start  = payload_begin();
            auto finish = start + payload_size();
            copy_backward(start, finish, end(data));
        }

        std::copy(header.begin(), header.end(), payload_begin());

        payload_size(payload_size() + header.header_size());

        return *this;
    }


    template<typename Message_Ty>
    Message& Message::operator<<(const Message_Ty& header)
    {
        return append(header);
    }


    template<typename Message_Ty>
    Message_Ty* Message::view_as()
    {
        set_view_iterators();
        return reinterpret_cast<Message_Ty*>(data.data());
    }

} // namespace Navtech::Colossus_network_protocol

#endif // CP_NETWORK_MESSAGE_H
