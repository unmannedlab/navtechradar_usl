#ifndef COLOSSUS_MESSAGE_BASE_H
#define COLOSSUS_MESSAGE_BASE_H

#include <cstddef>
#include <cstdint>

// DEBUG ONLY
#include <iomanip>
#include <iostream>

#include "Pointer_types.h"
#include "colossus_network_message.h"

namespace Navtech::Colossus_network_protocol {

    // -------------------------------------------------------------------------------------------
    // Base types for messages.  This implementation uses the Curiously Recurring Template Pattern
    // (CRTP).  This allows a set of common base types, with core functionality, and yet still
    // retain unique message interfaces, without requiring (dynamic) downcasting from the client
    // code.
    //
    namespace Message_base {

        template<typename Derived_Ty>
        class Protocol_buffer;

#pragma pack(1)
        template<typename Derived_Ty>
        class Header_only {
        public:
            // Derived types MUST override this function to return the size
            // of their header (in bytes)
            //
            std::size_t header_size() const { return actual().size(); }

            // begin() and end() give pointers to the Derived_Ty
            // object.
            //
            const std::uint8_t* begin() const { return (reinterpret_cast<const std::uint8_t*>(this) + sizeof(Header_only)); }

            std::uint8_t* begin() { return (reinterpret_cast<std::uint8_t*>(this) + sizeof(Header_only)); }

            const std::uint8_t* end() const { return begin() + header_size(); }

            std::uint8_t* end() { return begin() + header_size(); }

        protected:
            friend Protocol_buffer<Derived_Ty>;

            Derived_Ty& actual() { return *(reinterpret_cast<Derived_Ty*>(this)); }

            const Derived_Ty& actual() const { return *(reinterpret_cast<const Derived_Ty*>(this)); }

            const std::uint8_t* payload_begin() const { return payload_start; }

            const std::uint8_t* payload_end() const { return payload_finish; }

            std::size_t size() const { return 0; }

            // The pointers are overlaid on the raw message buffer, re-using the
            // header signature memory.
            const std::uint8_t* const payload_start {};
            const std::uint8_t* const payload_finish {};

            static constexpr auto padding_sz = Message::header_size() - 2 * sizeof(std::uint8_t*);
            const std::byte alignment_padding[padding_sz] {};
        };
#pragma pack()

#pragma pack(1)
        template<typename Derived_Ty>
        class Protocol_buffer : public Header_only<Derived_Ty> {
        public:
            std::string to_string() const { return std::string { protobuf_begin(), protobuf_end() }; }

            std::vector<std::uint8_t> to_vector() const { return std::vector<std::uint8_t> { protobuf_begin(), protobuf_end() }; }

            std::size_t protobuf_size() const { return protobuf_end() - protobuf_begin(); }

        protected:
            using Header = Header_only<Derived_Ty>;


            const std::uint8_t* protobuf_begin() const { return Header::end(); }

            const std::uint8_t* protobuf_end() const { return Header::payload_end(); }
        };
#pragma pack()

    } // namespace Message_base

} // namespace Navtech::Colossus_network_protocol


#endif // COLOSSUS_MESSAGE_BASE_H
