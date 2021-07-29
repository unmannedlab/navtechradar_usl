#ifndef COLOSSUS_MESSAGE_BASE_H
#define COLOSSUS_MESSAGE_BASE_H

#include <cstdint>

// DEBUG ONLY
#include <iomanip>
#include <iostream>

#include "../utility/Pointer_types.h"
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
        class Header_only {
        public:
            Header_only(Message::Payload& parent) : payload_ptr { associate_with(parent) } { }

            // self() returns an overlay to the underlying payload.  It ensures
            // the Derived_Ty is correctly aligned to the raw (payload) data.
            // Derived types MUST use self() to access any of their attributes
            //
            Derived_Ty* self() { return reinterpret_cast<Derived_Ty*>(payload()->begin() - sizeof(Header_only)); }

            const Derived_Ty* self() const
            {
                // auto base = payload()->begin();
                // auto sz   = sizeof(Header_only);

                // std::cout << std::hex;
                // std::cout << "base addr: " << reinterpret_cast<unsigned long>(base) << std::endl;
                // std::cout << "size:      " << sz << std::endl;
                // std::cout << "self:      " << reinterpret_cast<unsigned long>((payload()->begin() -
                // sizeof(Header_only))) << std::endl; std::cout << std::dec << std::endl;

                return reinterpret_cast<const Derived_Ty*>(payload()->begin() - sizeof(Header_only));
            }


            // Derived types MUST override this function to return the size
            // of their header (in bytes)
            //
            std::size_t header_size() const { return 0; }

            Association_to<Message::Payload> payload() { return payload_ptr; }

            Association_to<const Message::Payload> payload() const { return payload_ptr; }

        protected:
            Association_to<Message::Payload> payload_ptr;
        };


        template<typename Derived_Ty>
        class Protocol_buffer : public Header_only<Derived_Ty> {
        public:
            using Header_msg = Header_only<Derived_Ty>;

            Protocol_buffer(Message::Payload& parent) : Header_only<Derived_Ty> { parent } { }

            std::size_t protobuf_size() const
            {
                return (Header_msg::payload()->size() - Header_msg::self()->header_size());
            }

            std::uint8_t* begin() { return (Header_msg::payload()->begin() + Header_msg::self()->header_size()); }

            const std::uint8_t* begin() const
            {
                return (Header_msg::payload()->begin() + Header_msg::self()->header_size());
            }

            std::uint8_t* end() { return Header_msg::payload()->end(); }

            const std::uint8_t* end() const { return Header_msg::payload()->end(); }

            std::string to_string() const { return std::string { begin(), end() }; }
        };

    } // namespace Message_base

} // namespace Navtech::Colossus_network_protocol


#endif // COLOSSUS_MESSAGE_BASE_H