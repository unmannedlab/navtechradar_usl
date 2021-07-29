#include <iomanip>
#include <iostream>

#include "colossus_messages.h"

using namespace std;


namespace Navtech::Colossus_network_protocol {

    Set_navigation_threshold::Set_navigation_threshold(Message::Payload& parent) : Header_only { parent } { }


    std::uint16_t Set_navigation_threshold::threshold() const
    {
        // auto addr = &(self()->threshold_val);

        // std::cout << std::hex << "addr of threshold: " << reinterpret_cast<unsigned long>(addr) << std::endl;

        return ntohs(self()->threshold_val);
    }


    void Set_navigation_threshold::threshold(std::uint16_t val) { self()->threshold_val = htons(val); }


    std::size_t Set_navigation_threshold::header_size() const { return sizeof(std::uint16_t); }


} // namespace Navtech::Colossus_network_protocol