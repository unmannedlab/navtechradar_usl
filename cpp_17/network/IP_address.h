#ifndef IP_ADDRESS_H
#define IP_ADDRESS_H

#include <cstdint>
#include <string>

namespace Navtech::Utility {

    class IP_address {
    public:
        IP_address() = default;
        IP_address(std::string ip_addr_str);

        IP_address& operator=(std::uint32_t ip_addr);
        IP_address& operator=(const std::string& ip_addr_str);

        operator std::uint32_t() const;
        std::string to_string() const;

    private:
        void parse_string(const std::string& ip_addr_str);

        union Address
        {
            std::uint32_t word;
            std::uint8_t byte[4];
        };

        Address address { 0x7F000001 };
    };

} // namespace Navtech::Utility

#endif // IP_ADDRESS_H
