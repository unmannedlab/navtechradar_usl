#ifndef IP_ADDRESS_H
#define IP_ADDRESS_H

#include <string>
#include <string_view>
#include <cstdint>


namespace Navtech::Utility {

    enum class Endian { host, network };

    class IP_address {
    public:
        IP_address() = default;
        IP_address(std::string_view add_str);
        IP_address(std::uint32_t raw_addr);
        IP_address(std::uint32_t raw_addr, Endian endian);

        IP_address& operator=(std::uint32_t ip_addr);
        IP_address& operator=(std::string_view ip_addr_str);
    
        operator std::uint32_t() const;        // host-endian
        std::string   to_string()  const;
        std::uint32_t to_host_endian() const;
        std::uint32_t to_network_endian() const;
        std::uint32_t value_as(Endian endian) const;

        IP_address  operator~() const;

        IP_address  operator& (const IP_address& rhs) const;
        IP_address& operator&=(const IP_address& rhs);

        IP_address  operator& (std::uint32_t rhs) const;
        IP_address& operator&=(std::uint32_t rhs);
        friend IP_address operator&(std::uint32_t lhs, const IP_address& rhs);

        IP_address  operator| (const IP_address& rhs) const;
        IP_address& operator|=(const IP_address& rhs);

        IP_address  operator| (std::uint32_t rhs) const;
        IP_address& operator|=(std::uint32_t rhs);
        friend IP_address operator|(std::uint32_t lhs, const IP_address& rhs);

        bool operator==(const IP_address& rhs) const;
        bool operator==(std::uint32_t rhs) const;
        bool operator!=(const IP_address& rhs) const;
        bool operator!=(std::uint32_t rhs) const;

    private:
        void parse_string(std::string_view ip_addr_str);
        void validate(std::string_view ip_addr_str);

        union Address {
            std::uint32_t word;
            std::uint8_t  byte[4];
        };

        Address address { };
    };

    // Static object representing localhost (127.0.0.1)
    //
    extern IP_address localhost;


    inline IP_address operator""_ipv4(const char* addr_str, std::size_t sz)
    {
        return IP_address { std::string_view { addr_str, sz } };
    }


    inline IP_address operator""_ipv4(unsigned long long raw_addr)
    {
        return IP_address { static_cast<std::uint32_t>(raw_addr) };
    }

} // namespace Navtech::Utility

#endif // IP_ADDRESS_H