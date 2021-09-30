#ifndef NET_FLOAT_H
#define NET_FLOAT_H

#include <cstdint>
#include <array>
#include <vector>
#include <cstring>

namespace Navtech::Utility {

    using Byte_array_2 = std::array<std::uint8_t, 2>;
    using Byte_array_4 = std::array<std::uint8_t, 4>;

    std::uint16_t to_uint16_network(std::uint16_t host_value);
    std::uint16_t to_uint16_host(std::uint16_t network_value);

    std::uint32_t to_uint32_host(float host_value);
    std::uint32_t to_uint32_network(float host_value);
    float         from_uint32_host(std::uint32_t host_value);
    float         from_uint32_network(std::uint32_t network_value);
    
    std::uint32_t to_uint32_host(std::uint32_t network_value);
    std::uint32_t to_uint32_network(std::uint32_t host_value);

    Byte_array_4 to_byte_array(std::uint32_t value);
    Byte_array_2 to_byte_array(std::uint16_t value);
    std::uint32_t from_byte_array(const Byte_array_4& value);
    std::uint16_t from_byte_array(const Byte_array_2& value);

    template <typename T>
    std::vector<std::uint8_t> to_vector(const T& value)
    {
        std::vector<std::uint8_t> result(sizeof(T));
        std::memcpy(result.data(), &value, sizeof(T));
        return result;
    }

} // namespace Navtech::Utility

#endif // NET_FLOAT_H