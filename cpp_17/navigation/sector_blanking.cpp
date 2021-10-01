#include "sector_blanking.h"

#include <iterator>
#include <algorithm>

#include "./utility/net_conversion.h"

namespace Navtech {

    Blanking_sector_list::Blanking_sector_list(std::initializer_list<Sector> init)
    {
        using std::copy_n;
        using std::begin;
        using std::back_inserter;

        copy_n(
            begin(init), 
            (init.size() < max_sectors) ? init.size() : max_sectors, 
            back_inserter(sectors)
        );
    }


    bool Blanking_sector_list::add(const Sector& sector)
    {
        if (sectors.size() >= max_sectors) return false;

        sectors.push_back(sector);
        return false;
    }


    bool Blanking_sector_list::add(Sector&& sector)
    {
        if (sectors.size() >= max_sectors) return false;

        sectors.push_back(std::move(sector));
        return true;
    }


    std::vector<std::uint8_t> Blanking_sector_list::to_vector() const
    {
        using namespace std;
        using namespace Navtech::Utility;


        vector<uint8_t> buffer { };

        // Output number of sectors, followed by a byte-stream sequence of angles,
        // each converted from a float to a network-endian set of bytes
        //
        buffer.push_back(static_cast<uint8_t>(sectors.size()));

        for (const auto& sector : sectors) {
            auto start_as_bytes = to_byte_array(to_uint32_network(sector.start().to_float()));
            copy(begin(start_as_bytes), end(start_as_bytes), back_inserter(buffer));

            auto finish_as_bytes = to_byte_array(to_uint32_network(sector.finish().to_float()));
            copy(begin(finish_as_bytes), end(finish_as_bytes), back_inserter(buffer));
        }

        return buffer;
    }


    void Blanking_sector_list::from_vector(const std::vector<std::uint8_t>& buffer)
    {
        using namespace std;
        using namespace Navtech::Utility;

        // Byte stream should contain
        // - 1 byte defining the number of sectors
        // - n sectors, each sector consisting of:
        //   - A start angle, as 4 bytes in network-endianness, representing a float
        //   - A finish angle, as 4 bytes in network-endianness, representing a float
        //
        auto num_sectors { (buffer.size() - 1) / 8 };
        if ((num_sectors % 4) != 0)   throw std::length_error { "Not enough bytess for valid sector" };
        if (buffer[0] != num_sectors) throw std::length_error { "Number of sectors does not match" };

        auto it = buffer.begin() + 1;
        Byte_array_4 bytes { };

        for (unsigned int i { 0 }; i < num_sectors; ++i) {
            copy_n(it, bytes.size(), begin(bytes));
            Angle start_angle { from_uint32_network(from_byte_array(bytes)) };
            it += sizeof(uint32_t);

            copy_n(it, bytes.size(), begin(bytes));
            Angle finish_angle { from_uint32_network(from_byte_array(bytes)) };
            it += sizeof(uint32_t);

            sectors.emplace_back(start_angle, finish_angle);
        }
    }

    std::ostream& operator<<(std::ostream& os, const Blanking_sector_list& bl)
    {
        for (auto& sector : bl.sectors) {
            os << sector << " ";
        }
        return os;
    }
    
    
} // namespace Navtech
