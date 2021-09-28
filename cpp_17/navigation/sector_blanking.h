#ifndef SECTOR_BLANKING_H
#define SECTOR_BLANKING_H

#include <utility>
#include <vector>
#include <initializer_list>
#include <cstdint>
#include <iostream>

#include "angle.h"

namespace Navtech {

    class Sector : private std::pair<Utility::Angle, Utility::Angle> {
    public:
        using Pair = std::pair<Utility::Angle, Utility::Angle>;

        using Pair::pair;

        void start(const Utility::Angle& value)
        {
            Pair::first = value;
        }

        Utility::Angle start() const
        {
            return Pair::first;
        }

        void finish(const Utility::Angle& value)
        {
            Pair::second = value;
        }

        Utility::Angle finish() const
        {
            return Pair::second;
        }

        friend std::ostream& operator<<(std::ostream& os, const Sector& s)
        {
            os << "[" << s.start() << ", " << s.finish() << "]";
            return os;
        }
    };


    class Blanking_sector_list {
    public:
        Blanking_sector_list() = default;
        Blanking_sector_list(std::initializer_list<Sector> init);

        bool add(const Sector& sector);
        bool add(Sector&& sector);

        friend std::ostream& operator<<(std::ostream& os, const Blanking_sector_list& bl);

        std::vector<std::uint8_t> to_vector() const;
        void from_vector(const std::vector<std::uint8_t>& buffer);

    private:
        static constexpr std::size_t max_sectors { 8 };

        std::vector<Sector> sectors { };
    };

} // namespace Navtech

#endif // SECTOR_BLANKING_H