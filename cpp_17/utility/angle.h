#ifndef ANGLE_H
#define ANGLE_H

#include <iostream>
#include <cmath>

namespace Navtech::Utility {

    // Angle represents a degree measurement between
    // 0.0 and 360.0, measured clockwise
    // Angles greater than 360.0 will be wrapped
    // Angles less than zero are taken to be anti-clockwise
    // and converted to their clockwise equivalent.
    //
    class Angle {
    public:
        constexpr Angle() = default;
        constexpr Angle(float init) : value { normalise(init) }
        {
        }

        constexpr float to_float() const
        {
            return value;
        }

        constexpr Angle operator+(const Angle& rhs) const
        {
            return Angle { this->value + rhs.value };
        }

        constexpr Angle operator-(const Angle& rhs) const
        {
            return Angle { this->value - rhs.value };
        }

        Angle& operator+=(const Angle& rhs)
        {
            value = normalise(this->value + rhs.value);
            return *this;
        }

        Angle& operator-=(const Angle& rhs)
        {
            value = normalise(this->value - rhs.value);
            return *this;
        }

        friend std::ostream& operator<<(std::ostream& os, const Angle& a)
        {
            os << a.to_float();
            return os;
        }

    private:
        float value { };

        constexpr float normalise(float value) const
        {
            float result { };
            if (value >= 360.0)  value = (value - 360.0);
            if (value <= -360.0) value = (value + 360.0);
            if (value < 0.0)    value = (360.0 + value);

            return (result + value);
        }
    };

} // namespace Navtech::Utility

constexpr Navtech::Utility::Angle operator""_deg(long double value)
{
    return Navtech::Utility::Angle { static_cast<float>(value) };
}

constexpr Navtech::Utility::Angle operator""_deg(unsigned long long int value)
{
    return Navtech::Utility::Angle { static_cast<float>(value) };
}

#endif // ANGLE_H