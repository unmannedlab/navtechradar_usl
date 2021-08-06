
#include <sstream>

#include "IP_address.h"

namespace Navtech::Utility {

    IP_address::IP_address(std::string ip_addr_str) { parse_string(ip_addr_str); }

    IP_address::operator std::uint32_t() const { return address.word; }

    IP_address& IP_address::operator=(std::uint32_t ip_addr)
    {
        address.word = ip_addr;
        return *this;
    }

    IP_address& IP_address::operator=(const std::string& ip_addr_str)
    {
        parse_string(ip_addr_str);
        return *this;
    }

    std::string IP_address::to_string() const
    {
        std::ostringstream os {};

        os << static_cast<int>(address.byte[3]);
        os << ".";
        os << static_cast<int>(address.byte[2]);
        os << ".";
        os << static_cast<int>(address.byte[1]);
        os << ".";
        os << static_cast<int>(address.byte[0]);

        return os.str();
    }

    void IP_address::parse_string(const std::string& ip_addr_str)
    {
        using std::getline;
        using std::istringstream;
        using std::stoi;
        using std::string;

        istringstream stream { ip_addr_str };
        string str {};

        for (int i { 3 }; i >= 0; --i) {
            if (getline(stream, str, '.')) { address.byte[i] = stoi(str); }
        }
    }

} // namespace Navtech::Utility
