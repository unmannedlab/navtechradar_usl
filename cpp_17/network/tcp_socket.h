// Copyright 2016 Navtech Radar Limited
// This file is part of iasdk which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//

#ifndef TCP_SOCKET_H
#define TCP_SOCKET_H

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#ifdef _WIN32
#include <WinSock2.h>
#else
#include <netinet/in.h>
#endif

#include "../utility/ip_address.h"

namespace Navtech {
    class Tcp_socket {
    public:
        enum Close_option { do_not_shutdown, shutdown };
        enum Receive_option { consume, peek };

        explicit Tcp_socket(const Utility::IP_address& destination, const std::uint16_t& port = 6317);
        ~Tcp_socket();
        Tcp_socket(const Tcp_socket&) = delete;
        Tcp_socket& operator=(const Tcp_socket&) = delete;

        bool is_valid() const;

        bool create(std::uint32_t receive_timeout = 0);
        bool connect();
        bool close(Close_option opt = do_not_shutdown);
        std::uint32_t send(const std::vector<std::uint8_t>& data);
        std::uint32_t send(std::vector<std::uint8_t>&& data);
        std::uint32_t receive(std::vector<std::uint8_t>& data,
                              std::int32_t bytes_to_read,
                              Receive_option peek = consume);
        void set_send_timeout(std::uint32_t send_timeout);

    private:
        std::atomic<std::int32_t> sock { -1 };
        Utility::IP_address destination { "192.168.0.1" };
        std::uint16_t port { 6317 };
        sockaddr_in addr {};
    };

} // namespace Navtech

#endif // TCP_SOCKET_H
