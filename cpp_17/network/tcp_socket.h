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

namespace Navtech {
    class Tcp_socket {
    public:
        explicit Tcp_socket(const std::string& destination, const uint16_t& port = 6317);
        ~Tcp_socket();
        Tcp_socket(const Tcp_socket&) = delete;
        Tcp_socket& operator=(const Tcp_socket&) = delete;

        const bool is_valid() const;

        bool create(uint32_t receive_timeout = 0);
        bool connect();
        bool close(bool shutdown = false);
        uint32_t send(const std::vector<uint8_t>& data);
        uint32_t receive(std::vector<uint8_t>& data, int32_t bytes_to_read, bool peek = false);
        void set_send_timeout(uint32_t send_timeout);

    private:
        std::atomic<int32_t> _sock { -1 };
        std::string _destination { "192.168.0.1" };
        uint16_t _port { 6317 };
        sockaddr_in _addr {};
    };

} // namespace Navtech

#endif // TCP_SOCKET_H
