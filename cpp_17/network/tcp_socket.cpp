// Copyright 2016 Navtech Radar Limited
// This file is part of iasdk which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//

#include <cstring>

#ifdef _WIN32
#include <WinSock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#include "tcp_socket.h"
#include <common.h>

namespace Navtech {
    Tcp_socket::Tcp_socket(const std::string& destination, const uint16_t& port) :
        _sock { -1 }, _destination { destination }
    { }

    Tcp_socket::~Tcp_socket()
    {
        if (is_valid()) {
#ifdef _WIN32
            closesocket(_sock);
#else
            ::close(_sock);
#endif
        }
    }

    const bool Tcp_socket::is_valid() const { return _sock != -1; }

    bool Tcp_socket::Create(uint32_t receiveTimeout)
    {
        _sock = socket(AF_INET, SOCK_STREAM, 0);

        if (!is_valid()) {
            Helpers::Log("Failed to create socket");
            return false;
        }

        auto on = 1;
        if (setsockopt(_sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&on, sizeof(on)) == -1) {
            Helpers::Log("Failed to set SO_REUSEADDR socket option on socket [" + std::to_string(_sock) + "]");
            return false;
        }

        linger lingerVal;
        lingerVal.l_onoff  = 0;
        lingerVal.l_linger = 2;
        if (setsockopt(_sock, SOL_SOCKET, SO_LINGER, (const char*)&lingerVal, sizeof(lingerVal)) == -1) {
            Helpers::Log("Failed to set SO_LINGER socket option on socket [" + std::to_string(_sock) + "]");
            return false;
        }

        if (receiveTimeout > 0) {
            struct timeval tv;
            tv.tv_sec  = receiveTimeout;
            tv.tv_usec = 0; // Not initialising this can cause strange errors
            setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(struct timeval));
        }

        Helpers::Log("Created socket [" + std::to_string(_sock) + "]");
        return true;
    }

    void Tcp_socket::Set_send_timeout(uint32_t sendTimeout)
    {
        if (sendTimeout > 0) {
            struct timeval tv;
            tv.tv_sec  = sendTimeout;
            tv.tv_usec = 0; // Not initialising this can cause strange errors
            setsockopt(_sock, SOL_SOCKET, SO_SNDTIMEO, (char*)&tv, sizeof(struct timeval));
        }
    }

    bool Tcp_socket::Connect()
    {
        if (!is_valid()) return false;

        _addr.sin_family = AF_INET;
        _addr.sin_port   = htons(_port);
        auto status      = inet_pton(AF_INET, _destination.c_str(), &_addr.sin_addr);
        status           = ::connect(_sock, (sockaddr*)&_addr, sizeof(_addr));

        if (status == 0) { return true; }
        else {
            Helpers::Log("Failed to connect socket [" + std::to_string(_sock) + "]");
            return false;
        }
    }

    uint32_t Tcp_socket::Send(const std::vector<uint8_t>& data)
    {
#ifdef _WIN32
        auto status = ::send(_sock, (char*)&data[0], data.size(), 0);
#else
        auto status = ::send(_sock, (char*)&data[0], data.size(), MSG_NOSIGNAL);
#endif

        if (status == -1) {
            auto errorNumber = errno;
            Helpers::Log("Send error");
            return errorNumber;
        }
        else {
            return 0;
        }
    }

    uint32_t Tcp_socket::Receive(std::vector<uint8_t>& data, int32_t bytesToRead, bool peek)
    {
        int32_t bytesRead = 0;
        int32_t flags     = 0;

        if (peek) { flags |= MSG_PEEK; }

        int32_t status;
        while (bytesRead < bytesToRead) {
            std::vector<uint8_t> buf(bytesToRead, 0);
            status = ::recv(_sock, (char*)buf.data(), bytesToRead - bytesRead, flags);

            if (status <= 0) { return 0; }
            else
                bytesRead += status;

            data.insert(data.end(), buf.begin(), buf.begin() + status);

            if (!is_valid()) return 0;
        }

        if (status == -1) { return 0; }
        else if (status == 0) {
            return 0;
        }
        else {
            return bytesRead;
        }
    }

    bool Tcp_socket::Close(bool shutdown)
    {
        if (!is_valid()) return false;
        if (shutdown) ::shutdown(_sock, 2);

#ifdef _WIN32
        auto close_return = closesocket(_sock);
#else
        auto close_return = ::close(_sock);
#endif

        if (close_return == -1) {
            Helpers::Log("Failed to close socket [" + std::to_string(_sock) + "]");
            return false;
        }

        Helpers::Log("Closed socket [" + std::to_string(_sock) + "]");
        _sock = -1;
        return true;
    }

} // namespace Navtech
