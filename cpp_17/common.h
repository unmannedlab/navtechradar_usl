// Copyright 2016 Navtech Radar Limited
// This file is part of iasdk which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to
// https://opensource.org/licenses/MIT for full license details.
//

#ifndef COMMON_H
#define COMMON_H

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace Navtech {

    static void Log(const std::string& message)
    {
        auto now       = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);

        auto nowSeconds      = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
        auto nowMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        auto millisecondsRemaining = nowMilliseconds - (nowSeconds * 1000);

        constexpr auto bufsize { 20 };
        constexpr auto logDateBufSize { 30 };
        char buf[bufsize] {};
        char logDateBuf[logDateBufSize] {};

        std::strftime(buf, bufsize, "%Y%m%d", std::gmtime(&in_time_t));
        std::strftime(logDateBuf, logDateBufSize, "%Y-%m-%dT%H:%M:%S", std::gmtime(&in_time_t));
        std::stringstream ss;
        ss << std::string(logDateBuf) << "." << std::setw(3) << std::setfill('0') << millisecondsRemaining << "Z";
        std::cout << ss.str() << " - " << message << std::endl;
    }

} // namespace Navtech

#endif // COMMON_H
