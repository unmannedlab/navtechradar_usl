// Copyright 2016 Navtech Radar Limited
// This file is part of iasdk which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//

#include <chrono>
#include <iostream>
#include <thread>

#include "timer.h"

namespace Navtech {
    Timer::Timer() : Timer::Timer(1000) { }

    Timer::Timer(uint32_t timeout) : timeout_in_milliseconds { timeout }, is_enabled { false } { }

    void Timer::set_callback(std::function<void()> fn)
    {
        std::lock_guard<std::mutex> lock(callback_mutex);
        callback = std::move(fn);
    }

    uint32_t Timer::timeout() const { return timeout_in_milliseconds; }

    void Timer::timeout(const uint32_t new_timeout) { timeout_in_milliseconds = new_timeout; }

    void Timer::do_work()
    {
        auto now =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
                .count();

        while (
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
                .count() < now + timeout_in_milliseconds) {
            std::this_thread::sleep_for(std::chrono::microseconds(10000));

            if (std::abs(std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now().time_since_epoch())
                             .count() -
                         now) > 15000) {
                now = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now().time_since_epoch())
                          .count();
            }

            if (stop_requested) break;
        }

        std::lock_guard<std::mutex> lock(callback_mutex);
        if (is_enabled && callback != nullptr) callback();
    }

    void Timer::enable(const bool enable)
    {
        if (is_enabled != enable) {
            is_enabled = enable;

            if (is_enabled)
                start();
            else
                stop();
        }
    }

} // namespace Navtech
