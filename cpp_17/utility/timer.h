// Copyright 2016 Navtech Radar Limited
// This file is part of iasdk which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//

#ifndef TIMER_H
#define TIMER_H

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>

#include "threaded_class.h"

namespace Navtech {
    class Timer : public Threaded_class {
    public:
        std::function<void()> callback = nullptr;
        std::mutex callback_mutex;
        std::chrono::milliseconds timeout_in_milliseconds;
        std::atomic_bool is_enabled;

    protected:
        void do_work();

    public:
        explicit Timer();
        explicit Timer(std::chrono::milliseconds timeout);

        Timer(const Timer&) = delete;
        Timer& operator=(const Timer&) = delete;

        void set_callback(std::function<void()> callback = nullptr);
        void enable(const bool enable);
        std::chrono::milliseconds timeout() const;
        void timeout(const std::chrono::milliseconds& new_timeout);
    };
} // namespace Navtech

#endif // TIMER_H
