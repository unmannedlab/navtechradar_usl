// Copyright 2016 Navtech Radar Limited
// This file is part of iasdk which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//

#pragma once

#include <atomic>
#include <memory>
#include <thread>

namespace Navtech {
    class Threaded_class {
    public:
        explicit Threaded_class() = default;
        virtual ~Threaded_class() { }

        Threaded_class(const Threaded_class&) = delete;
        Threaded_class& operator=(const Threaded_class&) = delete;

        virtual void start();
        virtual void stop(const bool finish_work = false);
        virtual void join(void);

    protected:
        virtual void do_work() = 0;
        virtual void pre_stop(const bool finish_work = false);
        virtual void post_stop();

        std::thread thread;
        std::atomic<bool> stop_requested;

    private:
        void thread_method();
    };

} // namespace Navtech
