// Copyright 2016 Navtech Radar Limited
// This file is part of iasdk which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//

#include "threaded_class.h"

namespace Navtech {
    void Threaded_class::thread_method()
    {
        while (!stop_requested) {
            do_work();
        }
    }

    void Threaded_class::post_stop() { }

    void Threaded_class::pre_stop(const bool finishWork) { }

    void Threaded_class::start()
    {
        if (thread.joinable()) return;

        stop_requested = false;
        thread         = std::thread(&Threaded_class::thread_method, this);

        std::this_thread::yield();
        post_stop();
    }

    void Threaded_class::stop(const bool finishWork)
    {
        if (!thread.joinable() || stop_requested) return;

        stop_requested = true;
        pre_stop(finishWork);
        join();
    }

    void Threaded_class::join(void)
    {
        if (!thread.joinable()) return;
        thread.join();
    }

} // namespace Navtech
