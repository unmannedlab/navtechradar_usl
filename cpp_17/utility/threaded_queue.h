// Copyright 2016 Navtech Radar Limited
// This file is part of iasdk which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//

#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <queue>

#include "threaded_class.h"

constexpr std::uint32_t max_queue_depth = 10000;

namespace Navtech {
    template<class T>
    class Threaded_queue : public Threaded_class {
    public:
        explicit Threaded_queue() = default;


        void enqueue(T&& item, bool notify = true)
        {
            if (stopping || dequeue_callback == nullptr || !thread.joinable() || queue.size() > max_queue_depth) return;
            std::lock_guard lock { queue_mutex };
            queue.push(std::move(item));
            if (notify) condition.notify_all();
        }


        void set_dequeue_callback(std::function<void(T&&)> fn = nullptr)
        {
            std::lock_guard lock { queue_mutex };
            dequeue_callback = std::move(fn);
        }

        void notify() { condition.notify_all(); }

    protected:
        void do_work()
        {
            std::unique_lock lock { queue_mutex };

            if (!queue.empty()) {
                T item = queue.front();
                queue.pop();
                lock.unlock();

                if (dequeue_callback != nullptr) { dequeue_callback(std::move(item)); };
            }
            else {
                lock.unlock();
            }

            lock.lock();
            if (queue.empty() && !stop_requested) { condition.wait(lock); }
            lock.unlock();
        }


        void pre_stop(const bool finish_work)
        {
            stopping = true;

            if (finish_work) {
                while (!dequeue()) { }
            }

            std::queue<T> empty;
            std::swap(queue, empty);
            condition.notify_all();
        }

        void post_stop() { stopping = false; }

    private:
        std::atomic<bool> stopping;
        std::queue<T> queue;
        std::mutex queue_mutex;
        std::condition_variable condition;
        std::function<void(T&&)> dequeue_callback;

        bool dequeue()
        {
            if (!queue.empty()) {
                T item = std::move(queue.front());
                queue.pop();

                if (dequeue_callback != nullptr) { dequeue_callback(std::move(item)); }
            }

            return queue.empty();
        }
    };

} // namespace Navtech
