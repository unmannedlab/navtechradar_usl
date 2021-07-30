#ifndef SIGNAL_HANDLER_H
#define SIGNAL_HANDLER_H

#include <cstdint>
#include <functional>
#include <map>

#include <signal.h>

namespace Navtech {

    std::function<void(std::int32_t, siginfo_t* info)> myCb;
    void CallCb(std::int32_t value, siginfo_t* info, void* unused) { myCb(value, info); }

    class Signal_handler {
    public:
        explicit Signal_handler()             = default;
        Signal_handler(const Signal_handler&) = delete;
        Signal_handler& operator=(const Signal_handler&) = delete;

        void RegisterHandler(std::int32_t signal, std::function<void(std::int32_t signal, std::int32_t info)> callback = nullptr)
        {
            if (callback == nullptr) return;

            _signalCallbacks[signal] = std::move(callback);

            struct sigaction new_action = { { 0 } };
            myCb                        = std::bind(&Signal_handler::Handler, this, std::placeholders::_1, std::placeholders::_2);
            new_action.sa_sigaction     = CallCb;
            sigemptyset(&new_action.sa_mask);
            new_action.sa_flags = SA_SIGINFO | SA_RESTART;

            struct sigaction old_action = { { 0 } };
            sigaction(signal, nullptr, &old_action);

            if (old_action.sa_handler != SIG_IGN) sigaction(signal, &new_action, nullptr);
        }

        void UnRegisterHandler(std::int32_t signal) { _signalCallbacks.erase(signal); }

    private:
        std::map<std::int32_t, std::function<void(std::int32_t signal, std::int32_t info)>> _signalCallbacks;
        void Handler(std::int32_t signal, siginfo_t* info)
        {
            if (_signalCallbacks.count(signal) != 0) _signalCallbacks[signal](signal, info->si_int);
        }
    };

} // namespace Navtech

#endif // SIGNAL_HANDLER_H
