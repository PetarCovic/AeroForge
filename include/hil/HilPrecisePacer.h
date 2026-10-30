#pragma once

#include <chrono>
#include <stop_token>
#include <system_error>
#include <thread>

#ifdef _WIN32

#ifndef NOMINMAX
#define NOMINMAX
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <Windows.h>

#endif

namespace aeroforge::hil
{
    class HilPrecisePacer final
    {
    public:
        using Clock = std::chrono::steady_clock;

        HilPrecisePacer()
        {
#ifdef _WIN32
#ifndef CREATE_WAITABLE_TIMER_HIGH_RESOLUTION
#define CREATE_WAITABLE_TIMER_HIGH_RESOLUTION 0x00000002
#endif

            constexpr DWORD access =
                TIMER_MODIFY_STATE | SYNCHRONIZE;

            // Prefer a high-resolution Windows timer.
            timer_ = ::CreateWaitableTimerExW(
                nullptr,
                nullptr,
                CREATE_WAITABLE_TIMER_HIGH_RESOLUTION,
                access);

            // Compatibility fallback for older Windows.
            // Ordinary timers may have worse precision.
            if (timer_ == nullptr)
            {
                timer_ = ::CreateWaitableTimerExW(
                    nullptr,
                    nullptr,
                    0,
                    access);
            }

            if (timer_ == nullptr)
            {
                throwWin32("CreateWaitableTimerExW");
            }

            // A separate event interrupts a pending
            // timer wait when stop is requested.
            stopEvent_ = ::CreateEventW(
                nullptr,
                TRUE,
                FALSE,
                nullptr);

            if (stopEvent_ == nullptr)
            {
                const DWORD error = ::GetLastError();

                ::CloseHandle(timer_);
                timer_ = nullptr;

                throw std::system_error(
                    static_cast<int>(error),
                    std::system_category(),
                    "CreateEventW");
            }
#endif
        }

        ~HilPrecisePacer() noexcept
        {
#ifdef _WIN32
            if (timer_ != nullptr)
            {
                ::CancelWaitableTimer(timer_);
                ::CloseHandle(timer_);
            }

            if (stopEvent_ != nullptr)
            {
                ::CloseHandle(stopEvent_);
            }
#endif
        }

        HilPrecisePacer(
            const HilPrecisePacer&) = delete;

        HilPrecisePacer& operator=(
            const HilPrecisePacer&) = delete;

        HilPrecisePacer(
            HilPrecisePacer&&) = delete;

        HilPrecisePacer& operator=(
            HilPrecisePacer&&) = delete;

        // Returns false if the caller requests stop.
        // The deadline is an absolute steady_clock
        // timestamp, preventing accumulated drift.
        bool waitUntil(
            Clock::time_point deadline,
            std::stop_token stop)
        {
            if (stop.stop_requested())
            {
                return false;
            }

#ifdef _WIN32

            // The OS timer wakes slightly ahead of
            // the target, followed by a short spin.
            //
            // At 100 Hz, a 400 us spin window uses
            // at most about 4% of one logical core.
            constexpr auto spinMargin =
                std::chrono::microseconds(400);

            // This callback signals the event even
            // if the worker is blocked in the OS.
            std::stop_callback stopCallback(
                stop,
                [this]() noexcept
                {
                    ::SetEvent(stopEvent_);
                });

            const auto now = Clock::now();

            if (deadline - now > spinMargin)
            {
                const auto durationToWait =
                    deadline -
                    spinMargin -
                    Clock::now();

                if (durationToWait >
                    Clock::duration::zero())
                {
                    // Windows relative timer deadlines
                    // use 100 nanosecond units.
                    using HundredNanoseconds =
                        std::chrono::duration<
                        long long,
                        std::ratio<1, 10000000>>;

                    const auto ticks =
                        std::chrono::duration_cast<
                        HundredNanoseconds>(
                            durationToWait).count();

                    LARGE_INTEGER dueTime{};

                    // Negative values mean relative
                    // time rather than UTC timestamps.
                    dueTime.QuadPart =
                        -((ticks > 0) ? ticks : 1);

                    if (!::SetWaitableTimerEx(
                        timer_,
                        &dueTime,
                        0,
                        nullptr,
                        nullptr,
                        nullptr,
                        0))
                    {
                        throwWin32(
                            "SetWaitableTimerEx");
                    }

                    // Put the cancellation event first
                    // so stop wins if both are signaled.
                    const HANDLE handles[] =
                    {
                        stopEvent_,
                        timer_
                    };

                    const DWORD result =
                        ::WaitForMultipleObjects(
                            2,
                            handles,
                            FALSE,
                            INFINITE);

                    if (result == WAIT_OBJECT_0)
                    {
                        return false;
                    }

                    if (result != WAIT_OBJECT_0 + 1)
                    {
                        throwWin32(
                            "WaitForMultipleObjects");
                    }
                }
            }

            // Finish with a short precision wait.
            // YieldProcessor reduces the cost of
            // a CPU spin on supported architectures.
            while (Clock::now() < deadline)
            {
                if (stop.stop_requested())
                {
                    return false;
                }

                YieldProcessor();
            }

#else

            // Portable fallback. Check cancellation
            // between short sleep intervals.
            while (Clock::now() < deadline)
            {
                if (stop.stop_requested())
                {
                    return false;
                }

                const auto current = Clock::now();

                const auto next =
                    (deadline - current >
                        std::chrono::milliseconds(2))
                    ? current +
                    std::chrono::milliseconds(1)
                    : deadline;

                std::this_thread::sleep_until(next);
            }

#endif

            return !stop.stop_requested();
        }

    private:
#ifdef _WIN32

        static void throwWin32(
            const char* operation)
        {
            throw std::system_error(
                static_cast<int>(::GetLastError()),
                std::system_category(),
                operation);
        }

        HANDLE timer_ = nullptr;
        HANDLE stopEvent_ = nullptr;

#endif
    };
}
