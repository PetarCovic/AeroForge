
#pragma once

#include <chrono>
#include <optional>

namespace aeroforge::hil
{
    enum class HilWatchdogState
    {
        WAITING,
        HEALTHY,
        TIMED_OUT,
        DISCONNECTED,
        FAULTED
    };

    class HilCommandWatchdog
    {
    public:
        using Clock = std::chrono::steady_clock;
        using TimePoint = Clock::time_point;

        explicit HilCommandWatchdog(
            std::chrono::milliseconds timeout);

        // Notify the watchdog only after a command
        // has passed all protocol and freshness checks.
        bool observeAcceptedCommand(TimePoint now);

        // Explicit communication or controller faults.
        void markDisconnected();
        void markFault();

        // A deliberate new session is required to rearm.
        void reset();

        HilWatchdogState state(TimePoint now) const;

        bool permitsCommand(TimePoint now) const;

        std::chrono::milliseconds timeout() const;

    private:
        std::chrono::milliseconds timeout_;

        std::optional<TimePoint> lastAcceptedCommand_;

        mutable bool timedOut_ = false;
        mutable bool disconnected_ = false;
        mutable bool faulted_ = false;
    };
}
