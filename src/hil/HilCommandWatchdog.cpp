
#include <hil/HilCommandWatchdog.h>

#include <stdexcept>

namespace aeroforge::hil
{
    HilCommandWatchdog::HilCommandWatchdog(
        std::chrono::milliseconds timeout)
        : timeout_(timeout)
    {
        if (timeout_ <= std::chrono::milliseconds::zero())
        {
            throw std::invalid_argument(
                "HIL watchdog timeout must be positive");
        }
    }

    bool HilCommandWatchdog::observeAcceptedCommand(
        TimePoint now)
    {
        // Faulted and disconnected sessions must be
        // explicitly reset before accepting commands.
        if (disconnected_ || faulted_ || timedOut_)
        {
            return false;
        }

        // A late command must not automatically rearm
        // a session whose previous command has expired.
        if (lastAcceptedCommand_)
        {
            if (now < *lastAcceptedCommand_ ||
                now - *lastAcceptedCommand_ > timeout_)
            {
                timedOut_ = true;
                return false;
            }
        }

        lastAcceptedCommand_ = now;

        return true;
    }

    void HilCommandWatchdog::markDisconnected()
    {
        disconnected_ = true;
        lastAcceptedCommand_.reset();
    }

    void HilCommandWatchdog::markFault()
    {
        faulted_ = true;
        lastAcceptedCommand_.reset();
    }

    void HilCommandWatchdog::reset()
    {
        lastAcceptedCommand_.reset();

        timedOut_ = false;
        disconnected_ = false;
        faulted_ = false;
    }

    HilWatchdogState HilCommandWatchdog::state(
        TimePoint now) const
    {
        if (disconnected_)
        {
            disconnected_ = true;
            return HilWatchdogState::DISCONNECTED;
        }

        if (faulted_)
        {
            faulted_ = true;
            return HilWatchdogState::FAULTED;
        }

        if (timedOut_)
        {
            timedOut_ = true;
            return HilWatchdogState::TIMED_OUT;
        }

        if (!lastAcceptedCommand_)
        {
            return HilWatchdogState::WAITING;
        }

        if (now < *lastAcceptedCommand_ ||
            now - *lastAcceptedCommand_ > timeout_)
        {
            timedOut_ = true;
            return HilWatchdogState::TIMED_OUT;
        }

        return HilWatchdogState::HEALTHY;
    }

    bool HilCommandWatchdog::permitsCommand(
        TimePoint now) const
    {
        return state(now) == HilWatchdogState::HEALTHY;
    }

    std::chrono::milliseconds
        HilCommandWatchdog::timeout() const
    {
        return timeout_;
    }
}
