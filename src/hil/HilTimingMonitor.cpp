
#include <hil/HilTimingMonitor.h>

#include <algorithm>
#include <stdexcept>

namespace aeroforge::hil
{
    HilTimingMonitor::HilTimingMonitor(
        Duration stepDeadline)
        : stepDeadline_(stepDeadline)
    {
        if (stepDeadline_ <= Duration::zero())
        {
            throw std::invalid_argument(
                "HIL step deadline must be positive");
        }
    }

    bool HilTimingMonitor::recordDuration(
        HilDurationStatistics& statistics,
        Duration elapsed)
    {
        if (elapsed < Duration::zero())
        {
            return false;
        }

        const double elapsedMs =
            std::chrono::duration<double, std::milli>(
                elapsed).count();

        ++statistics.count;

        // Online running average; does not store samples.
        statistics.averageMs +=
            (elapsedMs - statistics.averageMs) /
            static_cast<double>(statistics.count);

        statistics.maximumMs =
            std::max(statistics.maximumMs, elapsedMs);

        return true;
    }

    bool HilTimingMonitor::recordSimulationStep(
        Duration elapsed)
    {
        if (!recordDuration(
            statistics_.simulationSteps, elapsed))
        {
            return false;
        }

        if (elapsed > stepDeadline_)
        {
            ++statistics_.stepDeadlineMisses;
        }

        return true;
    }

    bool HilTimingMonitor::recordCommunication(
        Duration elapsed)
    {
        return recordDuration(
            statistics_.communication, elapsed);
    }

    void HilTimingMonitor::recordSensorSent(
        std::uint32_t sequence,
        TimePoint sentAt)
    {
        // Update an existing outstanding record if the same
        // sequence appears again. This should be uncommon
        // during normal operation, but keeps storage bounded.
        for (auto& record : sensorHistory_)
        {
            if (record.valid &&
                record.sequence == sequence)
            {
                record.sentAt = sentAt;
                return;
            }
        }

        // Fixed-size ring buffer: the oldest slot is replaced
        // when the history capacity is exceeded.
        sensorHistory_[nextSensorSlot_] = {
            true,
            sequence,
            sentAt
        };

        nextSensorSlot_ =
            (nextSensorSlot_ + 1) % SENSOR_HISTORY_CAPACITY;
    }

    bool HilTimingMonitor::recordCommandAccepted(
        std::uint32_t referencedSensorSequence,
        TimePoint receivedAt)
    {
        for (auto& record : sensorHistory_)
        {
            if (!record.valid ||
                record.sequence != referencedSensorSequence)
            {
                continue;
            }

            // Reject inconsistent backwards time without
            // consuming the corresponding sensor record.
            if (receivedAt < record.sentAt)
            {
                ++statistics_.unmatchedCommandResponses;
                return false;
            }

            const Duration latency =
                receivedAt - record.sentAt;

            if (!recordDuration(
                statistics_.commandLatency, latency))
            {
                return false;
            }

            record.valid = false;
            return true;
        }

        ++statistics_.unmatchedCommandResponses;
        return false;
    }

    const HilTimingStatistics&
        HilTimingMonitor::statistics() const
    {
        return statistics_;
    }

    HilTimingMonitor::Duration
        HilTimingMonitor::stepDeadline() const
    {
        return stepDeadline_;
    }

    void HilTimingMonitor::reset()
    {
        for (auto& record : sensorHistory_)
        {
            record = {};
        }

        nextSensorSlot_ = 0;
        statistics_ = {};
    }
}
