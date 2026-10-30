#include <hil/HilSequenceTracker.h>

namespace aeroforge::hil
{
    HilSequenceTracker::HilSequenceTracker(
        std::uint64_t maximumCommandAgeUs)
        : maximumCommandAgeUs_(maximumCommandAgeUs)
    {
    }

    bool HilSequenceTracker::isNewer(
        std::uint32_t incoming,
        std::uint32_t previous)
    {
        const std::uint32_t difference =
            incoming - previous;

        return difference != 0u &&
            difference < 0x80000000u;
    }

    bool HilSequenceTracker::recordSentSensor(
        std::uint32_t sequence,
        std::uint64_t simulationTimeUs)
    {
        if (lastSensorSequence_ &&
            !isNewer(sequence, *lastSensorSequence_))
        {
            return false;
        }

        if (lastSensorTimeUs_ &&
            simulationTimeUs < *lastSensorTimeUs_)
        {
            return false;
        }

        history_[nextRecord_] = {
            sequence,
            simulationTimeUs
        };

        nextRecord_ =
            (nextRecord_ + 1) % HISTORY_CAPACITY;

        if (recordCount_ < HISTORY_CAPACITY)
        {
            ++recordCount_;
        }

        lastSensorSequence_ = sequence;
        lastSensorTimeUs_ = simulationTimeUs;

        return true;
    }

    bool HilSequenceTracker::acceptControl(
        std::uint32_t packetSequence,
        std::uint32_t referencedSensorSequence,
        std::uint64_t currentSimulationTimeUs)
    {
        if (lastControlSequence_ &&
            !isNewer(packetSequence, *lastControlSequence_))
        {
            return false;
        }

        if (lastReferencedSequence_ &&
            !isNewer(
                referencedSensorSequence,
                *lastReferencedSequence_))
        {
            return false;
        }

        std::optional<std::uint64_t> referencedTime;

        for (std::size_t i = 0; i < recordCount_; ++i)
        {
            const std::size_t index =
                (nextRecord_ + HISTORY_CAPACITY - 1 - i)
                % HISTORY_CAPACITY;

            if (history_[index].sequence ==
                referencedSensorSequence)
            {
                referencedTime = history_[index].timeUs;
                break;
            }
        }

        if (!referencedTime)
        {
            return false;
        }

        if (currentSimulationTimeUs < *referencedTime)
        {
            return false;
        }

        const std::uint64_t age =
            currentSimulationTimeUs - *referencedTime;

        if (age > maximumCommandAgeUs_)
        {
            return false;
        }

        lastControlSequence_ = packetSequence;
        lastReferencedSequence_ = referencedSensorSequence;
        lastAcceptedSensorTimeUs_ = *referencedTime;

        return true;
    }

    std::optional<std::uint64_t>
        HilSequenceTracker::lastAcceptedSensorTimeUs() const
    {
        return lastAcceptedSensorTimeUs_;
    }

    void HilSequenceTracker::reset()
    {
        nextRecord_ = 0;
        recordCount_ = 0;

        lastSensorSequence_.reset();
        lastSensorTimeUs_.reset();
        lastControlSequence_.reset();
        lastReferencedSequence_.reset();
        lastAcceptedSensorTimeUs_.reset();
    }
}