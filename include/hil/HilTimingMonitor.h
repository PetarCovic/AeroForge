
#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace aeroforge::hil
{
    struct HilDurationStatistics
    {
        std::uint64_t count = 0;
        double averageMs = 0.0;
        double maximumMs = 0.0;
    };

    struct HilTimingStatistics
    {
        HilDurationStatistics simulationSteps;
        HilDurationStatistics communication;
        HilDurationStatistics commandLatency;

        std::uint64_t stepDeadlineMisses = 0;
        std::uint64_t unmatchedCommandResponses = 0;
    };

    class HilTimingMonitor
    {
    public:
        using Clock = std::chrono::steady_clock;
        using TimePoint = Clock::time_point;
        using Duration = Clock::duration;

        static constexpr std::size_t SENSOR_HISTORY_CAPACITY = 128;

        explicit HilTimingMonitor(
            Duration stepDeadline =
            std::chrono::milliseconds(10));

        // Reject negative elapsed durations.
        bool recordSimulationStep(Duration elapsed);

        bool recordCommunication(Duration elapsed);

        // Record when the complete sensor frame has
        // been accepted by the transport.
        void recordSensorSent(
            std::uint32_t sequence,
            TimePoint sentAt);

        // Calculate response latency for an accepted command.
        // A successful match consumes the timing record.
        bool recordCommandAccepted(
            std::uint32_t referencedSensorSequence,
            TimePoint receivedAt);

        const HilTimingStatistics& statistics() const;

        Duration stepDeadline() const;

        void reset();

    private:
        struct SensorTimingRecord
        {
            bool valid = false;
            std::uint32_t sequence = 0;
            TimePoint sentAt{};
        };

        std::array<
            SensorTimingRecord,
            SENSOR_HISTORY_CAPACITY
        > sensorHistory_{};

        std::size_t nextSensorSlot_ = 0;

        Duration stepDeadline_;
        HilTimingStatistics statistics_{};

        static bool recordDuration(
            HilDurationStatistics& statistics,
            Duration elapsed);
    };
}
