#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace aeroforge::hil
{
    class HilSequenceTracker
    {
    public:
        static constexpr std::size_t HISTORY_CAPACITY = 128;

        explicit HilSequenceTracker(
            std::uint64_t maximumCommandAgeUs = 100000);

        bool recordSentSensor(
            std::uint32_t sequence,
            std::uint64_t simulationTimeUs);

        bool acceptControl(
            std::uint32_t packetSequence,
            std::uint32_t referencedSensorSequence,
            std::uint64_t currentSimulationTimeUs);

        std::optional<std::uint64_t>
            lastAcceptedSensorTimeUs() const;

        void reset();

    private:
        struct SensorRecord
        {
            std::uint32_t sequence = 0;
            std::uint64_t timeUs = 0;
        };

        std::array<SensorRecord, HISTORY_CAPACITY> history_{};

        std::size_t nextRecord_ = 0;
        std::size_t recordCount_ = 0;

        std::uint64_t maximumCommandAgeUs_;

        std::optional<std::uint32_t> lastSensorSequence_;
        std::optional<std::uint64_t> lastSensorTimeUs_;

        std::optional<std::uint32_t> lastControlSequence_;
        std::optional<std::uint32_t> lastReferencedSequence_;
        std::optional<std::uint64_t> lastAcceptedSensorTimeUs_;

        static bool isNewer(
            std::uint32_t incoming,
            std::uint32_t previous);
    };
}