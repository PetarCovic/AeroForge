#pragma once

#include <hil/HilCommandWatchdog.h>
#include <hil/HilSerialWorker.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>

namespace aeroforge::hil
{
    enum class HilLinkMode
    {
        OFFLINE,
        SOFTWARE_LOOPBACK,
        SERIAL
    };

    struct HilTelemetryPoint
    {
        double simulationTimeS = 0.0;
        double stepExecutionMs = 0.0;
        double communicationMs = 0.0;
        // -1 means that no new command latency was measured this step.
        double commandLatencyMs = -1.0;
    };

    struct HilHealthSnapshot
    {
        HilLinkMode mode = HilLinkMode::OFFLINE;
        HilSerialWorkerState serialState = HilSerialWorkerState::STOPPED;
        HilWatchdogState watchdogState = HilWatchdogState::WAITING;
        bool running = false;

        std::uint64_t queuedSamples = 0;
        std::uint64_t droppedSamples = 0;
        std::uint64_t acceptedCommands = 0;
        std::uint64_t rejectedCommands = 0;
        std::uint64_t deadlineMisses = 0;
        std::uint64_t stepCount = 0;
        std::uint64_t commandLatencyCount = 0;
        std::uint64_t serialRxDroppedBytes = 0;
        std::uint32_t serialLastError = 0;

        double targetPeriodMs = 10.0;
        double averageStepMs = 0.0;
        double maximumStepMs = 0.0;
        double averageCommandLatencyMs = 0.0;
        double maximumCommandLatencyMs = 0.0;
    };

    struct HilTelemetrySnapshot
    {
        static constexpr std::size_t CAPACITY = 512;
        std::array<HilTelemetryPoint, CAPACITY> points{};
        std::size_t count = 0;
        std::uint64_t revision = 0;
        HilHealthSnapshot health{};
    };

    // Publish on the simulation thread; read on the GUI thread.
    // Fixed memory; history overwrites its oldest sample when full.
    class HilTelemetryStore
    {
    public:
        void publish(const HilTelemetryPoint& point,
            const HilHealthSnapshot& health);
        void publishHealth(const HilHealthSnapshot& health);
        HilTelemetrySnapshot snapshot() const;
        void reset();

    private:
        mutable std::mutex mutex_;
        std::array<HilTelemetryPoint, HilTelemetrySnapshot::CAPACITY> ring_{};
        std::size_t next_ = 0;
        std::size_t count_ = 0;
        std::uint64_t revision_ = 0;
        HilHealthSnapshot health_{};
    };
}
