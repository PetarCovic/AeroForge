#pragma once

#include <hil/HilWireEndpoint.h>
#include <hil/HilSequenceTracker.h>
#include <hil/HilCommandWatchdog.h>
#include <hil/HilTimingMonitor.h>

#include <math/Vector3.h>
#include <sensors/ImuModel.h>

#include <chrono>
#include <cstdint>
#include <optional>

namespace aeroforge::hil
{
    struct HilBridgeConfig
    {
        // Maximum permitted simulation-time age of
        // a controller command's referenced sensor sample.
        std::uint64_t maximumCommandAgeUs = 100000;

        // Maximum wall-clock interval between accepted
        // controller commands.
        std::chrono::milliseconds wallClockCommandTimeout{ 100 };

        // Per-axis absolute commanded body-moment limits.
        math::Vector3 maximumAbsMomentB =
            math::Vector3(20.0, 20.0, 20.0);

        // Whether commands reporting DEGRADED status
        // may still be accepted.
        bool acceptDegradedCommands = false;
    };

    struct HilBridgeStats
    {
        std::uint64_t queuedSamples = 0;
        std::uint64_t droppedSamples = 0;

        std::uint64_t acceptedCommands = 0;
        std::uint64_t rejectedCommands = 0;
    };

    class HilSimulationBridge
    {
    public:
        HilSimulationBridge(
            HilWireEndpoint& endpoint,
            const HilBridgeConfig& config);

        // Queue an IMU measurement for transmission.
        // Returns false if encoding fails or the
        // endpoint already has a pending frame.
        bool publishSensorSample(
            const sensors::ImuMeasurement& measurement);

        // Progress outgoing transmission, receive
        // incoming packets and validate commands.
        //
        // Call from the simulation thread.
        void pump(
            std::uint64_t currentSimulationTimeUs);

        // Return the most recent valid body-moment command.
        // Returns zero if the command is stale, missing,
        // timed out, or the session has faulted.
        math::Vector3 currentCommandedMomentB(
            std::uint64_t currentSimulationTimeUs) const;

        // Explicitly reset the HIL communication session.
        void reset();

        // Notify the bridge that the transport has failed.
        // This latches the watchdog into DISCONNECTED.
        void notifyTransportDisconnected();

        // Return the current wall-clock watchdog state.
        HilWatchdogState watchdogState() const;

        // Attach an optional timing monitor.
        // The monitor must outlive its attachment.
        void attachTimingMonitor(
            HilTimingMonitor* monitor);

        // Access bridge-level counters.
        const HilBridgeStats& stats() const;

        // Convert simulation seconds to microseconds.
        // The output is unchanged if conversion fails.
        static bool secondsToMicroseconds(
            double seconds,
            std::uint64_t& result);

    private:
        HilWireEndpoint& endpoint_;

        HilBridgeConfig config_;

        HilCommandWatchdog watchdog_;
        HilSequenceTracker tracker_;

        std::uint32_t nextSensorSequence_ = 0;

        math::Vector3 commandedMomentB_{};

        std::optional<std::uint64_t>
            acceptedSensorTimeUs_;

        HilBridgeStats stats_{};

        // Non-owning; nullptr disables instrumentation.
        HilTimingMonitor* timingMonitor_ = nullptr;

        bool momentWithinLimits(
            const math::Vector3& moment) const;
    };
}
