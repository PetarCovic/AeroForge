#include <hil/HilSimulationBridge.h>

#include <cmath>
#include <stdexcept>
#include <utility>
#include <variant>

namespace aeroforge::hil
{
    HilSimulationBridge::HilSimulationBridge(
        HilWireEndpoint& endpoint,
        const HilBridgeConfig& config)
        : endpoint_(endpoint),
        config_(config),
        watchdog_(config.wallClockCommandTimeout),
        tracker_(config.maximumCommandAgeUs)
    {
        const auto& maximum = config_.maximumAbsMomentB;

        if (!maximum.isFinite() ||
            maximum.x < 0.0 ||
            maximum.y < 0.0 ||
            maximum.z < 0.0)
        {
            throw std::invalid_argument(
                "HIL moment limits must be finite and nonnegative");
        }
    }

    bool HilSimulationBridge::secondsToMicroseconds(
        double seconds,
        std::uint64_t& result)
    {
        if (!std::isfinite(seconds) || seconds < 0.0)
        {
            return false;
        }

        const long double microseconds =
            std::round(
                static_cast<long double>(seconds) *
                1000000.0L);

        // 2^64 is the first value that cannot be
        // represented by an unsigned 64-bit integer.
        const long double upperExclusive =
            std::ldexp(1.0L, 64);

        if (!std::isfinite(microseconds) ||
            microseconds < 0.0L ||
            microseconds >= upperExclusive)
        {
            return false;
        }

        result = static_cast<std::uint64_t>(
            microseconds);

        return true;
    }

    bool HilSimulationBridge::publishSensorSample(
        const sensors::ImuMeasurement& measurement)
    {
        std::uint64_t timeUs = 0;

        if (!secondsToMicroseconds(
            measurement.timeS, timeUs))
        {
            ++stats_.droppedSamples;
            return false;
        }

        SensorSampleMessage sample{};

        sample.simulationTimeUs = timeUs;

        sample.specificForceB =
            measurement.specificForceB;

        sample.angularVelocityB =
            measurement.angularVelocityB;

        // The endpoint is responsible for serialization,
        // packet checksums, COBS framing, and buffering.
        if (!endpoint_.queueSensor(
            sample, nextSensorSequence_))
        {
            ++stats_.droppedSamples;
            return false;
        }

        // Only advance the sequence after the endpoint
        // successfully accepts the outgoing frame.
        ++nextSensorSequence_;
        ++stats_.queuedSamples;

        return true;
    }

    bool HilSimulationBridge::momentWithinLimits(
        const math::Vector3& moment) const
    {
        if (!moment.isFinite())
        {
            return false;
        }

        const auto& maximum =
            config_.maximumAbsMomentB;

        return
            std::abs(moment.x) <= maximum.x &&
            std::abs(moment.y) <= maximum.y &&
            std::abs(moment.z) <= maximum.z;
    }

    void HilSimulationBridge::pump(
        std::uint64_t currentSimulationTimeUs)
    {
        const auto pumpStart =
            HilTimingMonitor::Clock::now();

        // --------------------------------------------------
        // 1. Progress pending outgoing sensor transmission.
        // --------------------------------------------------

        const auto receipt = endpoint_.flushPending();

        if (receipt &&
            receipt->type == MessageType::SENSOR_SAMPLE)
        {
            // A sensor sequence becomes eligible for
            // controller referencing only after the
            // entire frame has been accepted for sending.
            const bool recorded = tracker_.recordSentSensor(
                receipt->sequence,
                receipt->sensorTimeUs);

            if (recorded && timingMonitor_ != nullptr)
            {
                timingMonitor_->recordSensorSent(
                    receipt->sequence,
                    HilTimingMonitor::Clock::now());
            }
        }

        // --------------------------------------------------
        // 2. Read currently available incoming bytes.
        // --------------------------------------------------

        endpoint_.pollIncoming();

        // --------------------------------------------------
        // 3. Process decoded packets.
        // --------------------------------------------------

        HilPacket packet{};

        while (endpoint_.tryReceive(packet))
        {
            // Only controller commands are expected
            // on the simulation side.
            if (packet.header.messageType !=
                static_cast<std::uint8_t>(
                    MessageType::CONTROL_COMMAND))
            {
                continue;
            }

            const auto* command =
                std::get_if<ControlCommandMessage>(
                    &packet.payload);

            if (command == nullptr)
            {
                ++stats_.rejectedCommands;
                continue;
            }

            // ----------------------------------------------
            // 3a. Validate controller status.
            // ----------------------------------------------

            const auto status =
                static_cast<ControllerStatus>(
                    command->controllerStatus);

            if (status == ControllerStatus::FAULT)
            {
                // An explicitly reported controller fault
                // permanently disarms this session
                // until a deliberate reset.
                watchdog_.markFault();

                commandedMomentB_ = math::Vector3();
                acceptedSensorTimeUs_.reset();

                ++stats_.rejectedCommands;
                continue;
            }

            if (status == ControllerStatus::DEGRADED &&
                !config_.acceptDegradedCommands)
            {
                ++stats_.rejectedCommands;
                continue;
            }

            // ----------------------------------------------
            // 3b. Validate command limits and finiteness.
            // ----------------------------------------------

            if (!momentWithinLimits(
                command->commandedMomentB))
            {
                ++stats_.rejectedCommands;
                continue;
            }

            // ----------------------------------------------
            // 3c. Check watchdog state before modifying
            //     the sequence tracker.
            // ----------------------------------------------

            const auto acceptedAt =
                HilCommandWatchdog::Clock::now();

            const HilWatchdogState state =
                watchdog_.state(acceptedAt);

            // WAITING is permitted because the first valid
            // command is responsible for arming the watchdog.
            //
            // HEALTHY is permitted for ongoing commands.
            //
            // TIMED_OUT, DISCONNECTED, and FAULTED require
            // an explicit session reset.
            if (state != HilWatchdogState::WAITING &&
                state != HilWatchdogState::HEALTHY)
            {
                ++stats_.rejectedCommands;
                continue;
            }

            // ----------------------------------------------
            // 3d. Validate sequence and simulation-time age.
            // ----------------------------------------------

            if (!tracker_.acceptControl(
                packet.header.sequence,
                command->referencedSensorSequence,
                currentSimulationTimeUs))
            {
                ++stats_.rejectedCommands;
                continue;
            }

            // ----------------------------------------------
            // 3e. Refresh or arm the wall-clock watchdog.
            // ----------------------------------------------

            if (!watchdog_.observeAcceptedCommand(
                acceptedAt))
            {
                ++stats_.rejectedCommands;
                continue;
            }

            // ----------------------------------------------
            // 3f. Commit the accepted command.
            // ----------------------------------------------

            commandedMomentB_ =
                command->commandedMomentB;

            acceptedSensorTimeUs_ =
                tracker_.lastAcceptedSensorTimeUs();

            ++stats_.acceptedCommands;

            // ----------------------------------------------
            // 3g. Record end-to-end command response latency.
            // ----------------------------------------------

            if (timingMonitor_ != nullptr)
            {
                timingMonitor_->recordCommandAccepted(
                    command->referencedSensorSequence,
                    acceptedAt);
            }
        }

        // --------------------------------------------------
        // 4. Record this entire communication cycle.
        // --------------------------------------------------
        //
        // This must be outside the packet-processing loop.
        // One pump() call produces one duration measurement,
        // even if no controller packet was received.

        if (timingMonitor_ != nullptr)
        {
            timingMonitor_->recordCommunication(
                HilTimingMonitor::Clock::now() - pumpStart);
        }
    }

    math::Vector3
        HilSimulationBridge::currentCommandedMomentB(
            std::uint64_t currentSimulationTimeUs) const
    {
        // --------------------------------------------------
        // 1. Check wall-clock command freshness.
        // --------------------------------------------------

        if (!watchdog_.permitsCommand(
            HilCommandWatchdog::Clock::now()))
        {
            return math::Vector3();
        }

        // --------------------------------------------------
        // 2. Require an accepted sensor reference.
        // --------------------------------------------------

        if (!acceptedSensorTimeUs_)
        {
            return math::Vector3();
        }

        // --------------------------------------------------
        // 3. Reject commands referencing future time.
        // --------------------------------------------------

        if (currentSimulationTimeUs <
            *acceptedSensorTimeUs_)
        {
            return math::Vector3();
        }

        // --------------------------------------------------
        // 4. Enforce simulation-time command age.
        // --------------------------------------------------

        const std::uint64_t age =
            currentSimulationTimeUs -
            *acceptedSensorTimeUs_;

        if (age > config_.maximumCommandAgeUs)
        {
            return math::Vector3();
        }

        // --------------------------------------------------
        // 5. Return the most recent accepted command.
        // --------------------------------------------------

        return commandedMomentB_;
    }

    void HilSimulationBridge::notifyTransportDisconnected()
    {
        // This is a latched fault state.
        // Reconnection alone must not rearm commands.
        watchdog_.markDisconnected();

        commandedMomentB_ = math::Vector3();
        acceptedSensorTimeUs_.reset();
    }

    HilWatchdogState
        HilSimulationBridge::watchdogState() const
    {
        return watchdog_.state(
            HilCommandWatchdog::Clock::now());
    }

    void HilSimulationBridge::attachTimingMonitor(
        HilTimingMonitor* monitor)
    {
        // The monitor is not owned by the bridge.
        // Attach before running the simulation.
        timingMonitor_ = monitor;
    }

    void HilSimulationBridge::reset()
    {
        // Reset all communication and validation state.
        // The caller must ensure the physical session
        // has been deliberately restarted/synchronized.
        endpoint_.reset();
        tracker_.reset();
        watchdog_.reset();

        nextSensorSequence_ = 0;

        commandedMomentB_ = math::Vector3();
        acceptedSensorTimeUs_.reset();

        stats_ = {};

        // The attached timing monitor is intentionally
        // not reset here because the bridge does not
        // own that object.
        //
        // The session coordinator can separately call:
        // timingMonitor.reset();
    }

    const HilBridgeStats&
        HilSimulationBridge::stats() const
    {
        return stats_;
    }
}