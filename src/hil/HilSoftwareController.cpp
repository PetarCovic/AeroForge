#include <hil/HilSoftwareController.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <variant>

namespace aeroforge::hil
{
    HilSoftwareController::HilSoftwareController(
        HilWireEndpoint& endpoint,
        const HilRateControllerConfig& config)
        : endpoint_(endpoint),
        config_(config)
    {
        const auto& gains = config_.rateGain;
        const auto& limits = config_.maximumAbsMomentB;

        if (!gains.isFinite() ||
            !limits.isFinite() ||
            gains.x < 0.0 ||
            gains.y < 0.0 ||
            gains.z < 0.0 ||
            limits.x < 0.0 ||
            limits.y < 0.0 ||
            limits.z < 0.0)
        {
            throw std::invalid_argument(
                "Invalid HIL rate-controller configuration");
        }
    }

    bool HilSoftwareController::isNewer(
        std::uint32_t incoming,
        std::uint32_t previous)
    {
        const std::uint32_t difference =
            incoming - previous;

        return difference != 0u &&
            difference < 0x80000000u;
    }

    math::Vector3
        HilSoftwareController::computeRateCommand(
            const math::Vector3& angularVelocityB) const
    {
        const auto& gains = config_.rateGain;
        const auto& limits = config_.maximumAbsMomentB;

        const auto computeAxis =
            [](double gain, double rate, double limit)
            {
                const double requested = -gain * rate;

                return std::clamp(
                    requested,
                    -limit,
                    limit);
            };

        return math::Vector3(
            computeAxis(
                gains.x,
                angularVelocityB.x,
                limits.x),
            computeAxis(
                gains.y,
                angularVelocityB.y,
                limits.y),
            computeAxis(
                gains.z,
                angularVelocityB.z,
                limits.z)
        );
    }

    void HilSoftwareController::pump()
    {
        // Finish transmitting any previous command.
        endpoint_.flushPending();

        // Receive available sensor frames.
        endpoint_.pollIncoming();

        HilPacket packet{};

        while (endpoint_.tryReceive(packet))
        {
            if (packet.header.messageType !=
                static_cast<std::uint8_t>(
                    MessageType::SENSOR_SAMPLE))
            {
                continue;
            }

            const auto* sensor =
                std::get_if<SensorSampleMessage>(
                    &packet.payload);

            if (sensor == nullptr)
            {
                ++stats_.rejectedSamples;
                continue;
            }

            if (lastSensorSequence_ &&
                !isNewer(
                    packet.header.sequence,
                    *lastSensorSequence_))
            {
                ++stats_.rejectedSamples;
                continue;
            }

            if (lastSensorTimeUs_ &&
                sensor->simulationTimeUs <
                *lastSensorTimeUs_)
            {
                ++stats_.rejectedSamples;
                continue;
            }

            lastSensorSequence_ =
                packet.header.sequence;

            lastSensorTimeUs_ =
                sensor->simulationTimeUs;

            ++stats_.receivedSamples;

            // Do not overwrite a partially sent frame.
            if (endpoint_.hasPendingTransmission())
            {
                ++stats_.droppedCommands;
                continue;
            }

            const math::Vector3 commandMoment =
                computeRateCommand(
                    sensor->angularVelocityB);

            if (!commandMoment.isFinite())
            {
                ++stats_.rejectedSamples;
                continue;
            }

            ControlCommandMessage command{};
            command.referencedSensorSequence =
                packet.header.sequence;

            command.commandedMomentB = commandMoment;

            command.controllerStatus =
                static_cast<std::uint8_t>(
                    ControllerStatus::OK);

            if (!endpoint_.queueControl(
                command, nextCommandSequence_))
            {
                ++stats_.droppedCommands;
                continue;
            }

            ++nextCommandSequence_;
            ++stats_.generatedCommands;
        }

        // Attempt to transmit the newest queued command.
        endpoint_.flushPending();
    }

    void HilSoftwareController::reset()
    {
        endpoint_.reset();

        nextCommandSequence_ = 0;

        lastSensorSequence_.reset();
        lastSensorTimeUs_.reset();

        stats_ = {};
    }

    const HilControllerStats&
        HilSoftwareController::stats() const
    {
        return stats_;
    }
}