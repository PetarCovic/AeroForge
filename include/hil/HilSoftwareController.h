#pragma once

#include <hil/HilWireEndpoint.h>
#include <math/Vector3.h>

#include <cstdint>
#include <optional>

namespace aeroforge::hil
{
    struct HilRateControllerConfig
    {
        math::Vector3 rateGain =
            math::Vector3(5.0, 5.0, 5.0);

        math::Vector3 maximumAbsMomentB =
            math::Vector3(20.0, 20.0, 20.0);
    };

    struct HilControllerStats
    {
        std::uint64_t receivedSamples = 0;
        std::uint64_t generatedCommands = 0;
        std::uint64_t rejectedSamples = 0;
        std::uint64_t droppedCommands = 0;
    };

    class HilSoftwareController
    {
    public:
        HilSoftwareController(
            HilWireEndpoint& endpoint,
            const HilRateControllerConfig& config);

        void pump();

        void reset();

        const HilControllerStats& stats() const;

    private:
        HilWireEndpoint& endpoint_;
        HilRateControllerConfig config_;

        std::uint32_t nextCommandSequence_ = 0;

        std::optional<std::uint32_t> lastSensorSequence_;
        std::optional<std::uint64_t> lastSensorTimeUs_;

        HilControllerStats stats_{};

        math::Vector3 computeRateCommand(
            const math::Vector3& angularVelocityB) const;

        static bool isNewer(
            std::uint32_t incoming,
            std::uint32_t previous);
    };
}