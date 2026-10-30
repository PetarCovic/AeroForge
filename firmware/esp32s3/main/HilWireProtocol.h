#pragma once

#include <cstddef>
#include <cstdint>

namespace aeroforge::hil::firmware
{
    inline constexpr std::size_t MAX_RAW_PACKET_SIZE = 128;
    inline constexpr std::size_t MAX_ENCODED_FRAME_SIZE = 130;

    struct SensorSample
    {
        std::uint32_t sequence = 0;
        std::uint64_t simulationTimeUs = 0;

        float specificForceB[3]{};
        float angularVelocityB[3]{};
    };

    struct ControlCommand
    {
        std::uint32_t sequence = 0;
        std::uint32_t referencedSensorSequence = 0;

        float commandedMomentB[3]{};

        std::uint8_t controllerStatus = 0;
    };

    // Input excludes the trailing 0x00 delimiter.
    bool decodeSensorFrame(
        const std::uint8_t* encoded,
        std::size_t encodedLength,
        SensorSample& output) noexcept;

    // Output includes the trailing 0x00 delimiter.
    // Returns zero on failure.
    std::size_t encodeControlFrame(
        const ControlCommand& command,
        std::uint8_t* output,
        std::size_t capacity) noexcept;

    std::uint32_t crc32(
        const std::uint8_t* data,
        std::size_t length) noexcept;
}
