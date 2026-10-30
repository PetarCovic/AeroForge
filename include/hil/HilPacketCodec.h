#pragma once

#include <hil/HilMessage.h>

#include <cstddef>
#include <cstdint>
#include <span>
#include <variant>

namespace aeroforge::hil
{
    using HilPayload = std::variant<SensorSampleMessage, ControlCommandMessage>;

    struct HilPacket
    {
        HilHeader header{};
        HilPayload payload{};
    };

    bool encodeSensorPacket(
        const SensorSampleMessage& sample,
        std::uint32_t sequence,
        std::span<std::uint8_t> output,
        std::size_t& bytesWritten
    );

    bool encodeControlPacket(
        const ControlCommandMessage& command,
        std::uint32_t sequence,
        std::span<std::uint8_t> output,
        std::size_t& bytesWritten
    );

    bool decodePacket(
        std::span<const std::uint8_t> rawPacket,
        HilPacket& result
    );

    bool encodeSensorFrame(
        const SensorSampleMessage& sample,
        std::uint32_t sequence,
        std::span<std::uint8_t> output,
        std::size_t& bytesWritten
    );

    bool encodeControlFrame(
        const ControlCommandMessage& command,
        std::uint32_t sequence,
        std::span<std::uint8_t> output,
        std::size_t& bytesWritten
    );
}