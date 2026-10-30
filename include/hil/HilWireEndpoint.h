#pragma once

#include <hil/HilPacketCodec.h>
#include <hil/HilStreamParser.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>

namespace aeroforge::hil
{
    struct HilBytePort
    {
        // Return number of bytes read/written.
        // Return zero when no progress can be made.
        std::function<std::size_t(
            std::span<std::uint8_t>)> tryRead;

        std::function<std::size_t(
            std::span<const std::uint8_t>)> tryWrite;
    };

    struct HilSendReceipt
    {
        MessageType type;
        std::uint32_t sequence;
        std::uint64_t sensorTimeUs;
    };

    class HilWireEndpoint
    {
    public:
        explicit HilWireEndpoint(HilBytePort port);

        bool queueSensor(
            const SensorSampleMessage& sample,
            std::uint32_t sequence);

        bool queueControl(
            const ControlCommandMessage& command,
            std::uint32_t sequence);

        std::optional<HilSendReceipt> flushPending();

        void pollIncoming(std::size_t maxReads = 4);

        bool tryReceive(HilPacket& packet);

        bool hasPendingTransmission() const;

        std::uint64_t invalidPacketCount() const;

        const HilParserStats& parserStats() const;

        void reset();

    private:
        HilBytePort port_;
        HilStreamParser parser_;

        std::array<std::uint8_t,
            MAX_FRAMED_PACKET_SIZE> pending_{};

        std::size_t pendingLength_ = 0;
        std::size_t pendingOffset_ = 0;

        std::optional<HilSendReceipt> pendingReceipt_;

        std::uint64_t invalidPackets_ = 0;
    };
}