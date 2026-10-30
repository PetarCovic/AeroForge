#pragma once

#include <hil/HilMessage.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace aeroforge::hil
{
    struct HilParserStats
    {
        std::uint64_t completedFrames = 0;
        std::uint64_t malformedFrames = 0;
        std::uint64_t oversizedFrames = 0;
        std::uint64_t droppedFrames = 0;
        std::uint64_t emptyFrames = 0;
    };

    class HilStreamParser
    {
    public:
        static constexpr std::size_t QUEUE_CAPACITY = 8;

        void consume(std::span<const std::uint8_t> bytes);

        bool tryPopPacket(
            std::span<std::uint8_t> output,
            std::size_t& bytesWritten);

        void reset();

        const HilParserStats& stats() const;

    private:
        std::array<std::uint8_t,
            MAX_COBS_ENCODED_SIZE> frame_{};

        std::size_t frameLength_ = 0;
        bool discardingFrame_ = false;

        std::array<
            std::array<std::uint8_t, MAX_RAW_PACKET_SIZE>,
            QUEUE_CAPACITY
        > packets_{};

        std::array<std::size_t, QUEUE_CAPACITY> lengths_{};

        std::size_t head_ = 0;
        std::size_t tail_ = 0;
        std::size_t count_ = 0;

        HilParserStats stats_{};

        void finishFrame();
    };
}