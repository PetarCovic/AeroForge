#include <hil/HilStreamParser.h>
#include <hil/HilCobs.h>

#include <algorithm>

namespace aeroforge::hil
{
    void HilStreamParser::consume(
        std::span<const std::uint8_t> bytes)
    {
        for (const std::uint8_t byte : bytes)
        {
            if (byte == 0x00)
            {
                if (discardingFrame_)
                {
                    discardingFrame_ = false;
                    frameLength_ = 0;
                    continue;
                }

                if (frameLength_ == 0)
                {
                    ++stats_.emptyFrames;
                    continue;
                }

                finishFrame();
                frameLength_ = 0;
                continue;
            }

            if (discardingFrame_)
            {
                continue;
            }

            if (frameLength_ >= frame_.size())
            {
                ++stats_.oversizedFrames;
                discardingFrame_ = true;
                frameLength_ = 0;
                continue;
            }

            frame_[frameLength_++] = byte;
        }
    }

    void HilStreamParser::finishFrame()
    {
        std::array<std::uint8_t,
            MAX_RAW_PACKET_SIZE> decoded{};

        std::size_t decodedSize = 0;

        const std::span<const std::uint8_t> encoded(
            frame_.data(), frameLength_);

        if (!cobsDecode(encoded, decoded, decodedSize))
        {
            ++stats_.malformedFrames;
            return;
        }

        if (count_ >= QUEUE_CAPACITY)
        {
            ++stats_.droppedFrames;
            return;
        }

        packets_[tail_] = decoded;
        lengths_[tail_] = decodedSize;

        tail_ = (tail_ + 1) % QUEUE_CAPACITY;
        ++count_;
        ++stats_.completedFrames;
    }

    bool HilStreamParser::tryPopPacket(
        std::span<std::uint8_t> output,
        std::size_t& bytesWritten)
    {
        if (count_ == 0)
        {
            return false;
        }

        const std::size_t packetLength = lengths_[head_];

        if (output.size() < packetLength)
        {
            return false;
        }

        std::copy_n(
            packets_[head_].begin(),
            packetLength,
            output.begin());

        bytesWritten = packetLength;

        head_ = (head_ + 1) % QUEUE_CAPACITY;
        --count_;

        return true;
    }

    void HilStreamParser::reset()
    {
        frameLength_ = 0;
        discardingFrame_ = false;

        head_ = 0;
        tail_ = 0;
        count_ = 0;

        stats_ = {};
    }

    const HilParserStats& HilStreamParser::stats() const
    {
        return stats_;
    }
}