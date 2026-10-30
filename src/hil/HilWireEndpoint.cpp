#include <hil/HilWireEndpoint.h>

#include <array>
#include <stdexcept>
#include <utility>

namespace aeroforge::hil
{
    HilWireEndpoint::HilWireEndpoint(HilBytePort port)
        : port_(std::move(port))
    {
        if (!port_.tryRead || !port_.tryWrite)
        {
            throw std::invalid_argument(
                "HIL byte port requires read/write callbacks");
        }
    }

    bool HilWireEndpoint::queueSensor(
        const SensorSampleMessage& sample,
        std::uint32_t sequence)
    {
        if (hasPendingTransmission())
        {
            return false;
        }

        std::size_t frameSize = 0;

        if (!encodeSensorFrame(
            sample, sequence, pending_, frameSize))
        {
            return false;
        }

        pendingLength_ = frameSize;
        pendingOffset_ = 0;

        pendingReceipt_ = HilSendReceipt{
            MessageType::SENSOR_SAMPLE,
            sequence,
            sample.simulationTimeUs
        };

        return true;
    }

    bool HilWireEndpoint::queueControl(
        const ControlCommandMessage& command,
        std::uint32_t sequence)
    {
        if (hasPendingTransmission())
        {
            return false;
        }

        std::size_t frameSize = 0;

        if (!encodeControlFrame(
            command, sequence, pending_, frameSize))
        {
            return false;
        }

        pendingLength_ = frameSize;
        pendingOffset_ = 0;

        pendingReceipt_ = HilSendReceipt{
            MessageType::CONTROL_COMMAND,
            sequence,
            0
        };

        return true;
    }

    std::optional<HilSendReceipt>
        HilWireEndpoint::flushPending()
    {
        if (!hasPendingTransmission())
        {
            return std::nullopt;
        }

        const std::span<const std::uint8_t> remaining(
            pending_.data() + pendingOffset_,
            pendingLength_ - pendingOffset_);

        const std::size_t written =
            port_.tryWrite(remaining);

        if (written > remaining.size())
        {
            throw std::runtime_error(
                "HIL transport reported invalid write size");
        }

        pendingOffset_ += written;

        if (pendingOffset_ < pendingLength_)
        {
            return std::nullopt;
        }

        const auto receipt = pendingReceipt_;

        pendingLength_ = 0;
        pendingOffset_ = 0;
        pendingReceipt_.reset();

        return receipt;
    }

    void HilWireEndpoint::pollIncoming(
        std::size_t maxReads)
    {
        std::array<std::uint8_t, 256> buffer{};

        for (std::size_t i = 0; i < maxReads; ++i)
        {
            const std::size_t received =
                port_.tryRead(buffer);

            if (received > buffer.size())
            {
                throw std::runtime_error(
                    "HIL transport reported invalid read size");
            }

            if (received == 0)
            {
                break;
            }

            parser_.consume(
                std::span<const std::uint8_t>(
                    buffer.data(), received));
        }
    }

    bool HilWireEndpoint::tryReceive(
        HilPacket& packet)
    {
        std::array<std::uint8_t,
            MAX_RAW_PACKET_SIZE> raw{};

        std::size_t rawSize = 0;

        while (parser_.tryPopPacket(raw, rawSize))
        {
            HilPacket decoded{};

            if (!decodePacket(
                std::span<const std::uint8_t>(
                    raw.data(), rawSize),
                decoded))
            {
                ++invalidPackets_;
                continue;
            }

            packet = std::move(decoded);
            return true;
        }

        return false;
    }

    bool HilWireEndpoint::hasPendingTransmission() const
    {
        return pendingLength_ != 0;
    }

    std::uint64_t
        HilWireEndpoint::invalidPacketCount() const
    {
        return invalidPackets_;
    }

    const HilParserStats&
        HilWireEndpoint::parserStats() const
    {
        return parser_.stats();
    }

    void HilWireEndpoint::reset()
    {
        parser_.reset();

        pendingLength_ = 0;
        pendingOffset_ = 0;
        pendingReceipt_.reset();

        invalidPackets_ = 0;
    }
}