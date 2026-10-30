#include <hil/HilPacketCodec.h>
#include <hil/HilSerializer.h>
#include <hil/HilCrc32.h>
#include <hil/HilCobs.h>

#include <algorithm>
#include <array>
#include <variant>

namespace aeroforge::hil
{
    namespace
    {
        bool validControllerStatus(std::uint8_t status)
        {
            return status <= static_cast<std::uint8_t>(
                ControllerStatus::FAULT);
        }

        bool frameRawPacket(
            std::span<const std::uint8_t> raw,
            std::span<std::uint8_t> output,
            std::size_t& bytesWritten)
        {
            std::array<std::uint8_t,
                MAX_COBS_ENCODED_SIZE> encoded{};

            std::size_t encodedSize = 0;

            if (!cobsEncode(raw, encoded, encodedSize))
            {
                return false;
            }

            if (output.size() < encodedSize + 1)
            {
                return false;
            }

            std::copy_n(
                encoded.begin(),
                encodedSize,
                output.begin());

            output[encodedSize] = 0x00;
            bytesWritten = encodedSize + 1;

            return true;
        }
    }

    bool encodeSensorPacket(
        const SensorSampleMessage& sample,
        std::uint32_t sequence,
        std::span<std::uint8_t> output,
        std::size_t& bytesWritten)
    {
        if (output.size() < SENSOR_PACKET_SIZE)
        {
            return false;
        }

        std::array<std::uint8_t,
            SENSOR_PACKET_SIZE> temporary{};

        HilHeader header{};
        header.messageType = static_cast<std::uint8_t>(
            MessageType::SENSOR_SAMPLE);
        header.payloadLength =
            static_cast<std::uint16_t>(
                SENSOR_SAMPLE_PAYLOAD_SIZE);
        header.sequence = sequence;

        std::size_t offset = 0;

        if (!writeHeader(temporary, offset, header) ||
            !writeSensorSample(temporary, offset, sample))
        {
            return false;
        }

        if (offset !=
            HEADER_SIZE + SENSOR_SAMPLE_PAYLOAD_SIZE)
        {
            return false;
        }

        const std::uint32_t crc = calculateCrc32(
            std::span<const std::uint8_t>(
                temporary.data(), offset));

        if (!writeU32(temporary, offset, crc) ||
            offset != SENSOR_PACKET_SIZE)
        {
            return false;
        }

        std::copy(
            temporary.begin(),
            temporary.end(),
            output.begin());

        bytesWritten = offset;
        return true;
    }

    bool encodeControlPacket(
        const ControlCommandMessage& command,
        std::uint32_t sequence,
        std::span<std::uint8_t> output,
        std::size_t& bytesWritten)
    {
        if (!validControllerStatus(command.controllerStatus) ||
            output.size() < CONTROL_PACKET_SIZE)
        {
            return false;
        }

        std::array<std::uint8_t,
            CONTROL_PACKET_SIZE> temporary{};

        HilHeader header{};
        header.messageType = static_cast<std::uint8_t>(
            MessageType::CONTROL_COMMAND);
        header.payloadLength =
            static_cast<std::uint16_t>(
                CONTROL_COMMAND_PAYLOAD_SIZE);
        header.sequence = sequence;

        std::size_t offset = 0;

        if (!writeHeader(temporary, offset, header) ||
            !writeControlCommand(temporary, offset, command))
        {
            return false;
        }

        if (offset !=
            HEADER_SIZE + CONTROL_COMMAND_PAYLOAD_SIZE)
        {
            return false;
        }

        const std::uint32_t crc = calculateCrc32(
            std::span<const std::uint8_t>(
                temporary.data(), offset));

        if (!writeU32(temporary, offset, crc) ||
            offset != CONTROL_PACKET_SIZE)
        {
            return false;
        }

        std::copy(
            temporary.begin(),
            temporary.end(),
            output.begin());

        bytesWritten = offset;
        return true;
    }

    bool decodePacket(
        std::span<const std::uint8_t> rawPacket,
        HilPacket& result)
    {
        if (rawPacket.size() < HEADER_SIZE + CRC_SIZE ||
            rawPacket.size() > MAX_RAW_PACKET_SIZE)
        {
            return false;
        }

        HilHeader header{};
        std::size_t offset = 0;

        if (!readHeader(rawPacket, offset, header))
        {
            return false;
        }

        const std::size_t expectedSize =
            HEADER_SIZE +
            static_cast<std::size_t>(header.payloadLength) +
            CRC_SIZE;

        if (rawPacket.size() != expectedSize)
        {
            return false;
        }

        const auto sensorType =
            static_cast<std::uint8_t>(
                MessageType::SENSOR_SAMPLE);

        const auto controlType =
            static_cast<std::uint8_t>(
                MessageType::CONTROL_COMMAND);

        if (header.messageType == sensorType)
        {
            if (header.payloadLength !=
                SENSOR_SAMPLE_PAYLOAD_SIZE)
            {
                return false;
            }
        }
        else if (header.messageType == controlType)
        {
            if (header.payloadLength !=
                CONTROL_COMMAND_PAYLOAD_SIZE)
            {
                return false;
            }
        }
        else
        {
            // HELLO, STATUS and FAULT payload schemas
            // are not implemented in this codec yet.
            return false;
        }

        const std::size_t crcOffset =
            rawPacket.size() - CRC_SIZE;

        std::size_t checksumOffset = crcOffset;
        std::uint32_t receivedCrc = 0;

        if (!readU32(
            rawPacket,
            checksumOffset,
            receivedCrc))
        {
            return false;
        }

        if (checksumOffset != rawPacket.size())
        {
            return false;
        }

        const std::uint32_t computedCrc =
            calculateCrc32(rawPacket.first(crcOffset));

        if (receivedCrc != computedCrc)
        {
            return false;
        }

        HilPacket temporary{};
        temporary.header = header;

        if (header.messageType == sensorType)
        {
            SensorSampleMessage sample{};

            if (!readSensorSample(
                rawPacket, offset, sample))
            {
                return false;
            }

            temporary.payload = sample;
        }
        else
        {
            ControlCommandMessage command{};

            if (!readControlCommand(
                rawPacket, offset, command))
            {
                return false;
            }

            if (!validControllerStatus(
                command.controllerStatus))
            {
                return false;
            }

            temporary.payload = command;
        }

        if (offset != crcOffset)
        {
            return false;
        }

        result = temporary;
        return true;
    }

    bool encodeSensorFrame(
        const SensorSampleMessage& sample,
        std::uint32_t sequence,
        std::span<std::uint8_t> output,
        std::size_t& bytesWritten)
    {
        std::array<std::uint8_t, SENSOR_PACKET_SIZE> raw{};
        std::size_t rawSize = 0;

        if (!encodeSensorPacket(
            sample, sequence, raw, rawSize))
        {
            return false;
        }

        return frameRawPacket(
            std::span<const std::uint8_t>(
                raw.data(), rawSize),
            output,
            bytesWritten);
    }

    bool encodeControlFrame(
        const ControlCommandMessage& command,
        std::uint32_t sequence,
        std::span<std::uint8_t> output,
        std::size_t& bytesWritten)
    {
        std::array<std::uint8_t, CONTROL_PACKET_SIZE> raw{};
        std::size_t rawSize = 0;

        if (!encodeControlPacket(
            command, sequence, raw, rawSize))
        {
            return false;
        }

        return frameRawPacket(
            std::span<const std::uint8_t>(
                raw.data(), rawSize),
            output,
            bytesWritten);
    }
}