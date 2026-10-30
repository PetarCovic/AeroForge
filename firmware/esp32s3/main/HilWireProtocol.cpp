#include "HilWireProtocol.h"

#include <cmath>
#include <cstring>
#include <limits>

namespace aeroforge::hil::firmware
{
    namespace
    {
        constexpr std::uint8_t VERSION = 1;

        constexpr std::uint8_t SENSOR_SAMPLE = 2;
        constexpr std::uint8_t CONTROL_COMMAND = 3;

        constexpr std::size_t HEADER_SIZE = 8;
        constexpr std::size_t CRC_SIZE = 4;

        constexpr std::size_t SENSOR_SIZE = 32;
        constexpr std::size_t COMMAND_SIZE = 17;

        static_assert(sizeof(float) == 4);
        static_assert(
            std::numeric_limits<float>::is_iec559);

        std::uint16_t load16(
            const std::uint8_t* p) noexcept
        {
            return
                static_cast<std::uint16_t>(p[0]) |
                static_cast<std::uint16_t>(p[1]) << 8;
        }

        std::uint32_t load32(
            const std::uint8_t* p) noexcept
        {
            return
                static_cast<std::uint32_t>(p[0]) |
                static_cast<std::uint32_t>(p[1]) << 8 |
                static_cast<std::uint32_t>(p[2]) << 16 |
                static_cast<std::uint32_t>(p[3]) << 24;
        }

        std::uint64_t load64(
            const std::uint8_t* p) noexcept
        {
            return
                static_cast<std::uint64_t>(
                    load32(p)) |
                static_cast<std::uint64_t>(
                    load32(p + 4)) << 32;
        }

        float loadFloat(
            const std::uint8_t* p) noexcept
        {
            const std::uint32_t bits =
                load32(p);

            float value = 0.0f;

            std::memcpy(
                &value,
                &bits,
                sizeof(value));

            return value;
        }

        void store16(
            std::uint8_t* p,
            std::uint16_t value) noexcept
        {
            p[0] = static_cast<std::uint8_t>(value);
            p[1] = static_cast<std::uint8_t>(
                value >> 8);
        }

        void store32(
            std::uint8_t* p,
            std::uint32_t value) noexcept
        {
            p[0] = static_cast<std::uint8_t>(value);
            p[1] = static_cast<std::uint8_t>(
                value >> 8);
            p[2] = static_cast<std::uint8_t>(
                value >> 16);
            p[3] = static_cast<std::uint8_t>(
                value >> 24);
        }

        float unusedFloatPlaceholder() = delete;

        void storeFloat(
            std::uint8_t* p,
            float value) noexcept
        {
            std::uint32_t bits = 0;

            std::memcpy(
                &bits,
                &value,
                sizeof(bits));

            store32(p, bits);
        }

        // COBS decoding.
        // Stream delimiter is handled separately.
        std::size_t cobsDecode(
            const std::uint8_t* source,
            std::size_t length,
            std::uint8_t* dest,
            std::size_t capacity) noexcept
        {
            if (!source || !dest || length == 0)
            {
                return 0;
            }

            std::size_t read = 0;
            std::size_t written = 0;

            while (read < length)
            {
                const std::uint8_t code =
                    source[read++];

                if (code == 0)
                {
                    return 0;
                }

                const std::size_t bytes =
                    static_cast<std::size_t>(
                        code - 1);

                if (bytes > length - read ||
                    bytes > capacity - written)
                {
                    return 0;
                }

                for (std::size_t i = 0;
                    i < bytes;
                    ++i)
                {
                    if (source[read] == 0)
                    {
                        return 0;
                    }

                    dest[written++] =
                        source[read++];
                }

                if (code != 0xFF &&
                    read < length)
                {
                    if (written >= capacity)
                    {
                        return 0;
                    }

                    dest[written++] = 0;
                }
            }

            return written;
        }

        // COBS encoding without stream delimiter.
        std::size_t cobsEncode(
            const std::uint8_t* source,
            std::size_t length,
            std::uint8_t* dest,
            std::size_t capacity) noexcept
        {
            if (!source || !dest || capacity < 2)
            {
                return 0;
            }

            std::size_t codeIndex = 0;
            std::size_t written = 1;

            std::uint8_t code = 1;

            for (std::size_t i = 0;
                i < length;
                ++i)
            {
                if (source[i] == 0)
                {
                    dest[codeIndex] = code;

                    if (written >= capacity)
                    {
                        return 0;
                    }

                    codeIndex = written++;
                    code = 1;
                }
                else
                {
                    if (written >= capacity)
                    {
                        return 0;
                    }

                    dest[written++] = source[i];
                    ++code;

                    if (code == 0xFF)
                    {
                        dest[codeIndex] = code;

                        if (written >= capacity)
                        {
                            return 0;
                        }

                        codeIndex = written++;
                        code = 1;
                    }
                }
            }

            dest[codeIndex] = code;

            return written;
        }
    }

    // CRC-32/ISO-HDLC.
    // Reflected polynomial: 0xEDB88320.
    // Initial value and final XOR: 0xFFFFFFFF.
    std::uint32_t crc32(
        const std::uint8_t* data,
        std::size_t length) noexcept
    {
        if (!data && length != 0)
        {
            return 0;
        }

        std::uint32_t result = 0xFFFFFFFFu;

        for (std::size_t i = 0;
            i < length;
            ++i)
        {
            result ^= data[i];

            for (int bit = 0;
                bit < 8;
                ++bit)
            {
                result =
                    (result >> 1) ^
                    ((result & 1u)
                        ? 0xEDB88320u
                        : 0u);
            }
        }

        return result ^ 0xFFFFFFFFu;
    }

    bool decodeSensorFrame(
        const std::uint8_t* encoded,
        std::size_t encodedLength,
        SensorSample& output) noexcept
    {
        if (!encoded ||
            encodedLength == 0 ||
            encodedLength >= MAX_ENCODED_FRAME_SIZE)
        {
            return false;
        }

        std::uint8_t raw[
            MAX_RAW_PACKET_SIZE]{};

            const std::size_t length =
                cobsDecode(
                    encoded,
                    encodedLength,
                    raw,
                    sizeof(raw));

            if (length !=
                HEADER_SIZE +
                SENSOR_SIZE +
                CRC_SIZE ||
                raw[0] != VERSION ||
                raw[1] != SENSOR_SAMPLE ||
                load16(raw + 2) != SENSOR_SIZE)
            {
                return false;
            }

            if (crc32(
                raw,
                length - CRC_SIZE) !=
                load32(
                    raw + length - CRC_SIZE))
            {
                return false;
            }

            SensorSample candidate{};

            candidate.sequence =
                load32(raw + 4);

            candidate.simulationTimeUs =
                load64(raw + HEADER_SIZE);

            const std::uint8_t* vectorData =
                raw + HEADER_SIZE + 8;

            for (int axis = 0;
                axis < 3;
                ++axis)
            {
                candidate.specificForceB[axis] =
                    loadFloat(
                        vectorData + axis * 4);

                candidate.angularVelocityB[axis] =
                    loadFloat(
                        vectorData + 12 + axis * 4);

                if (!std::isfinite(
                    candidate.specificForceB[axis]) ||
                    !std::isfinite(
                        candidate.angularVelocityB[axis]))
                {
                    return false;
                }
            }

            output = candidate;
            return true;
    }

    std::size_t encodeControlFrame(
        const ControlCommand& command,
        std::uint8_t* output,
        std::size_t capacity) noexcept
    {
        if (!output ||
            capacity < 2 ||
            command.controllerStatus > 2)
        {
            return 0;
        }

        for (float value :
        command.commandedMomentB)
        {
            if (!std::isfinite(value))
            {
                return 0;
            }
        }

        std::uint8_t raw[
            HEADER_SIZE +
                COMMAND_SIZE +
                CRC_SIZE]{};

            raw[0] = VERSION;
            raw[1] = CONTROL_COMMAND;

            store16(
                raw + 2,
                COMMAND_SIZE);

            store32(
                raw + 4,
                command.sequence);

            store32(
                raw + HEADER_SIZE,
                command.referencedSensorSequence);

            for (int axis = 0;
                axis < 3;
                ++axis)
            {
                storeFloat(
                    raw +
                    HEADER_SIZE +
                    4 +
                    axis * 4,
                    command.commandedMomentB[axis]);
            }

            raw[HEADER_SIZE + 16] =
                command.controllerStatus;

            store32(
                raw +
                HEADER_SIZE +
                COMMAND_SIZE,
                crc32(
                    raw,
                    HEADER_SIZE +
                    COMMAND_SIZE));

            const std::size_t count =
                cobsEncode(
                    raw,
                    sizeof(raw),
                    output,
                    capacity - 1);

            if (count == 0)
            {
                return 0;
            }

            output[count] = 0;

            return count + 1;
    }
}
