
#include <hil/HilCobs.h>
#include <hil/HilMessage.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace aeroforge::hil
{
    bool cobsEncode(
        std::span<const std::uint8_t> input,
        std::span<std::uint8_t> output,
        std::size_t& bytesWritten)
    {
        // Enforce the maximum raw HIL packet size.
        if (input.size() > MAX_RAW_PACKET_SIZE)
        {
            return false;
        }

        // For packets of at most 128 bytes, COBS requires
        // exactly one additional byte.
        const std::size_t requiredCapacity = input.size() + 1;

        if (output.size() < requiredCapacity)
        {
            return false;
        }

        // Encode into temporary storage so failures cannot
        // partially modify the caller's output.
        std::array<std::uint8_t, MAX_COBS_ENCODED_SIZE> tmpBuffer{};

        std::size_t readIndex = 0;
        std::size_t writeIndex = 1;
        std::size_t codeIndex = 0;

        std::uint8_t code = 1;

        while (readIndex < input.size())
        {
            const std::uint8_t currentByte = input[readIndex];

            if (currentByte == 0)
            {
                // Finalize the current block.
                tmpBuffer[codeIndex] = code;

                // Reserve space for the next block's code.
                codeIndex = writeIndex;
                ++writeIndex;

                code = 1;
            }
            else
            {
                // Append the nonzero byte.
                tmpBuffer[writeIndex] = currentByte;
                ++writeIndex;
                ++code;

                // A COBS code of 0xFF represents a block
                // containing 254 consecutive nonzero bytes.
                if (code == 0xFF)
                {
                    tmpBuffer[codeIndex] = code;

                    codeIndex = writeIndex;
                    ++writeIndex;

                    code = 1;
                }
            }

            ++readIndex;
        }

        // Finalize the last block.
        tmpBuffer[codeIndex] = code;

        const std::size_t encodedSize = writeIndex;

        // Validate the final encoded size.
        if (encodedSize > MAX_COBS_ENCODED_SIZE ||
            encodedSize > output.size())
        {
            return false;
        }

        // Commit the complete encoded message.
        std::copy_n(
            tmpBuffer.begin(),
            encodedSize,
            output.begin()
        );

        bytesWritten = encodedSize;

        return true;
    }

    bool cobsDecode(
        std::span<const std::uint8_t> input,
        std::span<std::uint8_t> output,
        std::size_t& bytesWritten)
    {
        // A valid COBS encoding contains at least one code byte.
        if (input.empty())
        {
            return false;
        }

        // Reject encoded frames larger than the protocol limit.
        if (input.size() > MAX_COBS_ENCODED_SIZE)
        {
            return false;
        }

        // Use temporary storage to prevent partial output.
        std::array<std::uint8_t, MAX_RAW_PACKET_SIZE> tmpBuffer{};

        std::size_t readIndex = 0;
        std::size_t writeIndex = 0;

        while (readIndex < input.size())
        {
            const std::uint8_t code = input[readIndex];
            ++readIndex;

            // Zero is never a valid COBS code.
            if (code == 0)
            {
                return false;
            }

            // The code specifies how many literal bytes follow.
            const std::size_t literalCount =
                static_cast<std::size_t>(code) - 1;

            // Reject truncated blocks.
            if (literalCount > input.size() - readIndex)
            {
                return false;
            }

            // Enforce the maximum decoded packet size.
            if (writeIndex > MAX_RAW_PACKET_SIZE ||
                literalCount > MAX_RAW_PACKET_SIZE - writeIndex)
            {
                return false;
            }

            // Copy the nonzero literal bytes.
            for (std::size_t i = 0; i < literalCount; ++i)
            {
                const std::uint8_t currentByte =
                    input[readIndex];

                // Encoded COBS data cannot contain zero bytes.
                if (currentByte == 0)
                {
                    return false;
                }

                tmpBuffer[writeIndex] = currentByte;

                ++readIndex;
                ++writeIndex;
            }

            // Insert a zero between blocks unless this was
            // a 0xFF block or the final block.
            if (code != 0xFF && readIndex < input.size())
            {
                if (writeIndex >= MAX_RAW_PACKET_SIZE)
                {
                    return false;
                }

                tmpBuffer[writeIndex] = 0;
                ++writeIndex;
            }
        }

        // Ensure the caller has sufficient output capacity.
        if (output.size() < writeIndex)
        {
            return false;
        }

        // Commit only after the entire encoded input is valid.
        std::copy_n(
            tmpBuffer.begin(),
            writeIndex,
            output.begin()
        );

        bytesWritten = writeIndex;

        return true;
    }
}