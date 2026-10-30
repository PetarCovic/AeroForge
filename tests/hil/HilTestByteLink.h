#pragma once

#include <hil/HilWireEndpoint.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <span>

namespace aeroforge::hil::tests
{
    struct HilTestByteLink
    {
        std::deque<std::uint8_t> aToB;
        std::deque<std::uint8_t> bToA;

        std::size_t maxReadChunk =
            std::numeric_limits<std::size_t>::max();

        std::size_t maxWriteChunk =
            std::numeric_limits<std::size_t>::max();

        bool blockAWrites = false;
        bool blockBWrites = false;

        static std::size_t read(
            std::deque<std::uint8_t>& queue,
            std::span<std::uint8_t> output,
            std::size_t limit)
        {
            const std::size_t count =
                std::min({ queue.size(), output.size(), limit });

            for (std::size_t i = 0; i < count; ++i)
            {
                output[i] = queue.front();
                queue.pop_front();
            }

            return count;
        }

        static std::size_t write(
            std::deque<std::uint8_t>& queue,
            std::span<const std::uint8_t> input,
            std::size_t limit,
            bool blocked)
        {
            if (blocked)
                return 0;

            const std::size_t count =
                std::min(input.size(), limit);

            for (std::size_t i = 0; i < count; ++i)
                queue.push_back(input[i]);

            return count;
        }

        HilBytePort portA()
        {
            return {
                [this](std::span<std::uint8_t> output)
                {
                    return read(
                        bToA, output, maxReadChunk);
                },
                [this](std::span<const std::uint8_t> input)
                {
                    return write(
                        aToB, input, maxWriteChunk,
                        blockAWrites);
                }
            };
        }

        HilBytePort portB()
        {
            return {
                [this](std::span<std::uint8_t> output)
                {
                    return read(
                        aToB, output, maxReadChunk);
                },
                [this](std::span<const std::uint8_t> input)
                {
                    return write(
                        bToA, input, maxWriteChunk,
                        blockBWrites);
                }
            };
        }
    };
}