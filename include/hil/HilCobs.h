#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace aeroforge::hil
{
    bool cobsEncode(std::span<const std::uint8_t> input, std::span<std::uint8_t> output, std::size_t& bytesWritten);

    bool cobsDecode(std::span<const std::uint8_t> input, std::span<std::uint8_t> output, std::size_t& bytesWritten);
}