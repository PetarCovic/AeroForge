#pragma once

#include <cstdint>
#include <span>

namespace aeroforge::hil
{
    std::uint32_t calculateCrc32(std::span<const std::uint8_t> data);
}