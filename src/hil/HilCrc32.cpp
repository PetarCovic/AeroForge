#include <hil/HilCrc32.h>

namespace aeroforge::hil
{
    std::uint32_t calculateCrc32(
        std::span<const std::uint8_t> data)
    {
        std::uint32_t crc = 0xFFFFFFFFu;

        for (const std::uint8_t byte : data)
        {
            crc ^= static_cast<std::uint32_t>(byte);

            for (int bit = 0; bit < 8; ++bit)
            {
                if ((crc & 1u) != 0u)
                {
                    crc = (crc >> 1u) ^ 0xEDB88320u;
                }
                else
                {
                    crc >>= 1u;
                }
            }
        }

        return crc ^ 0xFFFFFFFFu;
    }
}