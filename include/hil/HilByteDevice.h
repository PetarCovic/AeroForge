#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace aeroforge::hil
{
    struct HilIoResult
    {
        bool ok = false;
        std::size_t bytes = 0;
        std::uint32_t error = 0;
    };

    class HilByteDevice
    {
    public:
        virtual ~HilByteDevice() = default;

        virtual bool open() = 0;
        virtual void close() noexcept = 0;

        // A successful read of zero bytes means no data.
        virtual HilIoResult read(
            std::span<std::uint8_t> output) = 0;

        // A successful write may be partial or write zero.
        virtual HilIoResult write(
            std::span<const std::uint8_t> input) = 0;

        virtual std::uint32_t lastError() const noexcept = 0;
    };
}
