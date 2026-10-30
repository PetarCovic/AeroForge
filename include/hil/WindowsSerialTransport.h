#pragma once

#include <hil/HilByteDevice.h>

#include <cstdint>
#include <string>

namespace aeroforge::hil
{
    struct WindowsSerialConfig
    {
        std::wstring portName = L"COM3";
        std::uint32_t baudRate = 115200;
        std::uint32_t writeTimeoutMs = 25;
    };

    class WindowsSerialTransport final : public HilByteDevice
    {
    public:
        explicit WindowsSerialTransport(
            WindowsSerialConfig config);

        ~WindowsSerialTransport() override;

        WindowsSerialTransport(
            const WindowsSerialTransport&) = delete;

        WindowsSerialTransport& operator=(
            const WindowsSerialTransport&) = delete;

        bool open() override;
        void close() noexcept override;

        HilIoResult read(
            std::span<std::uint8_t> output) override;

        HilIoResult write(
            std::span<const std::uint8_t> input) override;

        std::uint32_t lastError() const noexcept override;

        bool isOpen() const noexcept;

    private:
        WindowsSerialConfig config_;

        // Windows HANDLE is represented as a pointer.
        // Keep Windows.h out of this public header.
        void* handle_ = nullptr;

        std::uint32_t lastError_ = 0;
    };
}
