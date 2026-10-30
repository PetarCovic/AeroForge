#include <hil/WindowsSerialTransport.h>

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <Windows.h>

#include <algorithm>
#include <limits>
#include <utility>

namespace aeroforge::hil
{
    WindowsSerialTransport::WindowsSerialTransport(
        WindowsSerialConfig config)
        : config_(std::move(config))
    {
    }

    WindowsSerialTransport::~WindowsSerialTransport()
    {
        close();
    }

    bool WindowsSerialTransport::open()
    {
        close();
        lastError_ = 0;

        if (config_.portName.empty() ||
            config_.baudRate == 0 ||
            config_.writeTimeoutMs == 0)
        {
            lastError_ = ERROR_INVALID_PARAMETER;
            return false;
        }

        std::wstring devicePath = config_.portName;

        if (devicePath.rfind(L"\\\\.\\", 0) != 0)
        {
            devicePath = L"\\\\.\\" + devicePath;
        }

        HANDLE handle = CreateFileW(
            devicePath.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            nullptr,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            nullptr);

        if (handle == INVALID_HANDLE_VALUE)
        {
            lastError_ = GetLastError();
            return false;
        }

        handle_ = handle;

        DCB dcb{};
        dcb.DCBlength = sizeof(DCB);

        if (!GetCommState(handle, &dcb))
        {
            lastError_ = GetLastError();
            close();
            return false;
        }

        dcb.BaudRate = config_.baudRate;
        dcb.ByteSize = 8;
        dcb.Parity = NOPARITY;
        dcb.StopBits = ONESTOPBIT;

        dcb.fBinary = TRUE;
        dcb.fParity = FALSE;

        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDsrSensitivity = FALSE;

        dcb.fOutX = FALSE;
        dcb.fInX = FALSE;

        // Avoid automatically toggling reset/control
        // lines on the ESP-32S during initialization.
        dcb.fDtrControl = DTR_CONTROL_DISABLE;
        dcb.fRtsControl = RTS_CONTROL_DISABLE;

        if (!SetCommState(handle, &dcb))
        {
            lastError_ = GetLastError();
            close();
            return false;
        }

        COMMTIMEOUTS timeouts{};

        // Return immediately with currently buffered
        // input bytes, including zero bytes if none.
        timeouts.ReadIntervalTimeout = MAXDWORD;
        timeouts.ReadTotalTimeoutMultiplier = 0;
        timeouts.ReadTotalTimeoutConstant = 0;

        timeouts.WriteTotalTimeoutMultiplier = 0;
        timeouts.WriteTotalTimeoutConstant =
            config_.writeTimeoutMs;

        if (!SetCommTimeouts(handle, &timeouts))
        {
            lastError_ = GetLastError();
            close();
            return false;
        }

        // Start a deliberately clean transport session.
        // Only do this during opening, never while active.
        if (!PurgeComm(
            handle,
            PURGE_RXCLEAR | PURGE_TXCLEAR))
        {
            lastError_ = GetLastError();
            close();
            return false;
        }

        lastError_ = 0;
        return true;
    }

    void WindowsSerialTransport::close() noexcept
    {
        if (handle_ != nullptr)
        {
            CloseHandle(static_cast<HANDLE>(handle_));
            handle_ = nullptr;
        }
    }

    HilIoResult WindowsSerialTransport::read(
        std::span<std::uint8_t> output)
    {
        if (!isOpen())
        {
            return { false, 0, ERROR_INVALID_HANDLE };
        }

        if (output.empty())
        {
            return { true, 0, 0 };
        }

        const DWORD requested = static_cast<DWORD>(
            std::min<std::size_t>(
                output.size(),
                std::numeric_limits<DWORD>::max()));

        DWORD received = 0;

        if (!ReadFile(
            static_cast<HANDLE>(handle_),
            output.data(),
            requested,
            &received,
            nullptr))
        {
            lastError_ = GetLastError();
            return { false, 0, lastError_ };
        }

        lastError_ = 0;
        return { true, received, 0 };
    }

    HilIoResult WindowsSerialTransport::write(
        std::span<const std::uint8_t> input)
    {
        if (!isOpen())
        {
            return { false, 0, ERROR_INVALID_HANDLE };
        }

        if (input.empty())
        {
            return { true, 0, 0 };
        }

        const DWORD requested = static_cast<DWORD>(
            std::min<std::size_t>(
                input.size(),
                std::numeric_limits<DWORD>::max()));

        DWORD written = 0;

        if (!WriteFile(
            static_cast<HANDLE>(handle_),
            input.data(),
            requested,
            &written,
            nullptr))
        {
            lastError_ = GetLastError();
            return { false, 0, lastError_ };
        }

        lastError_ = 0;
        return { true, written, 0 };
    }

    std::uint32_t
        WindowsSerialTransport::lastError() const noexcept
    {
        return lastError_;
    }

    bool WindowsSerialTransport::isOpen() const noexcept
    {
        return handle_ != nullptr;
    }
}
