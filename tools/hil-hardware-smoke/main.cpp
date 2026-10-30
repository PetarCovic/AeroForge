#include <Windows.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

namespace
{
    using Clock = std::chrono::steady_clock;
    using Byte = std::uint8_t;
    using Bytes = std::vector<Byte>;

    constexpr Byte PROTOCOL_VERSION = 1;
    constexpr Byte SENSOR_TYPE = 2;
    constexpr Byte CONTROL_TYPE = 3;

    constexpr std::size_t HEADER_SIZE = 8;
    constexpr std::size_t SENSOR_PAYLOAD_SIZE = 32;
    constexpr std::size_t CONTROL_PAYLOAD_SIZE = 17;
    constexpr std::size_t CRC_SIZE = 4;

    constexpr DWORD BAUD_RATE = 115200;

    constexpr auto RESPONSE_TIMEOUT =
        std::chrono::milliseconds(500);

    constexpr std::array<float, 3> TEST_RATES{
        0.3f, -0.2f, 0.4f
    };

    constexpr std::array<float, 3> EXPECTED_MOMENTS{
        -1.5f, 1.0f, -2.0f
    };

    void require(bool condition, const std::string& message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    void write16(Bytes& data, std::size_t offset,
        std::uint16_t value)
    {
        data[offset] = static_cast<Byte>(value);
        data[offset + 1] =
            static_cast<Byte>(value >> 8);
    }

    void write32(Bytes& data, std::size_t offset,
        std::uint32_t value)
    {
        for (unsigned i = 0; i < 4; ++i)
        {
            data[offset + i] =
                static_cast<Byte>(value >> (8 * i));
        }
    }

    void write64(Bytes& data, std::size_t offset,
        std::uint64_t value)
    {
        for (unsigned i = 0; i < 8; ++i)
        {
            data[offset + i] =
                static_cast<Byte>(value >> (8 * i));
        }
    }

    void writeFloat(Bytes& data, std::size_t offset,
        float value)
    {
        static_assert(sizeof(float) == 4);
        static_assert(
            std::numeric_limits<float>::is_iec559);

        std::uint32_t bits = 0;
        std::memcpy(&bits, &value, sizeof(bits));
        write32(data, offset, bits);
    }

    std::uint16_t read16(const Bytes& data,
        std::size_t offset)
    {
        return static_cast<std::uint16_t>(
            static_cast<std::uint16_t>(data[offset]) |
            (static_cast<std::uint16_t>(
                data[offset + 1]) << 8));
    }

    std::uint32_t read32(const Bytes& data,
        std::size_t offset)
    {
        std::uint32_t value = 0;

        for (unsigned i = 0; i < 4; ++i)
        {
            value |=
                static_cast<std::uint32_t>(
                    data[offset + i]) << (8 * i);
        }

        return value;
    }

    float readFloat(const Bytes& data, std::size_t offset)
    {
        const std::uint32_t bits = read32(data, offset);
        float value = 0.0f;
        std::memcpy(&value, &bits, sizeof(value));
        return value;
    }

    std::uint32_t crc32(const Byte* data,
        std::size_t length)
    {
        std::uint32_t crc = 0xFFFFFFFFu;

        for (std::size_t i = 0; i < length; ++i)
        {
            crc ^= data[i];

            for (int bit = 0; bit < 8; ++bit)
            {
                crc = (crc >> 1) ^
                    ((crc & 1u) != 0
                        ? 0xEDB88320u
                        : 0u);
            }
        }

        return crc ^ 0xFFFFFFFFu;
    }

    Bytes cobsEncode(const Bytes& raw)
    {
        Bytes encoded;
        encoded.reserve(raw.size() + 3);

        std::size_t codeIndex = 0;
        encoded.push_back(0);

        Byte code = 1;

        for (Byte value : raw)
        {
            if (value == 0)
            {
                encoded[codeIndex] = code;
                codeIndex = encoded.size();
                encoded.push_back(0);
                code = 1;
            }
            else
            {
                encoded.push_back(value);
                ++code;

                if (code == 0xFF)
                {
                    encoded[codeIndex] = code;
                    codeIndex = encoded.size();
                    encoded.push_back(0);
                    code = 1;
                }
            }
        }

        encoded[codeIndex] = code;
        encoded.push_back(0);

        return encoded;
    }

    std::optional<Bytes> cobsDecode(
        const Bytes& encoded)
    {
        if (encoded.empty())
        {
            return std::nullopt;
        }

        Bytes raw;
        raw.reserve(encoded.size());

        std::size_t index = 0;

        while (index < encoded.size())
        {
            const Byte code = encoded[index++];

            if (code == 0)
            {
                return std::nullopt;
            }

            const std::size_t count =
                static_cast<std::size_t>(code - 1);

            if (count > encoded.size() - index)
            {
                return std::nullopt;
            }

            for (std::size_t i = 0; i < count; ++i)
            {
                if (encoded[index] == 0)
                {
                    return std::nullopt;
                }

                raw.push_back(encoded[index++]);
            }

            if (code != 0xFF &&
                index < encoded.size())
            {
                raw.push_back(0);
            }
        }

        return raw;
    }

    Bytes makeSensorFrame(std::uint32_t sequence)
    {
        Bytes raw(
            HEADER_SIZE +
            SENSOR_PAYLOAD_SIZE +
            CRC_SIZE, 0);

        raw[0] = PROTOCOL_VERSION;
        raw[1] = SENSOR_TYPE;

        write16(
            raw,
            2,
            static_cast<std::uint16_t>(
                SENSOR_PAYLOAD_SIZE));

        write32(raw, 4, sequence);

        // One simulated 10 ms step per sequence.
        write64(
            raw,
            HEADER_SIZE,
            static_cast<std::uint64_t>(
                sequence) * 10000u);

        // Specific force: zero for this protocol test.
        for (int axis = 0; axis < 3; ++axis)
        {
            writeFloat(
                raw,
                HEADER_SIZE + 8 +
                static_cast<std::size_t>(axis) * 4,
                0.0f);
        }

        // Body angular velocity in rad/s.
        for (int axis = 0; axis < 3; ++axis)
        {
            writeFloat(
                raw,
                HEADER_SIZE + 20 +
                static_cast<std::size_t>(axis) * 4,
                TEST_RATES[axis]);
        }

        write32(
            raw,
            raw.size() - CRC_SIZE,
            crc32(
                raw.data(),
                raw.size() - CRC_SIZE));

        return cobsEncode(raw);
    }

    struct ControlResponse
    {
        std::uint32_t sequence = 0;
        std::uint32_t referencedSensorSequence = 0;
        std::array<float, 3> moments{};
        Byte status = 0;
    };

    std::optional<ControlResponse> decodeControl(
        const Bytes& encoded)
    {
        const auto decoded = cobsDecode(encoded);

        if (!decoded.has_value())
        {
            return std::nullopt;
        }

        const Bytes& raw = *decoded;

        if (raw.size() !=
            HEADER_SIZE +
            CONTROL_PAYLOAD_SIZE +
            CRC_SIZE)
        {
            return std::nullopt;
        }

        if (raw[0] != PROTOCOL_VERSION ||
            raw[1] != CONTROL_TYPE ||
            read16(raw, 2) != CONTROL_PAYLOAD_SIZE)
        {
            return std::nullopt;
        }

        const std::uint32_t expectedCrc =
            crc32(
                raw.data(),
                raw.size() - CRC_SIZE);

        const std::uint32_t receivedCrc =
            read32(raw, raw.size() - CRC_SIZE);

        if (expectedCrc != receivedCrc)
        {
            return std::nullopt;
        }

        ControlResponse result{};

        result.sequence = read32(raw, 4);
        result.referencedSensorSequence =
            read32(raw, HEADER_SIZE);

        for (int axis = 0; axis < 3; ++axis)
        {
            result.moments[axis] =
                readFloat(
                    raw,
                    HEADER_SIZE + 4 +
                    static_cast<std::size_t>(axis) * 4);

            if (!std::isfinite(result.moments[axis]))
            {
                return std::nullopt;
            }
        }

        result.status =
            raw[HEADER_SIZE + 16];

        return result;
    }

    class SerialPort final
    {
    public:
        explicit SerialPort(const std::string& port)
        {
            const std::string path =
                "\\\\.\\" + port;

            handle_ = CreateFileA(
                path.c_str(),
                GENERIC_READ | GENERIC_WRITE,
                0,
                nullptr,
                OPEN_EXISTING,
                FILE_ATTRIBUTE_NORMAL,
                nullptr);

            if (handle_ == INVALID_HANDLE_VALUE)
            {
                throwWindowsError(
                    "Cannot open " + port);
            }

            try
            {
                configure();
            }
            catch (...)
            {
                CloseHandle(handle_);
                handle_ = INVALID_HANDLE_VALUE;
                throw;
            }
        }

        ~SerialPort()
        {
            if (handle_ != INVALID_HANDLE_VALUE)
            {
                CloseHandle(handle_);
            }
        }

        SerialPort(const SerialPort&) = delete;
        SerialPort& operator=(const SerialPort&) = delete;

        void write(const Bytes& frame)
        {
            std::size_t offset = 0;

            while (offset < frame.size())
            {
                DWORD written = 0;

                const DWORD remaining =
                    static_cast<DWORD>(
                        frame.size() - offset);

                if (!WriteFile(
                    handle_,
                    frame.data() + offset,
                    remaining,
                    &written,
                    nullptr))
                {
                    throwWindowsError("WriteFile");
                }

                require(
                    written != 0,
                    "Serial write made no progress");

                offset += written;
            }
        }

        std::optional<Bytes> receiveFrame(
            Clock::time_point deadline)
        {
            Bytes frame;
            frame.reserve(128);

            bool overflow = false;

            while (Clock::now() < deadline)
            {
                Byte value = 0;
                DWORD received = 0;

                if (!ReadFile(
                    handle_,
                    &value,
                    1,
                    &received,
                    nullptr))
                {
                    throwWindowsError("ReadFile");
                }

                if (received == 0)
                {
                    continue;
                }

                if (value == 0)
                {
                    if (!overflow && !frame.empty())
                    {
                        return frame;
                    }

                    frame.clear();
                    overflow = false;
                    continue;
                }

                if (overflow)
                {
                    continue;
                }

                if (frame.size() >= 128)
                {
                    frame.clear();
                    overflow = true;
                    continue;
                }

                frame.push_back(value);
            }

            return std::nullopt;
        }

    private:
        static void throwWindowsError(
            const std::string& operation)
        {
            throw std::system_error(
                static_cast<int>(GetLastError()),
                std::system_category(),
                operation);
        }

        void configure()
        {
            require(
                SetupComm(handle_, 4096, 4096) != 0,
                "SetupComm failed");

            DCB dcb{};
            dcb.DCBlength = sizeof(dcb);

            if (!GetCommState(handle_, &dcb))
            {
                throwWindowsError("GetCommState");
            }

            dcb.BaudRate = BAUD_RATE;
            dcb.ByteSize = 8;
            dcb.Parity = NOPARITY;
            dcb.StopBits = ONESTOPBIT;

            dcb.fBinary = TRUE;
            dcb.fParity = FALSE;

            dcb.fOutxCtsFlow = FALSE;
            dcb.fOutxDsrFlow = FALSE;

            dcb.fDtrControl = DTR_CONTROL_DISABLE;
            dcb.fRtsControl = RTS_CONTROL_DISABLE;

            dcb.fOutX = FALSE;
            dcb.fInX = FALSE;

            if (!SetCommState(handle_, &dcb))
            {
                throwWindowsError("SetCommState");
            }

            COMMTIMEOUTS timeouts{};

            timeouts.ReadIntervalTimeout = 1;
            timeouts.ReadTotalTimeoutMultiplier = 0;
            timeouts.ReadTotalTimeoutConstant = 20;

            timeouts.WriteTotalTimeoutMultiplier = 0;
            timeouts.WriteTotalTimeoutConstant = 200;

            if (!SetCommTimeouts(handle_, &timeouts))
            {
                throwWindowsError("SetCommTimeouts");
            }

            // A CH340-connected ESP32 can reset when
            // the serial port is opened. Allow firmware
            // startup to finish before clearing stale
            // boot output.
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1200));

            if (!PurgeComm(
                handle_,
                PURGE_RXCLEAR | PURGE_TXCLEAR))
            {
                throwWindowsError("PurgeComm");
            }
        }

        HANDLE handle_ = INVALID_HANDLE_VALUE;
    };

    void validateCrc()
    {
        constexpr std::array<Byte, 9> check{
            '1', '2', '3', '4', '5',
            '6', '7', '8', '9'
        };

        require(
            crc32(check.data(), check.size()) ==
            0xCBF43926u,
            "CRC-32 self-test failed");
    }
}

int main(int argc, char** argv)
{
    try
    {
        validateCrc();

        const std::string port =
            argc >= 2 ? argv[1] : "COM3";

        const unsigned long requested =
            argc >= 3
            ? std::stoul(argv[2])
            : 10ul;

        require(
            requested >= 1 && requested <= 1000,
            "Request count must be between 1 and 1000");

        const auto attempts =
            static_cast<std::uint32_t>(requested);

        std::cout
            << "AeroForge HIL hardware smoke test\n"
            << "Port: " << port << '\n'
            << "Baud: " << BAUD_RATE << '\n'
            << "Requested exchanges: " << attempts
            << "\n\n";

        SerialPort serial(port);

        std::uint32_t accepted = 0;
        double totalLatencyMs = 0.0;
        double maxLatencyMs = 0.0;

        for (std::uint32_t sequence = 1;
            sequence <= attempts;
            ++sequence)
        {
            const Bytes request =
                makeSensorFrame(sequence);

            const auto start = Clock::now();

            serial.write(request);

            const auto deadline =
                start + RESPONSE_TIMEOUT;

            bool matched = false;

            while (Clock::now() < deadline)
            {
                const auto frame =
                    serial.receiveFrame(deadline);

                if (!frame.has_value())
                {
                    break;
                }

                const auto response =
                    decodeControl(*frame);

                if (!response.has_value())
                {
                    // Ignore boot text, malformed frames,
                    // and unrelated serial traffic.
                    continue;
                }

                if (response->referencedSensorSequence
                    != sequence)
                {
                    continue;
                }

                const auto end = Clock::now();

                const double latencyMs =
                    std::chrono::duration<
                    double, std::milli>(
                        end - start).count();

                require(
                    response->status == 0,
                    "Controller returned nonzero status");

                for (int axis = 0; axis < 3; ++axis)
                {
                    require(
                        std::abs(
                            response->moments[axis] -
                            EXPECTED_MOMENTS[axis]) <
                        0.0001f,
                        "Controller moment mismatch");
                }

                ++accepted;
                totalLatencyMs += latencyMs;

                if (latencyMs > maxLatencyMs)
                {
                    maxLatencyMs = latencyMs;
                }

                std::cout
                    << "Sensor " << sequence
                    << " -> command "
                    << response->sequence
                    << " | latency "
                    << std::fixed
                    << std::setprecision(3)
                    << latencyMs << " ms"
                    << " | PASS\n";

                matched = true;
                break;
            }

            if (!matched)
            {
                std::cout
                    << "Sensor " << sequence
                    << " -> no matching valid response"
                    << " | FAIL\n";
            }

            std::this_thread::sleep_for(
                std::chrono::milliseconds(20));
        }

        std::cout
            << "\nResults\n"
            << "Accepted exchanges: "
            << accepted << '/' << attempts << '\n';

        if (accepted != 0)
        {
            std::cout
                << "Mean round-trip: "
                << totalLatencyMs /
                static_cast<double>(accepted)
                << " ms\n"
                << "Maximum round-trip: "
                << maxLatencyMs
                << " ms\n";
        }

        if (accepted != attempts)
        {
            std::cerr
                << "Hardware smoke test FAILED\n";
            return 1;
        }

        std::cout
            << "Hardware smoke test PASSED\n";

        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr
            << "Hardware smoke test error: "
            << error.what() << '\n';

        return 1;
    }
}