#include <gtest/gtest.h>

#include <hil/WindowsSerialTransport.h>

#include <array>
#include <cstdint>

using namespace aeroforge::hil;

TEST(WindowsSerialTransportTests, StartsClosed)
{
    WindowsSerialConfig config{};
    WindowsSerialTransport serial(config);

    EXPECT_FALSE(serial.isOpen());
}

TEST(WindowsSerialTransportTests, RejectsEmptyPortName)
{
    WindowsSerialConfig config{};
    config.portName = L"";

    WindowsSerialTransport serial(config);

    EXPECT_FALSE(serial.open());
    EXPECT_FALSE(serial.isOpen());
    EXPECT_NE(serial.lastError(), 0u);
}

TEST(WindowsSerialTransportTests, RejectsZeroBaudRate)
{
    WindowsSerialConfig config{};
    config.baudRate = 0;

    WindowsSerialTransport serial(config);

    EXPECT_FALSE(serial.open());
    EXPECT_FALSE(serial.isOpen());
}

TEST(WindowsSerialTransportTests, RejectsZeroWriteTimeout)
{
    WindowsSerialConfig config{};
    config.writeTimeoutMs = 0;

    WindowsSerialTransport serial(config);

    EXPECT_FALSE(serial.open());
    EXPECT_FALSE(serial.isOpen());
}

TEST(WindowsSerialTransportTests, ReadFailsWhenClosed)
{
    WindowsSerialTransport serial({});

    std::array<std::uint8_t, 16> buffer{};

    const HilIoResult result = serial.read(buffer);

    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.bytes, 0u);
    EXPECT_NE(result.error, 0u);
}

TEST(WindowsSerialTransportTests, WriteFailsWhenClosed)
{
    WindowsSerialTransport serial({});

    const std::array<std::uint8_t, 3> bytes{
        0x11, 0x22, 0x33
    };

    const HilIoResult result = serial.write(bytes);

    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.bytes, 0u);
    EXPECT_NE(result.error, 0u);
}

TEST(WindowsSerialTransportTests, CloseIsIdempotent)
{
    WindowsSerialTransport serial({});

    serial.close();
    serial.close();

    EXPECT_FALSE(serial.isOpen());
}
