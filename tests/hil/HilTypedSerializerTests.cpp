#include <gtest/gtest.h>

#include <hil/HilSerializer.h>
#include <hil/HilMessage.h>

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>

using namespace aeroforge;
using namespace aeroforge::hil;

TEST(HilTypedSerializerTests, Vector3ExactBytes)
{
    const math::Vector3 vector(1.0, -2.5, 0.5);

    std::array<std::uint8_t, VECTOR3_WIRE_SIZE> bytes{};
    std::size_t offset = 0;

    ASSERT_TRUE(writeVector3(bytes, offset, vector));

    const std::array<std::uint8_t, VECTOR3_WIRE_SIZE> expected{
        0x00, 0x00, 0x80, 0x3F,
        0x00, 0x00, 0x20, 0xC0,
        0x00, 0x00, 0x00, 0x3F
    };

    EXPECT_EQ(bytes, expected);
    EXPECT_EQ(offset, VECTOR3_WIRE_SIZE);

    math::Vector3 decoded{};
    offset = 0;

    ASSERT_TRUE(readVector3(bytes, offset, decoded));
    EXPECT_DOUBLE_EQ(decoded.x, 1.0);
    EXPECT_DOUBLE_EQ(decoded.y, -2.5);
    EXPECT_DOUBLE_EQ(decoded.z, 0.5);
}

TEST(HilTypedSerializerTests, Vector3FloatRounding)
{
    const math::Vector3 original(
        9.80665, -0.123456789, 1000000.125);

    std::array<std::uint8_t, 12> bytes{};
    std::size_t offset = 0;

    ASSERT_TRUE(writeVector3(bytes, offset, original));

    math::Vector3 decoded{};
    offset = 0;

    ASSERT_TRUE(readVector3(bytes, offset, decoded));

    EXPECT_EQ(decoded.x,
        static_cast<double>(static_cast<float>(original.x)));
    EXPECT_EQ(decoded.y,
        static_cast<double>(static_cast<float>(original.y)));
    EXPECT_EQ(decoded.z,
        static_cast<double>(static_cast<float>(original.z)));
}

TEST(HilTypedSerializerTests, Vector3RejectsNonFiniteWrite)
{
    const double nan =
        std::numeric_limits<double>::quiet_NaN();

    const double infinity =
        std::numeric_limits<double>::infinity();

    for (const math::Vector3 vector : {
        math::Vector3(nan, 0, 0),
            math::Vector3(0, infinity, 0),
            math::Vector3(0, 0, -infinity)
    })
    {
        std::array<std::uint8_t, 12> bytes{};
        bytes.fill(0xA5);

        const auto original = bytes;
        std::size_t offset = 0;

        EXPECT_FALSE(writeVector3(bytes, offset, vector));
        EXPECT_EQ(bytes, original);
        EXPECT_EQ(offset, 0u);
    }
}

TEST(HilTypedSerializerTests, Vector3RejectsFloatOverflow)
{
    const math::Vector3 vector(
        1.0,
        static_cast<double>(
            std::numeric_limits<float>::max()) * 2.0,
        3.0);

    std::array<std::uint8_t, 12> bytes{};
    bytes.fill(0xA5);

    const auto original = bytes;
    std::size_t offset = 0;

    EXPECT_FALSE(writeVector3(bytes, offset, vector));
    EXPECT_EQ(bytes, original);
    EXPECT_EQ(offset, 0u);
}

TEST(HilTypedSerializerTests, Vector3ReadFailurePreservesState)
{
    std::array<std::uint8_t, 12> bytes{};

    std::size_t writeOffset = 0;
    ASSERT_TRUE(writeF32(bytes, writeOffset, 1.0f));
    ASSERT_TRUE(writeF32(bytes, writeOffset, 2.0f));

    // IEEE-754 positive infinity in the third component.
    ASSERT_TRUE(writeU32(bytes, writeOffset, 0x7F800000u));

    math::Vector3 result(7.0, 8.0, 9.0);
    std::size_t offset = 0;

    EXPECT_FALSE(readVector3(bytes, offset, result));

    EXPECT_EQ(offset, 0u);
    EXPECT_DOUBLE_EQ(result.x, 7.0);
    EXPECT_DOUBLE_EQ(result.y, 8.0);
    EXPECT_DOUBLE_EQ(result.z, 9.0);
}

TEST(HilTypedSerializerTests, Vector3RejectsTruncatedBuffers)
{
    std::array<std::uint8_t, 11> bytes{};
    bytes.fill(0xA5);

    const auto original = bytes;
    std::size_t offset = 0;

    EXPECT_FALSE(writeVector3(
        bytes, offset, math::Vector3(1, 2, 3)));

    EXPECT_EQ(bytes, original);
    EXPECT_EQ(offset, 0u);

    math::Vector3 decoded(4, 5, 6);
    EXPECT_FALSE(readVector3(bytes, offset, decoded));

    EXPECT_DOUBLE_EQ(decoded.x, 4.0);
    EXPECT_DOUBLE_EQ(decoded.y, 5.0);
    EXPECT_DOUBLE_EQ(decoded.z, 6.0);
}

TEST(HilTypedSerializerTests, SensorSampleExactLayout)
{
    SensorSampleMessage sample{};
    sample.simulationTimeUs = 0x1122334455667788ULL;
    sample.specificForceB = math::Vector3(1, -2.5, 0.5);
    sample.angularVelocityB = math::Vector3(-1, 2, 0);

    std::array<std::uint8_t, SENSOR_SAMPLE_PAYLOAD_SIZE> bytes{};
    std::size_t offset = 0;

    ASSERT_TRUE(writeSensorSample(bytes, offset, sample));

    const std::array<std::uint8_t, 32> expected{
        0x88, 0x77, 0x66, 0x55,
        0x44, 0x33, 0x22, 0x11,

        0x00, 0x00, 0x80, 0x3F,
        0x00, 0x00, 0x20, 0xC0,
        0x00, 0x00, 0x00, 0x3F,

        0x00, 0x00, 0x80, 0xBF,
        0x00, 0x00, 0x00, 0x40,
        0x00, 0x00, 0x00, 0x00
    };

    EXPECT_EQ(bytes, expected);
    EXPECT_EQ(offset, 32u);
}

TEST(HilTypedSerializerTests, SensorSampleRoundTrip)
{
    SensorSampleMessage original{};
    original.simulationTimeUs = 123456789ULL;
    original.specificForceB =
        math::Vector3(9.80665, -1.25, 0.0);
    original.angularVelocityB =
        math::Vector3(0.01, -0.02, 0.03);

    std::array<std::uint8_t, 32> bytes{};
    std::size_t offset = 0;

    ASSERT_TRUE(writeSensorSample(bytes, offset, original));
    EXPECT_EQ(offset, 32u);

    SensorSampleMessage decoded{};
    offset = 0;

    ASSERT_TRUE(readSensorSample(bytes, offset, decoded));

    EXPECT_EQ(offset, 32u);
    EXPECT_EQ(decoded.simulationTimeUs,
        original.simulationTimeUs);

    EXPECT_EQ(decoded.specificForceB.x,
        static_cast<double>(
            static_cast<float>(original.specificForceB.x)));

    EXPECT_EQ(decoded.specificForceB.y, -1.25);
    EXPECT_EQ(decoded.angularVelocityB.z,
        static_cast<double>(
            static_cast<float>(original.angularVelocityB.z)));
}

TEST(HilTypedSerializerTests, SensorWriteIsTransactional)
{
    SensorSampleMessage sample{};
    sample.simulationTimeUs = 100;
    sample.specificForceB = math::Vector3(1, 2, 3);
    sample.angularVelocityB = math::Vector3(
        4, 5, std::numeric_limits<double>::infinity());

    std::array<std::uint8_t, 40> bytes{};
    bytes.fill(0xA5);

    const auto original = bytes;
    std::size_t offset = 4;

    EXPECT_FALSE(writeSensorSample(bytes, offset, sample));

    EXPECT_EQ(bytes, original);
    EXPECT_EQ(offset, 4u);
}

TEST(HilTypedSerializerTests, SensorReadIsTransactional)
{
    std::array<std::uint8_t, 32> bytes{};

    SensorSampleMessage sample{};
    sample.simulationTimeUs = 55;
    sample.specificForceB = math::Vector3(1, 2, 3);
    sample.angularVelocityB = math::Vector3(4, 5, 6);

    std::size_t writeOffset = 0;
    ASSERT_TRUE(writeSensorSample(bytes, writeOffset, sample));

    // Corrupt the final float into infinity.
    bytes[28] = 0x00;
    bytes[29] = 0x00;
    bytes[30] = 0x80;
    bytes[31] = 0x7F;

    SensorSampleMessage decoded{};
    decoded.simulationTimeUs = 999;
    decoded.specificForceB = math::Vector3(7, 8, 9);

    std::size_t readOffset = 0;
    EXPECT_FALSE(readSensorSample(bytes, readOffset, decoded));

    EXPECT_EQ(readOffset, 0u);
    EXPECT_EQ(decoded.simulationTimeUs, 999u);
    EXPECT_DOUBLE_EQ(decoded.specificForceB.x, 7.0);
}

TEST(HilTypedSerializerTests, SensorNonzeroOffset)
{
    SensorSampleMessage sample{};
    sample.simulationTimeUs = 100;
    sample.specificForceB = math::Vector3(1, 2, 3);
    sample.angularVelocityB = math::Vector3(4, 5, 6);

    std::array<std::uint8_t, 40> bytes{};
    bytes.fill(0xA5);

    std::size_t offset = 4;
    ASSERT_TRUE(writeSensorSample(bytes, offset, sample));

    EXPECT_EQ(offset, 36u);

    for (std::size_t i = 0; i < 4; ++i)
        EXPECT_EQ(bytes[i], 0xA5);

    for (std::size_t i = 36; i < 40; ++i)
        EXPECT_EQ(bytes[i], 0xA5);
}

TEST(HilTypedSerializerTests, ControlCommandExactLayout)
{
    ControlCommandMessage command{};
    command.referencedSensorSequence = 0x12345678u;
    command.commandedMomentB = math::Vector3(1, -2.5, 0);
    command.controllerStatus = 0;

    std::array<std::uint8_t, CONTROL_COMMAND_PAYLOAD_SIZE> bytes{};
    std::size_t offset = 0;

    ASSERT_TRUE(writeControlCommand(bytes, offset, command));

    const std::array<std::uint8_t, 17> expected{
        0x78, 0x56, 0x34, 0x12,
        0x00, 0x00, 0x80, 0x3F,
        0x00, 0x00, 0x20, 0xC0,
        0x00, 0x00, 0x00, 0x00,
        0x00
    };

    EXPECT_EQ(bytes, expected);
    EXPECT_EQ(offset, 17u);
}

TEST(HilTypedSerializerTests, ControlCommandRoundTrip)
{
    ControlCommandMessage original{};
    original.referencedSensorSequence = 7654321;
    original.commandedMomentB =
        math::Vector3(-1.5, 3.25, 0.125);
    original.controllerStatus =
        static_cast<std::uint8_t>(
            ControllerStatus::DEGRADED);

    std::array<std::uint8_t, 17> bytes{};
    std::size_t offset = 0;

    ASSERT_TRUE(writeControlCommand(bytes, offset, original));

    ControlCommandMessage decoded{};
    offset = 0;

    ASSERT_TRUE(readControlCommand(bytes, offset, decoded));

    EXPECT_EQ(offset, 17u);
    EXPECT_EQ(decoded.referencedSensorSequence,
        original.referencedSensorSequence);
    EXPECT_DOUBLE_EQ(decoded.commandedMomentB.x, -1.5);
    EXPECT_DOUBLE_EQ(decoded.commandedMomentB.y, 3.25);
    EXPECT_DOUBLE_EQ(decoded.commandedMomentB.z, 0.125);
    EXPECT_EQ(decoded.controllerStatus,
        original.controllerStatus);
}

TEST(HilTypedSerializerTests, ControlWriteIsTransactional)
{
    ControlCommandMessage command{};
    command.referencedSensorSequence = 42;
    command.commandedMomentB = math::Vector3(
        1.0,
        std::numeric_limits<double>::quiet_NaN(),
        3.0);

    std::array<std::uint8_t, 25> bytes{};
    bytes.fill(0xA5);

    const auto original = bytes;
    std::size_t offset = 3;

    EXPECT_FALSE(writeControlCommand(bytes, offset, command));

    EXPECT_EQ(bytes, original);
    EXPECT_EQ(offset, 3u);
}

TEST(HilTypedSerializerTests, ControlRejectsInsufficientCapacity)
{
    ControlCommandMessage command{};
    command.commandedMomentB = math::Vector3(1, 2, 3);

    std::array<std::uint8_t, 16> bytes{};
    bytes.fill(0xA5);

    const auto original = bytes;
    std::size_t offset = 0;

    EXPECT_FALSE(writeControlCommand(bytes, offset, command));

    EXPECT_EQ(bytes, original);
    EXPECT_EQ(offset, 0u);
}