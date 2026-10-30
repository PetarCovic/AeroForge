#include <gtest/gtest.h>

#include <hil/HilPacketCodec.h>
#include <hil/HilCrc32.h>
#include <hil/HilSerializer.h>
#include <hil/HilCobs.h>
#include <limits>

#include <array>
#include <cstdint>
#include <span>
#include <variant>

using namespace aeroforge;
using namespace aeroforge::hil;

namespace
{
    SensorSampleMessage makeSensor()
    {
        SensorSampleMessage sample{};
        sample.simulationTimeUs = 123456;
        sample.specificForceB =
            math::Vector3(1.0, -2.5, 0.5);
        sample.angularVelocityB =
            math::Vector3(0.25, -0.5, 1.0);
        return sample;
    }

    ControlCommandMessage makeControl()
    {
        ControlCommandMessage command{};
        command.referencedSensorSequence = 42;
        command.commandedMomentB =
            math::Vector3(-1.5, 2.0, 0.25);
        command.controllerStatus =
            static_cast<std::uint8_t>(
                ControllerStatus::OK);
        return command;
    }

    template <std::size_t N>
    void rewriteCrc(std::array<std::uint8_t, N>& bytes)
    {
        const auto crc = calculateCrc32(
            std::span<const std::uint8_t>(
                bytes.data(), N - CRC_SIZE));

        std::size_t offset = N - CRC_SIZE;
        ASSERT_TRUE(writeU32(bytes, offset, crc));
    }
}

TEST(HilPacketCodecTests, SensorPacketRoundTrip)
{
    const auto sample = makeSensor();

    std::array<std::uint8_t, SENSOR_PACKET_SIZE> raw{};
    std::size_t written = 0;

    ASSERT_TRUE(encodeSensorPacket(
        sample, 42, raw, written));

    EXPECT_EQ(written, SENSOR_PACKET_SIZE);

    HilPacket decoded{};
    ASSERT_TRUE(decodePacket(raw, decoded));

    EXPECT_EQ(decoded.header.version, PROTOCOL_VERSION);
    EXPECT_EQ(decoded.header.sequence, 42u);
    EXPECT_EQ(decoded.header.payloadLength, 32u);
    EXPECT_EQ(decoded.header.messageType,
        static_cast<std::uint8_t>(
            MessageType::SENSOR_SAMPLE));

    const auto* payload =
        std::get_if<SensorSampleMessage>(&decoded.payload);

    ASSERT_NE(payload, nullptr);

    EXPECT_EQ(payload->simulationTimeUs, 123456u);
    EXPECT_DOUBLE_EQ(payload->specificForceB.x, 1.0);
    EXPECT_DOUBLE_EQ(payload->specificForceB.y, -2.5);
    EXPECT_DOUBLE_EQ(payload->angularVelocityB.z, 1.0);
}

TEST(HilPacketCodecTests, ControlPacketRoundTrip)
{
    const auto command = makeControl();

    std::array<std::uint8_t, CONTROL_PACKET_SIZE> raw{};
    std::size_t written = 0;

    ASSERT_TRUE(encodeControlPacket(
        command, 100, raw, written));

    EXPECT_EQ(written, CONTROL_PACKET_SIZE);

    HilPacket decoded{};
    ASSERT_TRUE(decodePacket(raw, decoded));

    EXPECT_EQ(decoded.header.sequence, 100u);
    EXPECT_EQ(decoded.header.payloadLength, 17u);

    const auto* payload =
        std::get_if<ControlCommandMessage>(
            &decoded.payload);

    ASSERT_NE(payload, nullptr);

    EXPECT_EQ(payload->referencedSensorSequence, 42u);
    EXPECT_DOUBLE_EQ(payload->commandedMomentB.x, -1.5);
    EXPECT_DOUBLE_EQ(payload->commandedMomentB.y, 2.0);
    EXPECT_DOUBLE_EQ(payload->commandedMomentB.z, 0.25);
    EXPECT_EQ(payload->controllerStatus, 0);
}

TEST(HilPacketCodecTests, SensorHeaderExactBytes)
{
    std::array<std::uint8_t, SENSOR_PACKET_SIZE> raw{};
    std::size_t written = 0;

    ASSERT_TRUE(encodeSensorPacket(
        makeSensor(), 0x12345678u, raw, written));

    const std::array<std::uint8_t, 8> expected{
        0x01, 0x02, 0x20, 0x00,
        0x78, 0x56, 0x34, 0x12
    };

    for (std::size_t i = 0; i < expected.size(); ++i)
        EXPECT_EQ(raw[i], expected[i]);
}

TEST(HilPacketCodecTests, StoredCrcMatchesCalculatedCrc)
{
    std::array<std::uint8_t, SENSOR_PACKET_SIZE> raw{};
    std::size_t written = 0;

    ASSERT_TRUE(encodeSensorPacket(
        makeSensor(), 12, raw, written));

    std::size_t offset = written - CRC_SIZE;
    std::uint32_t storedCrc = 0;

    ASSERT_TRUE(readU32(raw, offset, storedCrc));

    const auto calculated = calculateCrc32(
        std::span<const std::uint8_t>(
            raw.data(), written - CRC_SIZE));

    EXPECT_EQ(storedCrc, calculated);
}

TEST(HilPacketCodecTests, RejectsCorruptedPayload)
{
    std::array<std::uint8_t, SENSOR_PACKET_SIZE> raw{};
    std::size_t written = 0;

    ASSERT_TRUE(encodeSensorPacket(
        makeSensor(), 42, raw, written));

    raw[20] ^= 0x01;

    HilPacket decoded{};

    EXPECT_FALSE(decodePacket(raw, decoded));
}

TEST(HilPacketCodecTests, RejectsCorruptedChecksum)
{
    std::array<std::uint8_t, CONTROL_PACKET_SIZE> raw{};
    std::size_t written = 0;

    ASSERT_TRUE(encodeControlPacket(
        makeControl(), 42, raw, written));

    raw[written - 1] ^= 0x80;

    HilPacket decoded{};

    EXPECT_FALSE(decodePacket(raw, decoded));
}

TEST(HilPacketCodecTests, RejectsTruncatedPacket)
{
    std::array<std::uint8_t, SENSOR_PACKET_SIZE> raw{};
    std::size_t written = 0;

    ASSERT_TRUE(encodeSensorPacket(
        makeSensor(), 42, raw, written));

    HilPacket decoded{};

    EXPECT_FALSE(decodePacket(
        std::span<const std::uint8_t>(
            raw.data(), written - 1),
        decoded));
}

TEST(HilPacketCodecTests, RejectsAppendedBytes)
{
    std::array<std::uint8_t, SENSOR_PACKET_SIZE + 1> raw{};
    std::size_t written = 0;

    ASSERT_TRUE(encodeSensorPacket(
        makeSensor(), 42, raw, written));

    raw[written] = 0xFF;

    HilPacket decoded{};

    EXPECT_FALSE(decodePacket(raw, decoded));
}

TEST(HilPacketCodecTests, RejectsWrongPayloadLength)
{
    std::array<std::uint8_t, SENSOR_PACKET_SIZE> raw{};
    std::size_t written = 0;

    ASSERT_TRUE(encodeSensorPacket(
        makeSensor(), 42, raw, written));

    // Declare 31 bytes instead of 32.
    raw[2] = 31;
    raw[3] = 0;

    rewriteCrc(raw);

    HilPacket decoded{};
    EXPECT_FALSE(decodePacket(raw, decoded));
}

TEST(HilPacketCodecTests, RejectsUnsupportedType)
{
    std::array<std::uint8_t, SENSOR_PACKET_SIZE> raw{};
    std::size_t written = 0;

    ASSERT_TRUE(encodeSensorPacket(
        makeSensor(), 42, raw, written));

    raw[1] = static_cast<std::uint8_t>(
        MessageType::HELLO);

    rewriteCrc(raw);

    HilPacket decoded{};
    EXPECT_FALSE(decodePacket(raw, decoded));
}

TEST(HilPacketCodecTests, RejectsUnsupportedVersion)
{
    std::array<std::uint8_t, SENSOR_PACKET_SIZE> raw{};
    std::size_t written = 0;

    ASSERT_TRUE(encodeSensorPacket(
        makeSensor(), 42, raw, written));

    raw[0] = PROTOCOL_VERSION + 1;
    rewriteCrc(raw);

    HilPacket decoded{};
    EXPECT_FALSE(decodePacket(raw, decoded));
}

TEST(HilPacketCodecTests, RejectsNonFinitePayload)
{
    std::array<std::uint8_t, SENSOR_PACKET_SIZE> raw{};
    std::size_t written = 0;

    ASSERT_TRUE(encodeSensorPacket(
        makeSensor(), 42, raw, written));

    // First specific-force component starts at byte 16.
    // Replace it with IEEE-754 positive infinity.
    raw[16] = 0x00;
    raw[17] = 0x00;
    raw[18] = 0x80;
    raw[19] = 0x7F;

    rewriteCrc(raw);

    HilPacket decoded{};
    decoded.header.sequence = 999;

    EXPECT_FALSE(decodePacket(raw, decoded));
    EXPECT_EQ(decoded.header.sequence, 999u);
}

TEST(HilPacketCodecTests, RejectsInvalidControllerStatus)
{
    std::array<std::uint8_t, CONTROL_PACKET_SIZE> raw{};
    std::size_t written = 0;

    ASSERT_TRUE(encodeControlPacket(
        makeControl(), 42, raw, written));

    raw[HEADER_SIZE + CONTROL_COMMAND_PAYLOAD_SIZE - 1] =
        0xFF;

    rewriteCrc(raw);

    HilPacket decoded{};
    EXPECT_FALSE(decodePacket(raw, decoded));
}

TEST(HilPacketCodecTests, InvalidEncodingPreservesDestination)
{
    auto sample = makeSensor();
    sample.angularVelocityB.z =
        std::numeric_limits<double>::infinity();

    std::array<std::uint8_t, SENSOR_PACKET_SIZE> raw{};
    raw.fill(0xA5);

    const auto original = raw;
    std::size_t written = 888;

    EXPECT_FALSE(encodeSensorPacket(
        sample, 42, raw, written));

    EXPECT_EQ(raw, original);
    EXPECT_EQ(written, 888u);
}

TEST(HilPacketCodecTests, RejectsInvalidStatusOnEncode)
{
    auto command = makeControl();
    command.controllerStatus = 255;

    std::array<std::uint8_t, CONTROL_PACKET_SIZE> raw{};
    raw.fill(0xA5);

    const auto original = raw;
    std::size_t written = 888;

    EXPECT_FALSE(encodeControlPacket(
        command, 42, raw, written));

    EXPECT_EQ(raw, original);
    EXPECT_EQ(written, 888u);
}

TEST(HilPacketCodecTests, FramedSensorPacketRoundTrip)
{
    std::array<std::uint8_t, MAX_FRAMED_PACKET_SIZE>
        frame{};

    std::size_t frameSize = 0;

    ASSERT_TRUE(encodeSensorFrame(
        makeSensor(), 77, frame, frameSize));

    ASSERT_GT(frameSize, 1u);
    EXPECT_EQ(frame[frameSize - 1], 0x00);

    for (std::size_t i = 0; i + 1 < frameSize; ++i)
        EXPECT_NE(frame[i], 0x00);

    std::array<std::uint8_t, MAX_RAW_PACKET_SIZE> raw{};
    std::size_t rawSize = 0;

    ASSERT_TRUE(cobsDecode(
        std::span<const std::uint8_t>(
            frame.data(), frameSize - 1),
        raw, rawSize));

    HilPacket decoded{};

    ASSERT_TRUE(decodePacket(
        std::span<const std::uint8_t>(
            raw.data(), rawSize),
        decoded));

    EXPECT_EQ(decoded.header.sequence, 77u);
}