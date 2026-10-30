#include <gtest/gtest.h>

#include <hil/HilWireEndpoint.h>
#include <hil/HilPacketCodec.h>
#include <hil/HilCobs.h>

#include "HilTestByteLink.h"

#include <array>
#include <cstdint>
#include <span>
#include <variant>

using namespace aeroforge;
using namespace aeroforge::hil;
using namespace aeroforge::hil::tests;

namespace
{
    SensorSampleMessage endpointSample()
    {
        SensorSampleMessage sample{};
        sample.simulationTimeUs = 10000;
        sample.specificForceB = math::Vector3(1, 2, 3);
        sample.angularVelocityB =
            math::Vector3(0.1, -0.2, 0.3);
        return sample;
    }
}

TEST(HilWireEndpointTests, SensorTransfersBetweenEndpoints)
{
    HilTestByteLink link;

    HilWireEndpoint sender(link.portA());
    HilWireEndpoint receiver(link.portB());

    ASSERT_TRUE(sender.queueSensor(endpointSample(), 42));

    const auto receipt = sender.flushPending();

    ASSERT_TRUE(receipt.has_value());
    EXPECT_EQ(receipt->sequence, 42u);
    EXPECT_EQ(receipt->sensorTimeUs, 10000u);
    EXPECT_EQ(receipt->type, MessageType::SENSOR_SAMPLE);

    receiver.pollIncoming();

    HilPacket packet{};
    ASSERT_TRUE(receiver.tryReceive(packet));

    EXPECT_EQ(packet.header.sequence, 42u);

    const auto* sample =
        std::get_if<SensorSampleMessage>(&packet.payload);

    ASSERT_NE(sample, nullptr);

    EXPECT_EQ(sample->simulationTimeUs, 10000u);
    EXPECT_DOUBLE_EQ(sample->specificForceB.x, 1.0);
}

TEST(HilWireEndpointTests, ControlTransfersInReverseDirection)
{
    HilTestByteLink link;

    HilWireEndpoint host(link.portA());
    HilWireEndpoint controller(link.portB());

    ControlCommandMessage command{};
    command.referencedSensorSequence = 42;
    command.commandedMomentB =
        math::Vector3(1.0, -2.0, 3.0);

    ASSERT_TRUE(controller.queueControl(command, 99));

    ASSERT_TRUE(
        controller.flushPending().has_value());

    host.pollIncoming();

    HilPacket packet{};
    ASSERT_TRUE(host.tryReceive(packet));

    EXPECT_EQ(packet.header.sequence, 99u);

    const auto* decoded =
        std::get_if<ControlCommandMessage>(&packet.payload);

    ASSERT_NE(decoded, nullptr);

    EXPECT_EQ(decoded->referencedSensorSequence, 42u);
    EXPECT_DOUBLE_EQ(decoded->commandedMomentB.y, -2.0);
}

TEST(HilWireEndpointTests, PartialWritesResumeCorrectly)
{
    HilTestByteLink link;
    link.maxWriteChunk = 3;

    HilWireEndpoint sender(link.portA());
    HilWireEndpoint receiver(link.portB());

    ASSERT_TRUE(sender.queueSensor(endpointSample(), 7));

    std::optional<HilSendReceipt> receipt;

    for (int i = 0; i < 100; ++i)
    {
        receipt = sender.flushPending();

        if (receipt)
            break;
    }

    ASSERT_TRUE(receipt.has_value());
    EXPECT_EQ(receipt->sequence, 7u);
    EXPECT_FALSE(sender.hasPendingTransmission());

    receiver.pollIncoming();

    HilPacket packet{};
    ASSERT_TRUE(receiver.tryReceive(packet));

    EXPECT_EQ(packet.header.sequence, 7u);
}

TEST(HilWireEndpointTests, PartialReadsReassemblePacket)
{
    HilTestByteLink link;
    link.maxReadChunk = 2;

    HilWireEndpoint sender(link.portA());
    HilWireEndpoint receiver(link.portB());

    ASSERT_TRUE(sender.queueSensor(endpointSample(), 7));
    ASSERT_TRUE(sender.flushPending().has_value());

    receiver.pollIncoming(1);

    HilPacket packet{};
    EXPECT_FALSE(receiver.tryReceive(packet));

    for (int i = 0; i < 100; ++i)
        receiver.pollIncoming(1);

    ASSERT_TRUE(receiver.tryReceive(packet));

    EXPECT_EQ(packet.header.sequence, 7u);
}

TEST(HilWireEndpointTests, CannotOverwritePendingFrame)
{
    HilTestByteLink link;
    link.blockAWrites = true;

    HilWireEndpoint sender(link.portA());

    ASSERT_TRUE(sender.queueSensor(endpointSample(), 1));

    EXPECT_FALSE(
        sender.queueSensor(endpointSample(), 2));

    EXPECT_FALSE(sender.flushPending().has_value());
    EXPECT_TRUE(sender.hasPendingTransmission());

    link.blockAWrites = false;

    EXPECT_TRUE(sender.flushPending().has_value());
    EXPECT_FALSE(sender.hasPendingTransmission());
}

TEST(HilWireEndpointTests, RejectsCorruptedRawPacket)
{
    HilTestByteLink link;

    HilWireEndpoint receiver(link.portB());

    std::array<std::uint8_t, SENSOR_PACKET_SIZE> raw{};
    std::size_t rawSize = 0;

    ASSERT_TRUE(encodeSensorPacket(
        endpointSample(), 42, raw, rawSize));

    // Corrupt payload without updating CRC.
    raw[18] ^= 0x01;

    std::array<std::uint8_t, MAX_COBS_ENCODED_SIZE>
        encoded{};

    std::size_t encodedSize = 0;

    ASSERT_TRUE(cobsEncode(
        std::span<const std::uint8_t>(
            raw.data(), rawSize),
        encoded,
        encodedSize));

    for (std::size_t i = 0; i < encodedSize; ++i)
        link.aToB.push_back(encoded[i]);

    link.aToB.push_back(0x00);

    receiver.pollIncoming();

    HilPacket packet{};

    EXPECT_FALSE(receiver.tryReceive(packet));
    EXPECT_EQ(receiver.invalidPacketCount(), 1u);
}

TEST(HilWireEndpointTests, ResetDiscardsPendingTransmission)
{
    HilTestByteLink link;
    link.blockAWrites = true;

    HilWireEndpoint sender(link.portA());

    ASSERT_TRUE(sender.queueSensor(endpointSample(), 1));
    ASSERT_TRUE(sender.hasPendingTransmission());

    sender.reset();

    EXPECT_FALSE(sender.hasPendingTransmission());
    EXPECT_EQ(sender.invalidPacketCount(), 0u);
}