#include <gtest/gtest.h>

#include <hil/HilSimulationBridge.h>
#include <hil/HilWireEndpoint.h>
#include <hil/HilPacketCodec.h>

#include <sensors/ImuModel.h>

#include "HilTestByteLink.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <variant>

using namespace aeroforge;
using namespace aeroforge::hil;
using namespace aeroforge::hil::tests;

namespace
{
    class HilSimulationBridgeTests :
        public ::testing::Test
    {
    protected:
        HilTestByteLink link;

        HilWireEndpoint host{ link.portA() };
        HilWireEndpoint peer{ link.portB() };

        HilBridgeConfig config{};
        HilSimulationBridge bridge{ host, config };

        std::uint32_t lastSensorSequence = 0;

        bool sendSensor(double timeS)
        {
            sensors::ImuMeasurement measurement{
                timeS,
                math::Vector3(1, 2, 3),
                math::Vector3(0.1, 0.2, 0.3)
            };

            if (!bridge.publishSensorSample(measurement))
                return false;

            std::uint64_t timeUs = 0;

            if (!HilSimulationBridge::secondsToMicroseconds(
                timeS, timeUs))
            {
                return false;
            }

            bridge.pump(timeUs);

            peer.pollIncoming();

            HilPacket packet{};

            if (!peer.tryReceive(packet))
                return false;

            if (packet.header.messageType !=
                static_cast<std::uint8_t>(
                    MessageType::SENSOR_SAMPLE))
            {
                return false;
            }

            lastSensorSequence = packet.header.sequence;
            return true;
        }

        bool sendCommand(
            std::uint32_t reference,
            const math::Vector3& moment,
            std::uint32_t packetSequence,
            std::uint64_t nowUs,
            ControllerStatus status = ControllerStatus::OK)
        {
            ControlCommandMessage command{};
            command.referencedSensorSequence = reference;
            command.commandedMomentB = moment;
            command.controllerStatus =
                static_cast<std::uint8_t>(status);

            if (!peer.queueControl(command, packetSequence))
                return false;

            if (!peer.flushPending().has_value())
                return false;

            bridge.pump(nowUs);
            return true;
        }
    };
}

TEST_F(HilSimulationBridgeTests, SendsSensorMeasurements)
{
    ASSERT_TRUE(sendSensor(0.01));

    EXPECT_EQ(bridge.stats().queuedSamples, 1u);
    EXPECT_EQ(bridge.stats().droppedSamples, 0u);
    EXPECT_EQ(lastSensorSequence, 0u);
}

TEST_F(HilSimulationBridgeTests, AcceptsValidCommand)
{
    ASSERT_TRUE(sendSensor(0.01));

    ASSERT_TRUE(sendCommand(
        lastSensorSequence,
        math::Vector3(1, -2, 3),
        0,
        20000));

    const math::Vector3 actual =
        bridge.currentCommandedMomentB(20000);

    EXPECT_DOUBLE_EQ(actual.x, 1.0);
    EXPECT_DOUBLE_EQ(actual.y, -2.0);
    EXPECT_DOUBLE_EQ(actual.z, 3.0);

    EXPECT_EQ(bridge.stats().acceptedCommands, 1u);
}

TEST_F(HilSimulationBridgeTests, RejectsUnknownReference)
{
    ASSERT_TRUE(sendSensor(0.01));

    ASSERT_TRUE(sendCommand(
        999,
        math::Vector3(1, 2, 3),
        0,
        20000));

    EXPECT_EQ(bridge.stats().acceptedCommands, 0u);
    EXPECT_EQ(bridge.stats().rejectedCommands, 1u);

    EXPECT_TRUE(
        bridge.currentCommandedMomentB(20000).isZero());
}

TEST_F(HilSimulationBridgeTests, RejectsExcessiveMoment)
{
    ASSERT_TRUE(sendSensor(0.01));

    ASSERT_TRUE(sendCommand(
        lastSensorSequence,
        math::Vector3(1000, 0, 0),
        0,
        20000));

    EXPECT_EQ(bridge.stats().acceptedCommands, 0u);
    EXPECT_EQ(bridge.stats().rejectedCommands, 1u);

    EXPECT_TRUE(
        bridge.currentCommandedMomentB(20000).isZero());
}

TEST_F(HilSimulationBridgeTests, RejectsStaleCommand)
{
    ASSERT_TRUE(sendSensor(0.01));

    ASSERT_TRUE(sendCommand(
        lastSensorSequence,
        math::Vector3(1, 2, 3),
        0,
        200001));

    EXPECT_EQ(bridge.stats().acceptedCommands, 0u);

    EXPECT_TRUE(
        bridge.currentCommandedMomentB(200001).isZero());
}

TEST_F(HilSimulationBridgeTests, ExpiresOldAcceptedCommand)
{
    ASSERT_TRUE(sendSensor(0.01));

    ASSERT_TRUE(sendCommand(
        lastSensorSequence,
        math::Vector3(1, 2, 3),
        0,
        20000));

    EXPECT_FALSE(
        bridge.currentCommandedMomentB(20000).isZero());

    EXPECT_TRUE(
        bridge.currentCommandedMomentB(200001).isZero());
}

TEST_F(HilSimulationBridgeTests, RejectsDegradedStatusByDefault)
{
    ASSERT_TRUE(sendSensor(0.01));

    ASSERT_TRUE(sendCommand(
        lastSensorSequence,
        math::Vector3(1, 2, 3),
        0,
        20000,
        ControllerStatus::DEGRADED));

    EXPECT_EQ(bridge.stats().rejectedCommands, 1u);

    EXPECT_TRUE(
        bridge.currentCommandedMomentB(20000).isZero());
}

TEST_F(HilSimulationBridgeTests, FaultStatusClearsCommand)
{
    ASSERT_TRUE(sendSensor(0.01));

    ASSERT_TRUE(sendCommand(
        lastSensorSequence,
        math::Vector3(1, 2, 3),
        0,
        20000));

    EXPECT_FALSE(
        bridge.currentCommandedMomentB(20000).isZero());

    ASSERT_TRUE(sendCommand(
        lastSensorSequence,
        math::Vector3(),
        1,
        21000,
        ControllerStatus::FAULT));

    EXPECT_TRUE(
        bridge.currentCommandedMomentB(21000).isZero());
}

TEST_F(HilSimulationBridgeTests, DroppedSampleWhenOutputBusy)
{
    link.blockAWrites = true;

    const sensors::ImuMeasurement measurement{
        0.01,
        math::Vector3(),
        math::Vector3()
    };

    ASSERT_TRUE(bridge.publishSensorSample(measurement));

    EXPECT_FALSE(bridge.publishSensorSample(measurement));

    EXPECT_EQ(bridge.stats().queuedSamples, 1u);
    EXPECT_EQ(bridge.stats().droppedSamples, 1u);
}

TEST_F(HilSimulationBridgeTests, ResetClearsAcceptedCommand)
{
    ASSERT_TRUE(sendSensor(0.01));

    ASSERT_TRUE(sendCommand(
        lastSensorSequence,
        math::Vector3(1, 2, 3),
        0,
        20000));

    bridge.reset();

    EXPECT_TRUE(
        bridge.currentCommandedMomentB(20000).isZero());

    EXPECT_EQ(bridge.stats().acceptedCommands, 0u);
    EXPECT_EQ(bridge.stats().queuedSamples, 0u);
}

TEST(HilSimulationBridgeTimeTests, ConvertsSecondsToMicroseconds)
{
    std::uint64_t value = 999;

    ASSERT_TRUE(
        HilSimulationBridge::secondsToMicroseconds(
            1.234567, value));

    EXPECT_EQ(value, 1234567u);
}

TEST(HilSimulationBridgeTimeTests, RejectsNegativeTime)
{
    std::uint64_t value = 999;

    EXPECT_FALSE(
        HilSimulationBridge::secondsToMicroseconds(
            -1.0, value));

    EXPECT_EQ(value, 999u);
}

TEST(HilSimulationBridgeTimeTests, RejectsNaN)
{
    std::uint64_t value = 999;

    EXPECT_FALSE(
        HilSimulationBridge::secondsToMicroseconds(
            std::numeric_limits<double>::quiet_NaN(),
            value));

    EXPECT_EQ(value, 999u);
}

TEST(HilSimulationBridgeTimeTests, RejectsInfinity)
{
    std::uint64_t value = 999;

    EXPECT_FALSE(
        HilSimulationBridge::secondsToMicroseconds(
            std::numeric_limits<double>::infinity(),
            value));

    EXPECT_EQ(value, 999u);
}