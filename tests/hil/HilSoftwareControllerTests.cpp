#include <gtest/gtest.h>

#include <hil/HilSoftwareController.h>
#include <hil/HilPacketCodec.h>
#include <hil/HilWireEndpoint.h>

#include "HilTestByteLink.h"

#include <cstdint>
#include <variant>

using namespace aeroforge;
using namespace aeroforge::hil;
using namespace aeroforge::hil::tests;

namespace
{
    class HilSoftwareControllerTests :
        public ::testing::Test
    {
    protected:
        HilTestByteLink link;

        HilWireEndpoint host{ link.portA() };
        HilWireEndpoint controllerEndpoint{ link.portB() };

        HilRateControllerConfig config{};

        HilSoftwareController controller{
            controllerEndpoint, config
        };

        bool transmitSensor(
            std::uint32_t sequence,
            std::uint64_t timestamp,
            const math::Vector3& angularVelocity)
        {
            SensorSampleMessage sample{};
            sample.simulationTimeUs = timestamp;
            sample.specificForceB = math::Vector3();
            sample.angularVelocityB = angularVelocity;

            if (!host.queueSensor(sample, sequence))
                return false;

            return host.flushPending().has_value();
        }

        bool readCommand(ControlCommandMessage& command)
        {
            host.pollIncoming();

            HilPacket packet{};

            if (!host.tryReceive(packet))
                return false;

            const auto* payload =
                std::get_if<ControlCommandMessage>(
                    &packet.payload);

            if (payload == nullptr)
                return false;

            command = *payload;
            return true;
        }
    };
}

TEST_F(HilSoftwareControllerTests, AppliesNegativeRateFeedback)
{
    ASSERT_TRUE(transmitSensor(
        10, 10000,
        math::Vector3(0.2, -0.4, 0.1)));

    controller.pump();

    ControlCommandMessage command{};
    ASSERT_TRUE(readCommand(command));

    EXPECT_EQ(command.referencedSensorSequence, 10u);

    EXPECT_NEAR(command.commandedMomentB.x, -1.0, 1e-6);
    EXPECT_NEAR(command.commandedMomentB.y, 2.0, 1e-6);
    EXPECT_NEAR(command.commandedMomentB.z, -0.5, 1e-6);

    EXPECT_EQ(command.controllerStatus,
        static_cast<std::uint8_t>(
            ControllerStatus::OK));
}

TEST_F(HilSoftwareControllerTests, ZeroRatesProduceZeroCommand)
{
    ASSERT_TRUE(transmitSensor(
        10, 10000, math::Vector3()));

    controller.pump();

    ControlCommandMessage command{};
    ASSERT_TRUE(readCommand(command));

    EXPECT_TRUE(command.commandedMomentB.isZero());
}

TEST_F(HilSoftwareControllerTests, SaturatesLargeRates)
{
    ASSERT_TRUE(transmitSensor(
        10, 10000,
        math::Vector3(100.0, -100.0, 200.0)));

    controller.pump();

    ControlCommandMessage command{};
    ASSERT_TRUE(readCommand(command));

    EXPECT_DOUBLE_EQ(command.commandedMomentB.x, -20.0);
    EXPECT_DOUBLE_EQ(command.commandedMomentB.y, 20.0);
    EXPECT_DOUBLE_EQ(command.commandedMomentB.z, -20.0);
}

TEST_F(HilSoftwareControllerTests, RejectsDuplicateSensorSequence)
{
    ASSERT_TRUE(transmitSensor(
        10, 10000, math::Vector3(0.1, 0, 0)));

    controller.pump();

    ControlCommandMessage command{};
    ASSERT_TRUE(readCommand(command));

    ASSERT_TRUE(transmitSensor(
        10, 20000, math::Vector3(0.2, 0, 0)));

    controller.pump();

    EXPECT_FALSE(readCommand(command));

    EXPECT_EQ(controller.stats().receivedSamples, 1u);
    EXPECT_EQ(controller.stats().generatedCommands, 1u);
    EXPECT_EQ(controller.stats().rejectedSamples, 1u);
}

TEST_F(HilSoftwareControllerTests, RejectsBackwardTimestamp)
{
    ASSERT_TRUE(transmitSensor(
        10, 20000, math::Vector3(0.1, 0, 0)));

    controller.pump();

    ControlCommandMessage command{};
    ASSERT_TRUE(readCommand(command));

    ASSERT_TRUE(transmitSensor(
        11, 10000, math::Vector3(0.2, 0, 0)));

    controller.pump();

    EXPECT_FALSE(readCommand(command));

    EXPECT_EQ(controller.stats().rejectedSamples, 1u);
}

TEST_F(HilSoftwareControllerTests, DropsCommandWhenOutputBlocked)
{
    link.blockBWrites = true;

    ASSERT_TRUE(transmitSensor(
        10, 10000, math::Vector3(0.1, 0, 0)));

    controller.pump();

    EXPECT_TRUE(
        controllerEndpoint.hasPendingTransmission());

    ASSERT_TRUE(transmitSensor(
        11, 20000, math::Vector3(0.2, 0, 0)));

    controller.pump();

    EXPECT_EQ(controller.stats().receivedSamples, 2u);
    EXPECT_EQ(controller.stats().generatedCommands, 1u);
    EXPECT_EQ(controller.stats().droppedCommands, 1u);
}

TEST_F(HilSoftwareControllerTests, ResetClearsSequenceState)
{
    ASSERT_TRUE(transmitSensor(
        10, 10000, math::Vector3(0.1, 0, 0)));

    controller.pump();

    ControlCommandMessage command{};
    ASSERT_TRUE(readCommand(command));

    controller.reset();

    ASSERT_TRUE(transmitSensor(
        0, 0, math::Vector3(0.1, 0, 0)));

    controller.pump();

    ASSERT_TRUE(readCommand(command));

    EXPECT_EQ(command.referencedSensorSequence, 0u);
    EXPECT_EQ(controller.stats().generatedCommands, 1u);
}