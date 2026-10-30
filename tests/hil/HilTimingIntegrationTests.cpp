
#include <gtest/gtest.h>

#include <hil/HilTimingMonitor.h>
#include <hil/HilSimulationBridge.h>
#include <hil/HilWireEndpoint.h>
#include <hil/HilPacketCodec.h>

#include <sensors/ImuModel.h>

#include "HilTestByteLink.h"

#include <variant>

using namespace aeroforge;
using namespace aeroforge::hil;
using namespace aeroforge::hil::tests;

TEST(HilTimingIntegrationTests, BridgeRecordsCommandLatency)
{
    HilTestByteLink link;

    HilWireEndpoint host(link.portA());
    HilWireEndpoint controller(link.portB());

    HilTimingMonitor monitor;
    HilBridgeConfig config;

    HilSimulationBridge bridge(host, config);
    bridge.attachTimingMonitor(&monitor);

    const sensors::ImuMeasurement measurement{
        0.01,
        math::Vector3(0, 0, 0),
        math::Vector3(0, 0, 0.2)
    };

    ASSERT_TRUE(bridge.publishSensorSample(measurement));

    bridge.pump(10000);

    controller.pollIncoming();

    HilPacket sensorPacket{};
    ASSERT_TRUE(controller.tryReceive(sensorPacket));

    ControlCommandMessage command{};
    command.referencedSensorSequence =
        sensorPacket.header.sequence;

    command.commandedMomentB =
        math::Vector3(0, 0, -1);

    command.controllerStatus =
        static_cast<std::uint8_t>(
            ControllerStatus::OK);

    ASSERT_TRUE(controller.queueControl(command, 0));

    ASSERT_TRUE(
        controller.flushPending().has_value());

    bridge.pump(11000);

    EXPECT_EQ(bridge.stats().acceptedCommands, 1u);

    const auto& stats = monitor.statistics();

    EXPECT_EQ(stats.commandLatency.count, 1u);
    EXPECT_EQ(stats.unmatchedCommandResponses, 0u);

    EXPECT_GT(stats.communication.count, 0u);
    EXPECT_GE(stats.commandLatency.averageMs, 0.0);
}
