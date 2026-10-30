#include <gtest/gtest.h>

#include <hil/HilSerialWorker.h>
#include <hil/HilWireEndpoint.h>
#include <hil/HilSimulationBridge.h>
#include <hil/HilTimingMonitor.h>
#include <hil/HilPacketCodec.h>
#include <hil/HilStreamParser.h>

#include <sensors/ImuModel.h>

#include "HilFakeByteDevice.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>
#include <variant>
#include <vector>

using namespace aeroforge;
using namespace aeroforge::hil;
using namespace aeroforge::hil::tests;
using namespace std::chrono_literals;

namespace
{
    bool waitForSerial(
        const std::function<bool()>& condition,
        std::chrono::milliseconds timeout = 2000ms)
    {
        const auto deadline =
            std::chrono::steady_clock::now() + timeout;

        while (std::chrono::steady_clock::now() < deadline)
        {
            if (condition())
                return true;

            std::this_thread::sleep_for(1ms);
        }

        return condition();
    }

    class HilSerialIntegrationTests : public ::testing::Test
    {
    protected:
        std::shared_ptr<FakeByteDeviceState> fake =
            std::make_shared<FakeByteDeviceState>();

        HilSerialWorker worker{
            std::make_unique<HilFakeByteDevice>(fake)
        };

        HilWireEndpoint endpoint{
            worker.makePort()
        };

        HilBridgeConfig config{};
        HilSimulationBridge bridge{
            endpoint, config
        };

        HilTimingMonitor timingMonitor;

        void SetUp() override
        {
            bridge.attachTimingMonitor(&timingMonitor);

            ASSERT_TRUE(worker.start());

            ASSERT_TRUE(waitForSerial([this]
                {
                    return worker.state() ==
                        HilSerialWorkerState::RUNNING;
                }));
        }

        void TearDown() override
        {
            bridge.notifyTransportDisconnected();
            worker.stop();
        }

        bool transmitAndDecodeSensor(
            HilPacket& sensorPacket)
        {
            const sensors::ImuMeasurement measurement{
                0.01,
                math::Vector3(0, 0, 0),
                math::Vector3(0, 0, 0.25)
            };

            if (!bridge.publishSensorSample(measurement))
                return false;

            std::vector<std::uint8_t> transmitted;

            const bool completed = waitForSerial([&]
                {
                    bridge.pump(10000);

                    const auto chunk = fake->takeOutgoing();

                    transmitted.insert(
                        transmitted.end(),
                        chunk.begin(),
                        chunk.end());

                    return !transmitted.empty() &&
                        transmitted.back() == 0x00 &&
                        !endpoint.hasPendingTransmission();
                });

            if (!completed)
                return false;

            HilStreamParser parser;
            parser.consume(transmitted);

            std::array<std::uint8_t,
                MAX_RAW_PACKET_SIZE> raw{};

            std::size_t rawSize = 0;

            if (!parser.tryPopPacket(raw, rawSize))
                return false;

            return decodePacket(
                std::span<const std::uint8_t>(
                    raw.data(), rawSize),
                sensorPacket);
        }

        bool injectValidControl(
            std::uint32_t referencedSensorSequence)
        {
            ControlCommandMessage command{};

            command.referencedSensorSequence =
                referencedSensorSequence;

            command.commandedMomentB =
                math::Vector3(0, 0, -1.25);

            command.controllerStatus =
                static_cast<std::uint8_t>(
                    ControllerStatus::OK);

            std::array<std::uint8_t,
                MAX_FRAMED_PACKET_SIZE> frame{};

            std::size_t frameSize = 0;

            if (!encodeControlFrame(
                command,
                0,
                frame,
                frameSize))
            {
                return false;
            }

            fake->inject(
                std::span<const std::uint8_t>(
                    frame.data(),
                    frameSize));

            return waitForSerial([this]
                {
                    bridge.pump(20000);
                    return bridge.stats().acceptedCommands == 1;
                });
        }
    };
}

TEST_F(
    HilSerialIntegrationTests,
    SensorAndControlExchangeAcrossSerialWorker)
{
    HilPacket sensorPacket{};

    ASSERT_TRUE(
        transmitAndDecodeSensor(sensorPacket));

    EXPECT_EQ(
        sensorPacket.header.messageType,
        static_cast<std::uint8_t>(
            MessageType::SENSOR_SAMPLE));

    const auto* sample =
        std::get_if<SensorSampleMessage>(
            &sensorPacket.payload);

    ASSERT_NE(sample, nullptr);

    EXPECT_EQ(sample->simulationTimeUs, 10000u);

    ASSERT_TRUE(injectValidControl(
        sensorPacket.header.sequence));

    const math::Vector3 command =
        bridge.currentCommandedMomentB(20000);

    EXPECT_NEAR(command.z, -1.25, 1e-6);
    EXPECT_EQ(bridge.stats().acceptedCommands, 1u);

    const auto& timing = timingMonitor.statistics();

    EXPECT_EQ(timing.commandLatency.count, 1u);
    EXPECT_EQ(timing.unmatchedCommandResponses, 0u);
    EXPECT_GT(timing.communication.count, 0u);
}

TEST_F(
    HilSerialIntegrationTests,
    TransportFailureForcesZeroCommand)
{
    HilPacket sensorPacket{};

    ASSERT_TRUE(
        transmitAndDecodeSensor(sensorPacket));

    ASSERT_TRUE(injectValidControl(
        sensorPacket.header.sequence));

    EXPECT_FALSE(
        bridge.currentCommandedMomentB(20000).isZero());

    // Simulate physical serial device failure.
    fake->failReads = true;

    ASSERT_TRUE(waitForSerial([this]
        {
            return worker.state() ==
                HilSerialWorkerState::FAILED;
        }));

    // The simulation-side coordinator observes the
    // failure and notifies the bridge.
    bridge.notifyTransportDisconnected();

    EXPECT_EQ(
        bridge.watchdogState(),
        HilWatchdogState::DISCONNECTED);

    EXPECT_TRUE(
        bridge.currentCommandedMomentB(20000).isZero());
}

TEST_F(
    HilSerialIntegrationTests,
    ExplicitRestartLeavesBridgeDisarmed)
{
    HilPacket sensorPacket{};

    ASSERT_TRUE(
        transmitAndDecodeSensor(sensorPacket));

    ASSERT_TRUE(injectValidControl(
        sensorPacket.header.sequence));

    fake->failReads = true;

    ASSERT_TRUE(waitForSerial([this]
        {
            return worker.state() ==
                HilSerialWorkerState::FAILED;
        }));

    bridge.notifyTransportDisconnected();
    worker.stop();

    fake->failReads = false;

    ASSERT_TRUE(worker.start());

    ASSERT_TRUE(waitForSerial([this]
        {
            return worker.state() ==
                HilSerialWorkerState::RUNNING;
        }));

    // Simulate deliberate peer synchronization/reset.
    bridge.reset();
    timingMonitor.reset();

    EXPECT_EQ(
        bridge.watchdogState(),
        HilWatchdogState::WAITING);

    EXPECT_TRUE(
        bridge.currentCommandedMomentB(20000).isZero());

    EXPECT_EQ(bridge.stats().acceptedCommands, 0u);
    EXPECT_EQ(timingMonitor.statistics().commandLatency.count, 0u);
}
