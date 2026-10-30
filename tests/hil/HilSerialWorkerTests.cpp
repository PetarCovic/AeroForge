#include <gtest/gtest.h>

#include <hil/HilSerialWorker.h>
#include <hil/HilWireEndpoint.h>
#include <hil/HilPacketCodec.h>
#include <hil/HilStreamParser.h>

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
    bool waitUntil(
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

    SensorSampleMessage makeSample()
    {
        SensorSampleMessage sample{};
        sample.simulationTimeUs = 10000;
        sample.specificForceB = math::Vector3(1, 2, 3);
        sample.angularVelocityB =
            math::Vector3(0.1, -0.2, 0.3);

        return sample;
    }

    class HilSerialWorkerTests : public ::testing::Test
    {
    protected:
        std::shared_ptr<FakeByteDeviceState> fake =
            std::make_shared<FakeByteDeviceState>();

        HilSerialWorker worker{
            std::make_unique<HilFakeByteDevice>(fake)
        };

        void SetUp() override
        {
            ASSERT_TRUE(worker.start());

            ASSERT_TRUE(waitUntil([this]
                {
                    return worker.state() ==
                        HilSerialWorkerState::RUNNING;
                }));
        }

        void TearDown() override
        {
            worker.stop();
        }
    };
}

TEST_F(HilSerialWorkerTests, OpensAndStopsCleanly)
{
    EXPECT_EQ(
        worker.state(),
        HilSerialWorkerState::RUNNING);

    worker.stop();

    EXPECT_EQ(
        worker.state(),
        HilSerialWorkerState::STOPPED);
}

TEST_F(HilSerialWorkerTests, SendsCompletePacket)
{
    HilWireEndpoint endpoint(worker.makePort());

    ASSERT_TRUE(endpoint.queueSensor(makeSample(), 42));

    std::optional<HilSendReceipt> receipt;

    ASSERT_TRUE(waitUntil([&]
        {
            receipt = endpoint.flushPending();
            return receipt.has_value();
        }));

    EXPECT_EQ(receipt->sequence, 42u);
    EXPECT_EQ(receipt->sensorTimeUs, 10000u);

    const auto outgoing = fake->takeOutgoing();

    const std::vector<std::uint8_t> framed(
        outgoing.begin(), outgoing.end());

    HilStreamParser parser;
    parser.consume(framed);

    std::array<std::uint8_t, MAX_RAW_PACKET_SIZE> raw{};
    std::size_t size = 0;

    ASSERT_TRUE(parser.tryPopPacket(raw, size));

    HilPacket decoded{};

    ASSERT_TRUE(decodePacket(
        std::span<const std::uint8_t>(
            raw.data(), size),
        decoded));

    EXPECT_EQ(decoded.header.sequence, 42u);

    const auto* sample =
        std::get_if<SensorSampleMessage>(
            &decoded.payload);

    ASSERT_NE(sample, nullptr);
    EXPECT_EQ(sample->simulationTimeUs, 10000u);
}

TEST_F(HilSerialWorkerTests, HandlesPartialPhysicalWrites)
{
    fake->maxWriteChunk = 2;

    HilWireEndpoint endpoint(worker.makePort());

    ASSERT_TRUE(endpoint.queueSensor(makeSample(), 9));

    std::optional<HilSendReceipt> receipt;

    ASSERT_TRUE(waitUntil([&]
        {
            receipt = endpoint.flushPending();
            return receipt.has_value();
        }));

    EXPECT_EQ(receipt->sequence, 9u);
    EXPECT_FALSE(endpoint.hasPendingTransmission());

    const auto outgoing = fake->takeOutgoing();

    HilStreamParser parser;
    parser.consume(
        std::vector<std::uint8_t>(
            outgoing.begin(), outgoing.end()));

    std::array<std::uint8_t, MAX_RAW_PACKET_SIZE> raw{};
    std::size_t size = 0;

    ASSERT_TRUE(parser.tryPopPacket(raw, size));

    HilPacket packet{};

    ASSERT_TRUE(decodePacket(
        std::span<const std::uint8_t>(
            raw.data(), size),
        packet));

    EXPECT_EQ(packet.header.sequence, 9u);
}

TEST_F(HilSerialWorkerTests, BlockedWriteDoesNotAcknowledge)
{
    fake->blockWrites = true;

    HilWireEndpoint endpoint(worker.makePort());

    ASSERT_TRUE(endpoint.queueSensor(makeSample(), 1));

    EXPECT_FALSE(
        endpoint.flushPending().has_value());

    EXPECT_TRUE(endpoint.hasPendingTransmission());

    EXPECT_FALSE(
        endpoint.queueSensor(makeSample(), 2));

    fake->blockWrites = false;

    std::optional<HilSendReceipt> receipt;

    ASSERT_TRUE(waitUntil([&]
        {
            receipt = endpoint.flushPending();
            return receipt.has_value();
        }));

    EXPECT_EQ(receipt->sequence, 1u);
}

TEST_F(HilSerialWorkerTests, ReceivesFramedPacket)
{
    HilWireEndpoint endpoint(worker.makePort());

    ControlCommandMessage command{};
    command.referencedSensorSequence = 123;
    command.commandedMomentB =
        math::Vector3(1, -2, 3);

    std::array<std::uint8_t, MAX_FRAMED_PACKET_SIZE>
        frame{};

    std::size_t frameSize = 0;

    ASSERT_TRUE(encodeControlFrame(
        command, 77, frame, frameSize));

    fake->inject(
        std::span<const std::uint8_t>(
            frame.data(), frameSize));

    HilPacket decoded{};

    ASSERT_TRUE(waitUntil([&]
        {
            endpoint.pollIncoming();
            return endpoint.tryReceive(decoded);
        }));

    EXPECT_EQ(decoded.header.sequence, 77u);

    const auto* result =
        std::get_if<ControlCommandMessage>(
            &decoded.payload);

    ASSERT_NE(result, nullptr);

    EXPECT_EQ(
        result->referencedSensorSequence,
        123u);

    EXPECT_DOUBLE_EQ(
        result->commandedMomentB.y,
        -2.0);
}

TEST_F(HilSerialWorkerTests, DetectsReadFailure)
{
    fake->failReads = true;

    ASSERT_TRUE(waitUntil([&]
        {
            return worker.state() ==
                HilSerialWorkerState::FAILED;
        }));

    EXPECT_EQ(worker.lastError(), 1002u);

    auto port = worker.makePort();

    std::array<std::uint8_t, 10> output{};

    EXPECT_EQ(port.tryRead(output), 0u);
}

TEST_F(HilSerialWorkerTests, DetectsWriteFailure)
{
    HilWireEndpoint endpoint(worker.makePort());

    fake->failWrites = true;

    ASSERT_TRUE(endpoint.queueSensor(makeSample(), 1));

    EXPECT_FALSE(
        endpoint.flushPending().has_value());

    ASSERT_TRUE(waitUntil([&]
        {
            return worker.state() ==
                HilSerialWorkerState::FAILED;
        }));

    EXPECT_EQ(worker.lastError(), 1003u);
}

TEST_F(HilSerialWorkerTests, CanRestartAfterFailure)
{
    fake->failReads = true;

    ASSERT_TRUE(waitUntil([&]
        {
            return worker.state() ==
                HilSerialWorkerState::FAILED;
        }));

    worker.stop();

    fake->failReads = false;

    ASSERT_TRUE(worker.start());

    ASSERT_TRUE(waitUntil([&]
        {
            return worker.state() ==
                HilSerialWorkerState::RUNNING;
        }));
}

TEST(HilSerialWorkerStartupTests, ReportsOpenFailure)
{
    auto fake = std::make_shared<FakeByteDeviceState>();
    fake->allowOpen = false;

    HilSerialWorker worker(
        std::make_unique<HilFakeByteDevice>(fake));

    ASSERT_TRUE(worker.start());

    ASSERT_TRUE(waitUntil([&]
        {
            return worker.state() ==
                HilSerialWorkerState::FAILED;
        }));

    EXPECT_EQ(worker.lastError(), 1001u);

    worker.stop();
}

TEST_F(
    HilSerialWorkerTests,
    ReceiveBufferOverflowFailsClosed)
{
    std::vector<std::uint8_t> bytes(
        HilSerialWorker::RX_CAPACITY + 1024,
        0x55);

    fake->inject(bytes);

    ASSERT_TRUE(waitUntil([this]
        {
            return worker.state() ==
                HilSerialWorkerState::FAILED;
        }));

    EXPECT_EQ(
        worker.lastError(),
        HIL_SERIAL_RX_OVERFLOW);

    EXPECT_GT(worker.droppedReceiveBytes(), 0u);
}