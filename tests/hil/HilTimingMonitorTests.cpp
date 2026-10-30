
#include <gtest/gtest.h>

#include <hil/HilTimingMonitor.h>

#include <chrono>
#include <cstdint>
#include <stdexcept>

using namespace aeroforge::hil;
using namespace std::chrono_literals;

namespace
{
    using Clock = HilTimingMonitor::Clock;
    const Clock::time_point start{};
}

TEST(HilTimingMonitorTests, StartsWithZeroStatistics)
{
    HilTimingMonitor monitor;

    const auto& stats = monitor.statistics();

    EXPECT_EQ(stats.simulationSteps.count, 0u);
    EXPECT_EQ(stats.communication.count, 0u);
    EXPECT_EQ(stats.commandLatency.count, 0u);
    EXPECT_EQ(stats.stepDeadlineMisses, 0u);
    EXPECT_EQ(stats.unmatchedCommandResponses, 0u);
}

TEST(HilTimingMonitorTests, RecordsSimulationStepDurations)
{
    HilTimingMonitor monitor(10ms);

    ASSERT_TRUE(monitor.recordSimulationStep(2ms));
    ASSERT_TRUE(monitor.recordSimulationStep(4ms));
    ASSERT_TRUE(monitor.recordSimulationStep(6ms));

    const auto& stats =
        monitor.statistics().simulationSteps;

    EXPECT_EQ(stats.count, 3u);
    EXPECT_DOUBLE_EQ(stats.averageMs, 4.0);
    EXPECT_DOUBLE_EQ(stats.maximumMs, 6.0);
}

TEST(HilTimingMonitorTests, DetectsDeadlineMisses)
{
    HilTimingMonitor monitor(10ms);

    ASSERT_TRUE(monitor.recordSimulationStep(5ms));
    ASSERT_TRUE(monitor.recordSimulationStep(11ms));
    ASSERT_TRUE(monitor.recordSimulationStep(15ms));

    EXPECT_EQ(
        monitor.statistics().stepDeadlineMisses,
        2u);

    EXPECT_EQ(
        monitor.statistics().simulationSteps.count,
        3u);
}

TEST(HilTimingMonitorTests, ExactDeadlineIsNotMissed)
{
    HilTimingMonitor monitor(10ms);

    ASSERT_TRUE(monitor.recordSimulationStep(10ms));

    EXPECT_EQ(
        monitor.statistics().stepDeadlineMisses,
        0u);
}

TEST(HilTimingMonitorTests, RecordsCommunicationSeparately)
{
    HilTimingMonitor monitor;

    ASSERT_TRUE(monitor.recordSimulationStep(5ms));

    ASSERT_TRUE(monitor.recordCommunication(1ms));
    ASSERT_TRUE(monitor.recordCommunication(3ms));

    const auto& stats = monitor.statistics();

    EXPECT_EQ(stats.simulationSteps.count, 1u);
    EXPECT_EQ(stats.communication.count, 2u);

    EXPECT_DOUBLE_EQ(
        stats.communication.averageMs, 2.0);

    EXPECT_DOUBLE_EQ(
        stats.communication.maximumMs, 3.0);
}

TEST(HilTimingMonitorTests, RejectsNegativeDurations)
{
    HilTimingMonitor monitor;

    const auto negative =
        -std::chrono::duration_cast<
        HilTimingMonitor::Duration>(1ms);

    EXPECT_FALSE(monitor.recordSimulationStep(negative));
    EXPECT_FALSE(monitor.recordCommunication(negative));

    EXPECT_EQ(
        monitor.statistics().simulationSteps.count,
        0u);

    EXPECT_EQ(
        monitor.statistics().communication.count,
        0u);

    EXPECT_EQ(
        monitor.statistics().stepDeadlineMisses,
        0u);
}

TEST(HilTimingMonitorTests, AcceptsZeroDuration)
{
    HilTimingMonitor monitor;

    ASSERT_TRUE(
        monitor.recordSimulationStep(
            HilTimingMonitor::Duration::zero()));

    EXPECT_EQ(
        monitor.statistics().simulationSteps.count,
        1u);

    EXPECT_DOUBLE_EQ(
        monitor.statistics().simulationSteps.averageMs,
        0.0);
}

TEST(HilTimingMonitorTests, MeasuresCommandLatency)
{
    HilTimingMonitor monitor;

    monitor.recordSensorSent(42, start);

    ASSERT_TRUE(monitor.recordCommandAccepted(
        42, start + 15ms));

    const auto& latency =
        monitor.statistics().commandLatency;

    EXPECT_EQ(latency.count, 1u);
    EXPECT_DOUBLE_EQ(latency.averageMs, 15.0);
    EXPECT_DOUBLE_EQ(latency.maximumMs, 15.0);
}

TEST(HilTimingMonitorTests, ComputesAverageCommandLatency)
{
    HilTimingMonitor monitor;

    monitor.recordSensorSent(1, start);
    monitor.recordSensorSent(2, start + 10ms);

    ASSERT_TRUE(monitor.recordCommandAccepted(
        1, start + 4ms));

    ASSERT_TRUE(monitor.recordCommandAccepted(
        2, start + 16ms));

    const auto& latency =
        monitor.statistics().commandLatency;

    EXPECT_EQ(latency.count, 2u);
    EXPECT_DOUBLE_EQ(latency.averageMs, 5.0);
    EXPECT_DOUBLE_EQ(latency.maximumMs, 6.0);
}

TEST(HilTimingMonitorTests, RejectsUnknownReference)
{
    HilTimingMonitor monitor;

    monitor.recordSensorSent(42, start);

    EXPECT_FALSE(monitor.recordCommandAccepted(
        999, start + 10ms));

    EXPECT_EQ(
        monitor.statistics().unmatchedCommandResponses,
        1u);

    EXPECT_EQ(
        monitor.statistics().commandLatency.count,
        0u);
}

TEST(HilTimingMonitorTests, DuplicateResponseDoesNotCountTwice)
{
    HilTimingMonitor monitor;

    monitor.recordSensorSent(42, start);

    ASSERT_TRUE(monitor.recordCommandAccepted(
        42, start + 5ms));

    EXPECT_FALSE(monitor.recordCommandAccepted(
        42, start + 10ms));

    EXPECT_EQ(
        monitor.statistics().commandLatency.count,
        1u);

    EXPECT_EQ(
        monitor.statistics().unmatchedCommandResponses,
        1u);
}

TEST(HilTimingMonitorTests, RejectsBackwardsTimestamp)
{
    HilTimingMonitor monitor;

    monitor.recordSensorSent(42, start + 20ms);

    EXPECT_FALSE(monitor.recordCommandAccepted(
        42, start + 10ms));

    EXPECT_EQ(
        monitor.statistics().commandLatency.count,
        0u);

    // A failed timing match must not consume
    // the original sensor record.
    EXPECT_TRUE(monitor.recordCommandAccepted(
        42, start + 30ms));

    EXPECT_DOUBLE_EQ(
        monitor.statistics().commandLatency.averageMs,
        10.0);
}

TEST(HilTimingMonitorTests, MatchesResponsesOutOfOrder)
{
    HilTimingMonitor monitor;

    monitor.recordSensorSent(10, start);
    monitor.recordSensorSent(11, start + 1ms);

    ASSERT_TRUE(monitor.recordCommandAccepted(
        11, start + 5ms));

    ASSERT_TRUE(monitor.recordCommandAccepted(
        10, start + 6ms));

    EXPECT_EQ(
        monitor.statistics().commandLatency.count,
        2u);

    EXPECT_DOUBLE_EQ(
        monitor.statistics().commandLatency.averageMs,
        5.0);
}

TEST(HilTimingMonitorTests, UpdatesRepeatedSensorSequence)
{
    HilTimingMonitor monitor;

    monitor.recordSensorSent(42, start);
    monitor.recordSensorSent(42, start + 10ms);

    ASSERT_TRUE(monitor.recordCommandAccepted(
        42, start + 15ms));

    EXPECT_DOUBLE_EQ(
        monitor.statistics().commandLatency.averageMs,
        5.0);
}

TEST(HilTimingMonitorTests, HistoryCapacityIsBounded)
{
    HilTimingMonitor monitor;

    for (std::uint32_t sequence = 0;
        sequence <
        HilTimingMonitor::SENSOR_HISTORY_CAPACITY + 1;
        ++sequence)
    {
        monitor.recordSensorSent(
            sequence,
            start + std::chrono::milliseconds(sequence));
    }

    // Sequence zero should have been evicted.
    EXPECT_FALSE(monitor.recordCommandAccepted(
        0, start + 200ms));

    const auto latestSequence =
        static_cast<std::uint32_t>(
            HilTimingMonitor::SENSOR_HISTORY_CAPACITY);

    EXPECT_TRUE(monitor.recordCommandAccepted(
        latestSequence, start + 200ms));
}

TEST(HilTimingMonitorTests, ResetClearsStatisticsAndHistory)
{
    HilTimingMonitor monitor;

    ASSERT_TRUE(monitor.recordSimulationStep(20ms));
    ASSERT_TRUE(monitor.recordCommunication(2ms));

    monitor.recordSensorSent(42, start);

    monitor.reset();

    const auto& stats = monitor.statistics();

    EXPECT_EQ(stats.simulationSteps.count, 0u);
    EXPECT_EQ(stats.communication.count, 0u);
    EXPECT_EQ(stats.commandLatency.count, 0u);
    EXPECT_EQ(stats.stepDeadlineMisses, 0u);

    EXPECT_FALSE(monitor.recordCommandAccepted(
        42, start + 10ms));
}

TEST(HilTimingMonitorTests, RejectsInvalidDeadline)
{
    EXPECT_THROW(
        HilTimingMonitor(0ms),
        std::invalid_argument);

    EXPECT_THROW(
        HilTimingMonitor(-1ms),
        std::invalid_argument);
}
