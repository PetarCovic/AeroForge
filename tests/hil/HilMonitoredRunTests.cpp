#include <gtest/gtest.h>

#include <hil/HilMonitoredRun.h>
#include <hil/HilTelemetryStore.h>

#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <vector>

using namespace aeroforge::hil;

TEST(
    HilMonitoredRunTests,
    SoftwareLoopbackRunsSixDofAndController)
{
    HilTelemetryStore telemetry;

    HilMonitoredRunConfig config{};
    config.steps = 100;
    config.realtimePacing = false;

    const auto result =
        runHilMonitoredLoopback(
            telemetry,
            nullptr,
            std::stop_token{},
            config);

    EXPECT_FALSE(result.cancelled);
    EXPECT_EQ(result.completedSteps, 100u);

    EXPECT_EQ(
        result.finalHealth.mode,
        HilLinkMode::SOFTWARE_LOOPBACK);

    EXPECT_FALSE(result.finalHealth.running);

    EXPECT_EQ(
        result.finalHealth.stepCount,
        100u);

    EXPECT_GT(
        result.finalHealth.acceptedCommands,
        0u);

    EXPECT_GT(
        result.finalHealth.commandLatencyCount,
        0u);

    EXPECT_EQ(
        result.finalHealth.droppedSamples,
        0u);

    // No scheduling statistics in unpaced mode.
    EXPECT_EQ(
        result.scheduling.samples,
        0u);

    const auto snapshot =
        telemetry.snapshot();

    EXPECT_EQ(snapshot.count, 100u);

    EXPECT_NEAR(
        snapshot.points[99].simulationTimeS,
        1.0,
        1e-12);
}

TEST(
    HilMonitoredRunTests,
    InvalidConfigurationRejected)
{
    HilTelemetryStore telemetry;
    HilMonitoredRunConfig config{};

    config.stepS = 0.0;

    EXPECT_THROW(
        runHilMonitoredLoopback(
            telemetry,
            nullptr,
            std::stop_token{},
            config),
        std::invalid_argument);

    config.stepS = 0.01;
    config.steps = 0;

    EXPECT_THROW(
        runHilMonitoredLoopback(
            telemetry,
            nullptr,
            std::stop_token{},
            config),
        std::invalid_argument);
}

TEST(
    HilMonitoredRunTests,
    HonorsPriorCancellation)
{
    HilTelemetryStore telemetry;
    HilMonitoredRunConfig config{};

    config.realtimePacing = false;

    std::stop_source source;
    source.request_stop();

    const auto result =
        runHilMonitoredLoopback(
            telemetry,
            nullptr,
            source.get_token(),
            config);

    EXPECT_TRUE(result.cancelled);
    EXPECT_EQ(result.completedSteps, 0u);
    EXPECT_FALSE(result.finalHealth.running);
}

TEST(
    HilMonitoredRunTests,
    SummarizesStartLatenessWithNearestRankPercentiles)
{
    const std::vector<double> observations{
        1.0, 2.0, 3.0, 4.0, 5.0,
        6.0, 7.0, 8.0, 9.0, 10.0
    };

    const auto stats =
        summarizeScheduling(
            observations,
            2,
            1.75);

    EXPECT_EQ(stats.samples, 10u);

    EXPECT_DOUBLE_EQ(
        stats.meanStartLatenessMs,
        5.5);

    EXPECT_DOUBLE_EQ(
        stats.p50StartLatenessMs,
        5.0);

    EXPECT_DOUBLE_EQ(
        stats.p95StartLatenessMs,
        10.0);

    EXPECT_DOUBLE_EQ(
        stats.p99StartLatenessMs,
        10.0);

    EXPECT_DOUBLE_EQ(
        stats.maxStartLatenessMs,
        10.0);

    EXPECT_EQ(
        stats.completionDeadlineMisses,
        2u);

    EXPECT_DOUBLE_EQ(
        stats.maxCompletionOverrunMs,
        1.75);
}

TEST(
    HilMonitoredRunTests,
    RejectsInvalidSchedulingSamples)
{
    const std::vector<double> negative{
        -0.01
    };

    EXPECT_THROW(
        summarizeScheduling(
            negative, 0, 0.0),
        std::invalid_argument);

    const std::vector<double> nan{
        std::numeric_limits<double>::quiet_NaN()
    };

    EXPECT_THROW(
        summarizeScheduling(
            nan, 0, 0.0),
        std::invalid_argument);

    const std::vector<double> valid{
        0.1, 0.2
    };

    EXPECT_THROW(
        summarizeScheduling(
            valid, 3, 0.0),
        std::invalid_argument);
}

TEST(
    HilMonitoredRunTests,
    PacedRunCollectsSchedulingSamples)
{
    HilTelemetryStore telemetry;

    HilMonitoredRunConfig config{};

    config.steps = 8;
    config.stepS = 0.002;
    config.realtimePacing = true;

    const auto result =
        runHilMonitoredLoopback(
            telemetry,
            nullptr,
            std::stop_token{},
            config);

    ASSERT_EQ(
        result.completedSteps,
        8u);

    EXPECT_EQ(
        result.scheduling.samples,
        8u);

    EXPECT_GE(
        result.scheduling.p99StartLatenessMs,
        0.0);

    EXPECT_GE(
        result.scheduling.maxStartLatenessMs,
        result.scheduling.p99StartLatenessMs);

    // Windows scheduling varies. Do not assert that
    // deadline misses must be zero on every machine.
    EXPECT_LE(
        result.scheduling.completionDeadlineMisses,
        8u);
}

TEST(
    HilMonitoredRunTests,
    StreamsEntireRunToCsv)
{
    HilTelemetryStore telemetry;

    const auto path =
        std::filesystem::temp_directory_path() /
        "aeroforge_hil_scheduling_test_samples.csv";

    std::filesystem::remove(path);

    HilMonitoredRunConfig config{};

    config.realtimePacing = false;
    config.steps = 550;
    config.csvPath = path.string();

    const auto result =
        runHilMonitoredLoopback(
            telemetry,
            nullptr,
            std::stop_token{},
            config);

    EXPECT_EQ(
        result.completedSteps,
        550u);

    EXPECT_EQ(
        telemetry.snapshot().count,
        HilTelemetrySnapshot::CAPACITY);

    std::ifstream in(path);

    ASSERT_TRUE(in.good());

    std::size_t lines = 0;
    std::string line;

    ASSERT_TRUE(std::getline(in, line));

    EXPECT_NE(
        line.find("start_lateness_ms"),
        std::string::npos);

    ++lines;

    while (std::getline(in, line))
    {
        ++lines;
    }

    in.close();

    EXPECT_EQ(lines, 551u);

    std::filesystem::remove(path);
}