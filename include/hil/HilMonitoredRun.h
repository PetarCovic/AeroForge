#pragma once

#include <hil/HilTelemetryStore.h>

#include <cstddef>
#include <cstdint>
#include <span>
#include <stop_token>
#include <string>

namespace aeroforge::visualization
{
    class LiveTelemetryPublisher;
}

namespace aeroforge::hil
{
    struct HilMonitoredRunConfig
    {
        double stepS = 0.01;
        std::size_t steps = 1000;
        bool realtimePacing = true;

        // Optional CSV output. Written after the timed run.
        std::string csvPath;
    };

    struct HilSchedulingStatistics
    {
        std::uint64_t samples = 0;

        double meanStartLatenessMs = 0.0;
        double p50StartLatenessMs = 0.0;
        double p95StartLatenessMs = 0.0;
        double p99StartLatenessMs = 0.0;
        double maxStartLatenessMs = 0.0;

        std::uint64_t completionDeadlineMisses = 0;
        double maxCompletionOverrunMs = 0.0;
    };

    // Uses nearest-rank percentiles.
    // Input samples must be finite and nonnegative.
    HilSchedulingStatistics summarizeScheduling(
        std::span<const double> startLatenessMs,
        std::uint64_t completionDeadlineMisses,
        double maxCompletionOverrunMs);

    struct HilMonitoredRunResult
    {
        bool cancelled = false;
        std::size_t completedSteps = 0;
        double wallElapsedS = 0.0;

        HilHealthSnapshot finalHealth{};
        HilSchedulingStatistics scheduling{};
    };

    // RK4 6DOF dynamics, packet encoding/decoding,
    // and software controller execution.
    //
    // No physical controller is required.
    //
    // Scheduling statistics are collected only when
    // realtimePacing is enabled.
    HilMonitoredRunResult runHilMonitoredLoopback(
        HilTelemetryStore& telemetry,
        visualization::LiveTelemetryPublisher* visualizationPublisher,
        std::stop_token stop,
        const HilMonitoredRunConfig& config = {});
}
