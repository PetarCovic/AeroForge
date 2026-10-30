#pragma once

#include <hil/HilMonitoredRun.h>
#include <hil/HilTelemetryStore.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stop_token>
#include <string>

namespace aeroforge::visualization
{
    class LiveTelemetryPublisher;
}

namespace aeroforge::hil
{
    struct HilHardwareRunConfig
    {
        std::wstring portName = L"COM3";
        std::uint32_t baudRate = 115200;
        double stepS = 0.01;
        std::size_t steps = 10000;
        std::chrono::milliseconds startupDelay{ 1500 };
        std::chrono::milliseconds commandTimeout{ 100 };
        std::string csvPath;
    };

    struct HilHardwareRunResult
    {
        bool cancelled = false;
        bool serialFailed = false;
        std::size_t completedSteps = 0;
        std::uint64_t stepsWithCommand = 0;
        std::uint64_t stepsUsingZeroFallback = 0;
        double wallElapsedS = 0.0;
        HilHealthSnapshot finalHealth{};
        HilSchedulingStatistics scheduling{};
    };

    // Pure validation; may be tested without a connected controller.
    void validateHilHardwareRunConfig(const HilHardwareRunConfig& config);

    // Hardware run: the ESP32 computes control commands via COM port.
    // Each physics step uses the most recent valid, fresh command.
    // No step blocks waiting for a controller response.
    HilHardwareRunResult runHilHardwareClosedLoop(
        HilTelemetryStore& telemetry,
        visualization::LiveTelemetryPublisher* visualPublisher,
        std::stop_token stop,
        const HilHardwareRunConfig& config = {});
}
