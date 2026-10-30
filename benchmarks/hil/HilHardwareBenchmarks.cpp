#include <hil/HilHardwareRun.h>
#include <hil/HilTelemetryStore.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <cstdint>
#include <limits>

namespace
{
    std::wstring widenAscii(const std::string& str)
    {
        // COM identifiers contain ASCII; reject arbitrary path input.
        if (str.size() < 4 || str.size() > 8 || str.substr(0, 3) != "COM")
            throw std::invalid_argument("Port must be COMn (e.g. COM3)");
        for (std::size_t i = 3; i < str.size(); ++i)
            if (str[i] < '0' || str[i] > '9')
                throw std::invalid_argument("Port must be COMn (e.g. COM3)");
        return { str.begin(), str.end() };
    }
}

int main(int argc, char** argv)
{
    try
    {
        using namespace aeroforge::hil;
        if (argc > 5)
            throw std::invalid_argument(
                "Usage: AeroForgeHilHardwareBenchmarks [COM3] [steps] [outputDir] [baud]");

        HilHardwareRunConfig config{};
        if (argc > 1) config.portName = widenAscii(argv[1]);
        if (argc > 2) config.steps = std::stoull(argv[2]);
        const std::filesystem::path output =
            argc > 3 ? std::filesystem::path(argv[3]) :
            std::filesystem::path("out/hil-benchmarks/hardware");
        if (argc > 4) config.baudRate = static_cast<std::uint32_t>(std::stoul(argv[4]));
        validateHilHardwareRunConfig(config);
        std::filesystem::create_directories(output);
        const auto csv = output / "hil_hardware_samples.csv";
        const auto json = output / "hil_hardware_summary.json";
        config.csvPath = csv.string();

        HilTelemetryStore telemetry;
        const auto result = runHilHardwareClosedLoop(
            telemetry, nullptr, std::stop_token{}, config);
        const auto& h = result.finalHealth;
        const auto& s = result.scheduling;

        std::ofstream report(json, std::ios::trunc);
        if (!report) throw std::runtime_error("Cannot create hardware report");
        report << std::fixed << std::setprecision(8)
            << "{\n"
            << "  \"mode\": \"physical_esp32_uart\",\n"
            << "  \"port\": \"" << std::string(config.portName.begin(), config.portName.end()) << "\",\n"
            << "  \"baud_rate\": " << config.baudRate << ",\n"
            << "  \"step_period_ms\": " << config.stepS * 1000.0 << ",\n"
            << "  \"completed_steps\": " << result.completedSteps << ",\n"
            << "  \"wall_elapsed_s\": " << result.wallElapsedS << ",\n"
            << "  \"cancelled\": " << (result.cancelled ? "true" : "false") << ",\n"
            << "  \"serial_failed\": " << (result.serialFailed ? "true" : "false") << ",\n"
            << "  \"serial_last_error\": " << h.serialLastError << ",\n"
            << "  \"serial_rx_dropped_bytes\": " << h.serialRxDroppedBytes << ",\n"
            << "  \"accepted_commands\": " << h.acceptedCommands << ",\n"
            << "  \"rejected_commands\": " << h.rejectedCommands << ",\n"
            << "  \"queued_sensor_samples\": " << h.queuedSamples << ",\n"
            << "  \"dropped_sensor_samples\": " << h.droppedSamples << ",\n"
            << "  \"steps_with_valid_command\": " << result.stepsWithCommand << ",\n"
            << "  \"steps_using_zero_fallback\": " << result.stepsUsingZeroFallback << ",\n"
            << "  \"mean_step_execution_ms\": " << h.averageStepMs << ",\n"
            << "  \"max_step_execution_ms\": " << h.maximumStepMs << ",\n"
            << "  \"step_execution_deadline_misses\": " << h.deadlineMisses << ",\n"
            << "  \"mean_command_response_ms\": " << h.averageCommandLatencyMs << ",\n"
            << "  \"max_command_response_ms\": " << h.maximumCommandLatencyMs << ",\n"
            << "  \"response_latency_samples\": " << h.commandLatencyCount << ",\n"
            << "  \"mean_start_lateness_ms\": " << s.meanStartLatenessMs << ",\n"
            << "  \"p50_start_lateness_ms\": " << s.p50StartLatenessMs << ",\n"
            << "  \"p95_start_lateness_ms\": " << s.p95StartLatenessMs << ",\n"
            << "  \"p99_start_lateness_ms\": " << s.p99StartLatenessMs << ",\n"
            << "  \"max_start_lateness_ms\": " << s.maxStartLatenessMs << ",\n"
            << "  \"completion_deadline_misses\": " << s.completionDeadlineMisses << ",\n"
            << "  \"max_completion_overrun_ms\": " << s.maxCompletionOverrunMs << "\n"
            << "}\n";
        report.close();
        if (!report) throw std::runtime_error("Cannot finish hardware report");

        std::cout << "AeroForge PHYSICAL ESP32 HIL benchmark\n"
            << "Port: " << std::string(config.portName.begin(), config.portName.end())
            << "  Baud: " << config.baudRate << '\n'
            << "Steps: " << result.completedSteps << '/' << config.steps << '\n'
            << "Accepted controller responses: " << h.acceptedCommands << '\n'
            << "Steps with valid control: " << result.stepsWithCommand << '\n'
            << "Zero-command fallback steps: " << result.stepsUsingZeroFallback << '\n'
            << "Mean/max response: " << h.averageCommandLatencyMs
            << " / " << h.maximumCommandLatencyMs << " ms\n"
            << "Mean/max computation: " << h.averageStepMs
            << " / " << h.maximumStepMs << " ms\n"
            << "Start lateness mean/p95/p99/max: "
            << s.meanStartLatenessMs << " / "
            << s.p95StartLatenessMs << " / "
            << s.p99StartLatenessMs << " / "
            << s.maxStartLatenessMs << " ms\n"
            << "Execution/completion deadline misses: " << h.deadlineMisses
            << " / " << s.completionDeadlineMisses << '\n'
            << "RX dropped bytes: " << h.serialRxDroppedBytes
            << "  Last error: " << h.serialLastError << '\n'
            << "CSV: " << csv << '\n' << "JSON: " << json << '\n';

        // A preliminary engineering threshold, not a real-time guarantee.
        // Require a response for at least 90% of the planned sensor steps
        // and a healthy control command in at least 90% of physics steps.
        const auto threshold = (config.steps * 9u + 9u) / 10u;
        const bool passed =
            !result.cancelled && !result.serialFailed &&
            result.completedSteps == config.steps &&
            h.acceptedCommands >= threshold &&
            result.stepsWithCommand >= threshold &&
            h.serialRxDroppedBytes == 0 &&
            h.serialLastError == 0;
        std::cout << (passed ? "HARDWARE ACCEPTANCE THRESHOLD MET\n" :
            "HARDWARE ACCEPTANCE THRESHOLD NOT MET\n");
        // Interpret individual timing and fault metrics before declaring
        // the complete physical HIL system validated.
        return passed ? 0 : 1;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Hardware HIL benchmark error: " << e.what() << '\n';
        return 1;
    }
}