#include <hil/HilMonitoredRun.h>
#include <hil/HilTelemetryStore.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <stop_token>
#include <string>

int main(int argc, char** argv)
{
    try
    {
        using namespace aeroforge::hil;

        std::size_t steps = 10000;

        std::filesystem::path output =
            "out/hil-benchmarks";

        bool paced = false;

        if (argc >= 2)
        {
            steps = std::stoull(argv[1]);
        }

        if (argc >= 3)
        {
            output = argv[2];
        }

        if (argc >= 4)
        {
            if (std::string(argv[3]) != "paced")
            {
                throw std::invalid_argument(
                    "Third argument must be 'paced'");
            }

            paced = true;
        }

        if (steps == 0)
        {
            throw std::invalid_argument(
                "Steps must be positive");
        }

        std::filesystem::create_directories(output);

        const auto csvPath =
            output / "hil_samples.csv";

        const auto jsonPath =
            output / "hil_summary.json";

        HilMonitoredRunConfig config{};

        config.stepS = 0.01;
        config.steps = steps;
        config.realtimePacing = paced;
        config.csvPath = csvPath.string();

        HilTelemetryStore telemetry;

        const auto result =
            runHilMonitoredLoopback(
                telemetry,
                nullptr,
                std::stop_token{},
                config);

        const auto& health =
            result.finalHealth;

        const auto& scheduling =
            result.scheduling;

        // ----------------------------------------------
        // JSON performance report
        // ----------------------------------------------

        std::ofstream json(jsonPath);

        if (!json)
        {
            throw std::runtime_error(
                "Cannot open benchmark JSON");
        }

        json << std::fixed << std::setprecision(9);

        json
            << "{\n"

            << "  \"mode\": "
            "\"software_loopback_ideal_imu\",\n"

            << "  \"hardware_connected\": false,\n"

            << "  \"realtime_pacing\": "
            << (paced ? "true" : "false") << ",\n"

            << "  \"step_period_ms\": "
            << health.targetPeriodMs << ",\n"

            << "  \"completed_steps\": "
            << result.completedSteps << ",\n"

            << "  \"wall_elapsed_s\": "
            << result.wallElapsedS << ",\n"

            << "  \"mean_step_execution_ms\": "
            << health.averageStepMs << ",\n"

            << "  \"max_step_execution_ms\": "
            << health.maximumStepMs << ",\n"

            << "  \"step_execution_deadline_misses\": "
            << health.deadlineMisses << ",\n"

            << "  \"accepted_commands\": "
            << health.acceptedCommands << ",\n"

            << "  \"rejected_commands\": "
            << health.rejectedCommands << ",\n"

            << "  \"dropped_sensor_samples\": "
            << health.droppedSamples << ",\n"

            << "  \"mean_command_response_ms\": "
            << health.averageCommandLatencyMs << ",\n"

            << "  \"max_command_response_ms\": "
            << health.maximumCommandLatencyMs << ",\n"

            << "  \"scheduling_measured\": "
            << (paced ? "true" : "false") << ",\n"

            << "  \"scheduling_samples\": "
            << scheduling.samples << ",\n"

            << "  \"mean_start_lateness_ms\": "
            << scheduling.meanStartLatenessMs << ",\n"

            << "  \"p50_start_lateness_ms\": "
            << scheduling.p50StartLatenessMs << ",\n"

            << "  \"p95_start_lateness_ms\": "
            << scheduling.p95StartLatenessMs << ",\n"

            << "  \"p99_start_lateness_ms\": "
            << scheduling.p99StartLatenessMs << ",\n"

            << "  \"max_start_lateness_ms\": "
            << scheduling.maxStartLatenessMs << ",\n"

            << "  \"completion_deadline_misses\": "
            << scheduling.completionDeadlineMisses << ",\n"

            << "  \"max_completion_overrun_ms\": "
            << scheduling.maxCompletionOverrunMs << "\n"

            << "}\n";

        json.close();

        if (!json)
        {
            throw std::runtime_error(
                "Cannot write benchmark JSON");
        }

        // ----------------------------------------------
        // Console report
        // ----------------------------------------------

        std::cout
            << "HIL SOFTWARE LOOPBACK benchmark\n"

            << "Steps: "
            << result.completedSteps << '\n'

            << "Step mean/max: "
            << health.averageStepMs << " / "
            << health.maximumStepMs << " ms\n"

            << "Execution deadline misses: "
            << health.deadlineMisses << '\n'

            << "Accepted commands: "
            << health.acceptedCommands << '\n'

            << "Mean command response: "
            << health.averageCommandLatencyMs
            << " ms\n";

        if (paced)
        {
            std::cout
                << "Start lateness mean/p95/p99/max: "
                << scheduling.meanStartLatenessMs << " / "
                << scheduling.p95StartLatenessMs << " / "
                << scheduling.p99StartLatenessMs << " / "
                << scheduling.maxStartLatenessMs
                << " ms\n"

                << "Completion deadline misses: "
                << scheduling.completionDeadlineMisses
                << '\n'

                << "Maximum completion overrun: "
                << scheduling.maxCompletionOverrunMs
                << " ms\n";
        }
        else
        {
            std::cout
                << "Scheduling jitter: "
                "not measured (unpaced run)\n";
        }

        std::cout
            << "CSV: " << csvPath << '\n'
            << "JSON: " << jsonPath << '\n';

        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "HIL benchmark error: "
            << e.what() << '\n';

        return 1;
    }
}
