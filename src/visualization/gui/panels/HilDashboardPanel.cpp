#include "HilDashboardPanel.h"

#include <hil/HilTelemetryStore.h>

#include <imgui.h>
#include <implot.h>

#include <array>
#include <cstddef>

namespace aeroforge::visualization::gui::panels
{
    namespace
    {
        const char* linkLabel(hil::HilLinkMode mode)
        {
            switch (mode)
            {
            case hil::HilLinkMode::SOFTWARE_LOOPBACK: return "SOFTWARE LOOPBACK (no hardware)";
            case hil::HilLinkMode::SERIAL: return "HARDWARE SERIAL";
            default: return "NO SOURCE";
            }
        }

        const char* watchdogLabel(hil::HilWatchdogState state)
        {
            switch (state)
            {
            case hil::HilWatchdogState::WAITING: return "WAITING";
            case hil::HilWatchdogState::HEALTHY: return "HEALTHY";
            case hil::HilWatchdogState::TIMED_OUT: return "TIMED OUT";
            case hil::HilWatchdogState::DISCONNECTED: return "DISCONNECTED";
            case hil::HilWatchdogState::FAULTED: return "FAULTED";
            }
            return "UNKNOWN";
        }

        const char* serialLabel(hil::HilSerialWorkerState state)
        {
            switch (state)
            {
            case hil::HilSerialWorkerState::STOPPED: return "STOPPED";
            case hil::HilSerialWorkerState::STARTING: return "STARTING";
            case hil::HilSerialWorkerState::RUNNING: return "RUNNING";
            case hil::HilSerialWorkerState::FAILED: return "FAILED";
            }
            return "UNKNOWN";
        }

        void plot(const char* title, const double* t, const double* y,
            std::size_t count, const char* yAxis,
            double deadline = -1.0)
        {
            if (count == 0) return;
            if (!ImPlot::BeginPlot(title, ImVec2(-1, 165))) return;
            ImPlot::SetupAxes("Simulation time [s]", yAxis);
            ImPlot::PlotLine("Measured", t, y, static_cast<int>(count));
            if (deadline >= 0.0 && count > 1)
            {
                const double dx[2] = { t[0], t[count - 1] };
                const double dy[2] = { deadline, deadline };
                ImPlot::PlotLine("Step deadline", dx, dy, 2);
            }
            ImPlot::EndPlot();
        }
    }

    void drawHilDashboardPanel(
        bool* open,
        const hil::HilTelemetryStore* telemetry)
    {
        if (!ImGui::Begin("HIL Monitor", open))
        {
            ImGui::End();
            return;
        }

        if (!telemetry)
        {
            ImGui::TextDisabled("No HIL run attached. Use File > Start HIL software loopback.");
            ImGui::End();
            return;
        }

        // One locked copy per GUI frame; no interaction with the HIL thread.
        const auto snapshot = telemetry->snapshot();
        const auto& h = snapshot.health;
        ImGui::Text("Source: %s", linkLabel(h.mode));
        ImGui::SameLine();
        ImGui::TextDisabled("(%s)", h.running ? "running" : "stopped");
        ImGui::Text("Watchdog: %s", watchdogLabel(h.watchdogState));
        if (h.mode == hil::HilLinkMode::SERIAL)
        {
            ImGui::Text("Serial worker: %s", serialLabel(h.serialState));
            ImGui::Text("Serial RX dropped bytes: %llu  Last error: 0x%08X",
                static_cast<unsigned long long>(h.serialRxDroppedBytes),
                static_cast<unsigned int>(h.serialLastError));
        }
        if (h.watchdogState != hil::HilWatchdogState::HEALTHY)
            ImGui::TextDisabled("Command authority is not confirmed healthy.");
        ImGui::Separator();

        ImGui::Text("Step period: %.3f ms | Measured steps: %llu",
            h.targetPeriodMs, static_cast<unsigned long long>(h.stepCount));
        ImGui::Text("Step execution mean / max: %.3f / %.3f ms",
            h.averageStepMs, h.maximumStepMs);
        ImGui::Text("Step execution deadline misses: %llu",
            static_cast<unsigned long long>(h.deadlineMisses));
        ImGui::Text("Command latency mean / max: %.3f / %.3f ms (%llu measurements)",
            h.averageCommandLatencyMs, h.maximumCommandLatencyMs,
            static_cast<unsigned long long>(h.commandLatencyCount));
        ImGui::Text("Sensor queued / dropped: %llu / %llu",
            static_cast<unsigned long long>(h.queuedSamples),
            static_cast<unsigned long long>(h.droppedSamples));
        ImGui::Text("Control accepted / rejected: %llu / %llu",
            static_cast<unsigned long long>(h.acceptedCommands),
            static_cast<unsigned long long>(h.rejectedCommands));
        ImGui::TextDisabled("These are execution durations, not scheduler jitter or USB one-way latency.");

        if (snapshot.count == 0)
        {
            ImGui::TextDisabled("Waiting for HIL telemetry samples...");
            ImGui::End();
            return;
        }

        constexpr std::size_t C = hil::HilTelemetrySnapshot::CAPACITY;
        std::array<double, C> t{};
        std::array<double, C> step{};
        std::array<double, C> comm{};
        std::array<double, C> latency{};
        std::array<double, C> latencyTime{};
        std::size_t validLatency = 0;

        for (std::size_t i = 0; i < snapshot.count; ++i)
        {
            const auto& p = snapshot.points[i];
            t[i] = p.simulationTimeS;
            step[i] = p.stepExecutionMs;
            comm[i] = p.communicationMs;
            if (p.commandLatencyMs >= 0.0)
            {
                latencyTime[validLatency] = p.simulationTimeS;
                latency[validLatency] = p.commandLatencyMs;
                ++validLatency;
            }
        }

        plot("Simulation execution", t.data(), step.data(),
            snapshot.count, "Execution time [ms]", h.targetPeriodMs);
        plot("HIL communication processing", t.data(), comm.data(),
            snapshot.count, "Processing time [ms]");
        plot("Command response latency", latencyTime.data(), latency.data(),
            validLatency, "Latency [ms]");
        ImGui::End();
    }
}
