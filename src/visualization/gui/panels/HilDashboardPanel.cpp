#include "HilDashboardPanel.h"
#include <hil/HilTelemetryStore.h>
#include <imgui.h>
#include <implot.h>
#include <array>
#include <cstddef>
#include <cstdio>

namespace aeroforge::visualization::gui::panels
{
    namespace
    {
        const char* modeLabel(hil::HilLinkMode value)
        {
            switch (value)
            {
            case hil::HilLinkMode::SOFTWARE_LOOPBACK: return "Software loopback (no hardware)";
            case hil::HilLinkMode::SERIAL: return "Physical serial (experimental)";
            default: return "No source";
            }
        }
        const char* watchdogLabel(hil::HilWatchdogState value)
        {
            switch (value)
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
        void plot(const char* title, const double* time, const double* value,
            std::size_t count, const char* unit, double deadline = -1.0)
        {
            if (count == 0) return;
            if (!ImPlot::BeginPlot(title, ImVec2(-1, 170))) return;
            ImPlot::SetupAxes("Simulation time [s]", unit);
            ImPlot::PlotLine("Measured", time, value, static_cast<int>(count));
            if (deadline >= 0.0 && count > 1)
            {
                const double x[2] = { time[0], time[count - 1] };
                const double y[2] = { deadline, deadline };
                ImPlot::PlotLine("Step period", x, y, 2);
            }
            ImPlot::EndPlot();
        }
    }

    void drawHilDashboardPanel(bool* open, const hil::HilTelemetryStore* telemetry)
    {
        if (!ImGui::Begin("HIL Monitor", open)) { ImGui::End(); return; }
        if (!telemetry)
        {
            ImGui::TextWrapped("No HIL data. Use HIL > Start software loopback, or configure a physical serial controller.");
            ImGui::End();
            return;
        }
        // The store creates one thread-safe snapshot; GUI never blocks the HIL worker.
        const auto snapshot = telemetry->snapshot();
        const auto& h = snapshot.health;
        ImGui::Text("Mode: %s", modeLabel(h.mode));
        ImGui::SameLine();
        ImGui::TextDisabled("(%s)", h.running ? "running" : "stopped");
        const bool healthy = h.watchdogState == hil::HilWatchdogState::HEALTHY;
        ImGui::Text("Controller watchdog:");
        ImGui::SameLine();
        ImGui::TextColored(healthy ? ImVec4(0.34f, 0.84f, 0.75f, 1) :
            ImVec4(1.0f, 0.66f, 0.38f, 1), "%s", watchdogLabel(h.watchdogState));
        if (!healthy)
            ImGui::TextWrapped("Controller authority is NOT confirmed healthy. Invalid or missing commands must not be used.");
        if (h.mode == hil::HilLinkMode::SERIAL)
        {
            ImGui::Text("Serial worker: %s", serialLabel(h.serialState));
            ImGui::Text("Dropped RX bytes: %llu", static_cast<unsigned long long>(h.serialRxDroppedBytes));
            ImGui::Text("Last serial error: 0x%08X", static_cast<unsigned int>(h.serialLastError));
            ImGui::TextDisabled("USB power reconnection without physical RST is not validated on the tested ESP32.");
        }
        ImGui::SeparatorText("Control & scheduling");
        if (ImGui::BeginTable("##hilStats", 2, ImGuiTableFlags_BordersInnerH))
        {
            const auto row = [](const char* title, const char* value)
                {
                    ImGui::TableNextRow(); ImGui::TableNextColumn();
                    ImGui::TextUnformatted(title); ImGui::TableNextColumn();
                    ImGui::TextUnformatted(value);
                };
            char buf[128];
            std::snprintf(buf, sizeof(buf), "%.3f ms", h.targetPeriodMs);
            row("Step period", buf);
            std::snprintf(buf, sizeof(buf), "%llu", static_cast<unsigned long long>(h.stepCount));
            row("Measured steps", buf);
            std::snprintf(buf, sizeof(buf), "%.3f / %.3f ms", h.averageStepMs, h.maximumStepMs);
            row("Execution mean / max", buf);
            std::snprintf(buf, sizeof(buf), "%llu", static_cast<unsigned long long>(h.deadlineMisses));
            row("Execution deadline misses", buf);
            std::snprintf(buf, sizeof(buf), "%.3f / %.3f ms", h.averageCommandLatencyMs,
                h.maximumCommandLatencyMs);
            row("Command latency mean / max", buf);
            std::snprintf(buf, sizeof(buf), "%llu", static_cast<unsigned long long>(h.commandLatencyCount));
            row("Command latency observations", buf);
            std::snprintf(buf, sizeof(buf), "%llu / %llu",
                static_cast<unsigned long long>(h.acceptedCommands),
                static_cast<unsigned long long>(h.rejectedCommands));
            row("Commands accepted / rejected", buf);
            std::snprintf(buf, sizeof(buf), "%llu / %llu",
                static_cast<unsigned long long>(h.queuedSamples),
                static_cast<unsigned long long>(h.droppedSamples));
            row("Sensor samples queued / dropped", buf);
            ImGui::EndTable();
        }
        ImGui::TextDisabled("Times shown here are execution/response durations, not USB one-way latency or scheduler jitter.");
        if (snapshot.count == 0)
        {
            ImGui::TextDisabled("Waiting for HIL measurements...");
            ImGui::End();
            return;
        }

        constexpr std::size_t C = hil::HilTelemetrySnapshot::CAPACITY;
        std::array<double, C> time{};
        std::array<double, C> step{};
        std::array<double, C> comm{};
        std::array<double, C> latency{};
        std::array<double, C> latencyTime{};
        std::size_t validLatency = 0;
        for (std::size_t i = 0; i < snapshot.count; ++i)
        {
            const auto& p = snapshot.points[i];
            time[i] = p.simulationTimeS;
            step[i] = p.stepExecutionMs;
            comm[i] = p.communicationMs;
            if (p.commandLatencyMs >= 0.0)
            {
                latencyTime[validLatency] = p.simulationTimeS;
                latency[validLatency] = p.commandLatencyMs;
                ++validLatency;
            }
        }
        ImGui::SeparatorText("Recent HIL traces");
        plot("Simulation execution", time.data(), step.data(),
            snapshot.count, "Execution [ms]", h.targetPeriodMs);
        plot("Communication processing", time.data(), comm.data(),
            snapshot.count, "Processing [ms]");
        plot("Command response latency", latencyTime.data(), latency.data(),
            validLatency, "Latency [ms]");
        ImGui::End();
    }
}
