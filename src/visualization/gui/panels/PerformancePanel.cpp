#include "PerformancePanel.h"
#include <visualization/VisualizationSession.h>
#include <visualization/FrameTimingStats.h>
#include <imgui.h>

namespace aeroforge::visualization::gui::panels
{
    void drawPerformancePanel(bool* open, const VisualizationSession& session,
        const FrameTimingStats& stats)
    {
        if (!ImGui::Begin("Performance", open)) { ImGui::End(); return; }
        ImGui::TextUnformatted("GUI responsiveness");
        ImGui::TextDisabled("Frame interval includes VSync and event polling; not GPU time.");
        ImGui::Separator();
        const auto s = stats.summary();
        ImGui::Text("Mean-based FPS: %.1f", s.estimatedFps);
        ImGui::Text("Mean interval: %.3f ms", s.meanMs);
        if (ImGui::BeginTable("##frameStats", 2, ImGuiTableFlags_BordersInnerH))
        {
            const auto row = [](const char* label, double value)
                {
                    ImGui::TableNextRow(); ImGui::TableNextColumn();
                    ImGui::TextUnformatted(label); ImGui::TableNextColumn();
                    ImGui::Text("%.3f ms", value);
                };
            row("P50", s.p50Ms);
            row("P95", s.p95Ms);
            row("P99", s.p99Ms);
            ImGui::EndTable();
        }
        ImGui::Text("Frames over 16.667 ms: %.1f%%", s.overBudgetPercent);
        ImGui::Text("Frame samples in window: %zu", s.count);
        ImGui::Text("Telemetry samples retained: %zu", session.samples().size());
        ImGui::Separator();
        ImGui::TextDisabled("Simulator runtime factor: not connected.");
        ImGui::TextDisabled("GPU frame time: not measured.");
        ImGui::TextDisabled("Publisher drop counts: not instrumented.");
        ImGui::End();
    }
}
