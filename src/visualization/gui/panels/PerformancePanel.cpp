#include "PerformancePanel.h"
#include <visualization/VisualizationSession.h>
#include <visualization/FrameTimingStats.h>
#include <imgui.h>
namespace aeroforge::visualization::gui::panels
{
    void drawPerformancePanel(bool* open, const VisualizationSession& session,
        const FrameTimingStats& stats)
    {
        if (!ImGui::Begin("Performance", open)) { ImGui::End();return; }
        ImGui::TextUnformatted("GUI frame interval incl. VSync/polling; NOT GPU render time");
        ImGui::Separator();
        const auto s = stats.summary();
        ImGui::Text("Window samples: %zu", s.count);
        ImGui::Text("Mean frame: %.3f ms; mean-based FPS: %.1f", s.meanMs, s.estimatedFps);
        ImGui::Text("P50: %.3f ms", s.p50Ms);
        ImGui::Text("P95: %.3f ms", s.p95Ms);
        ImGui::Text("P99: %.3f ms", s.p99Ms);
        ImGui::Text("Frames > 16.667 ms: %.1f%%", s.overBudgetPercent);
        ImGui::Text("Visual samples retained: %zu", session.samples().size());
        ImGui::TextDisabled("Simulator runtime factor: not connected.");
        ImGui::TextDisabled("GPU frame time: not measured.");
        ImGui::TextDisabled("Publisher drop counts: not instrumented.");
        ImGui::End();
    }
}
