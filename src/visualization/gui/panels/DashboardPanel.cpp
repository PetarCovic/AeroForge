#include "DashboardPanel.h"
#include <visualization/VisualizationSession.h>
#include <imgui.h>

namespace aeroforge::visualization::gui::panels
{
    namespace
    {
        void metric(const char* name, const char* format, double value)
        {
            ImGui::TableNextRow();
            ImGui::TableNextColumn();
            ImGui::TextDisabled("%s", name);
            ImGui::TableNextColumn();
            ImGui::Text(format, value);
        }
    }

    void drawDashboardPanel(bool* open, const VisualizationSession& session)
    {
        if (!ImGui::Begin("Dashboard", open)) { ImGui::End(); return; }
        ImGui::TextColored(ImVec4(0.34f, 0.84f, 0.75f, 1),
            "AEROFORGE / FLIGHT DYNAMICS");
        ImGui::Separator();
        const char* mode = "No source";
        if (session.mode() == VisualizationSession::Mode::Recorded) mode = "Recorded telemetry";
        if (session.mode() == VisualizationSession::Mode::Live) mode = "Live telemetry";
        ImGui::Text("Feed: %s", mode);
        if (!session.sourceName().empty())
        {
            ImGui::TextDisabled("Source name:");
            ImGui::TextWrapped("%s", session.sourceName().c_str());
        }
        ImGui::TextDisabled("Coordinate frame: NED (North / East / Down)");

        const auto* s = session.current();
        if (!s)
        {
            ImGui::Separator();
            ImGui::TextWrapped("No data yet. Use Simulation > Start 6DOF simulation, or load a telemetry CSV from File.");
            ImGui::End();
            return;
        }
        ImGui::SeparatorText("Current state");
        if (ImGui::BeginTable("##flightMetrics", 2,
            ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_SizingStretchProp))
        {
            metric("Simulation time [s]", "%.3f", s->simulationTimeS);
            metric("North [m]", "%.3f", s->truthPositionN.x);
            metric("East [m]", "%.3f", s->truthPositionN.y);
            metric("Down [m]", "%.3f", s->truthPositionN.z);
            metric("Altitude above NED origin [m]", "%.3f", -s->truthPositionN.z);
            metric("Truth speed [m/s]", "%.3f", s->truthVelocityN.magnitude());
            metric("Position residual [m]", "%.4f",
                (s->estimatedPositionN - s->truthPositionN).magnitude());
            ImGui::EndTable();
        }
        ImGui::SeparatorText("Orientation");
        ImGui::TextWrapped("q_NB [w, x, y, z]: %.4f, %.4f, %.4f, %.4f",
            s->truthAttitudeNB.w, s->truthAttitudeNB.x,
            s->truthAttitudeNB.y, s->truthAttitudeNB.z);
        ImGui::TextDisabled("Source identity and run state appear in the bottom status bar.");
        ImGui::End();
    }
}
