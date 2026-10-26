#include "DashboardPanel.h"
#include <visualization/VisualizationSession.h>
#include <imgui.h>
#include <cmath>
namespace aeroforge::visualization::gui::panels
{
    void drawDashboardPanel(bool* open, const VisualizationSession& session)
    {
        if (!ImGui::Begin("Dashboard", open)) { ImGui::End();return; }
        ImGui::TextUnformatted("AEROFORGE | Engineering Visualization");
        ImGui::Separator();
        const char* mode = "None";
        if (session.mode() == VisualizationSession::Mode::Recorded)mode = "Recorded playback";
        if (session.mode() == VisualizationSession::Mode::Live)mode = "Live publisher";
        ImGui::Text("Source: %s", mode);
        ImGui::TextWrapped("%s", session.sourceName().c_str());
        const auto* s = session.current();
        if (!s)
        {
            ImGui::TextDisabled("No simulation snapshot available.");ImGui::End();return;
        }
        ImGui::Text("Time: %.3f s", s->simulationTimeS);
        ImGui::Separator();
        ImGui::Text("Truth NED position [m]: N %.3f  E %.3f  D %.3f",
            s->truthPositionN.x, s->truthPositionN.y, s->truthPositionN.z);
        ImGui::Text("Altitude relative to NED origin: %.3f m", -s->truthPositionN.z);
        ImGui::Text("Truth speed: %.3f m/s", s->truthVelocityN.magnitude());
        ImGui::Text("Position residual norm: %.3f m",
            (s->estimatedPositionN - s->truthPositionN).magnitude());
        ImGui::Text("q_NB [w,x,y,z]: %.4f %.4f %.4f %.4f",
            s->truthAttitudeNB.w, s->truthAttitudeNB.x, s->truthAttitudeNB.y, s->truthAttitudeNB.z);
        if (session.mode() == VisualizationSession::Mode::Live)
            ImGui::TextDisabled("Live source is synthetic unless connected to actual simulation output.");
        ImGui::End();
    }
}
