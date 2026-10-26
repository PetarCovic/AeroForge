#include "EventsPanel.h"
#include <visualization/VisualizationSession.h>
#include <imgui.h>
namespace aeroforge::visualization::gui::panels
{
    void drawEventsPanel(bool* open, const VisualizationSession& session)
    {
        if (!ImGui::Begin("Events", open)) { ImGui::End();return; }
        ImGui::TextUnformatted("Event annotations (separate CSV)");
        ImGui::Separator();
        if (session.events().empty()) ImGui::TextDisabled("No events loaded.");
        for (const auto& e : session.events())
        {
            const bool active = e.simulationTimeS <= session.currentTimeS();
            ImGui::TextColored(active ? ImVec4(0.25f, 0.85f, 0.7f, 1) : ImVec4(0.65f, 0.65f, 0.65f, 1),
                "%.3f s  [%s]", e.simulationTimeS, e.type.c_str());
            ImGui::SameLine();ImGui::TextWrapped("%s", e.message.c_str());
        }
        ImGui::End();
    }
}
