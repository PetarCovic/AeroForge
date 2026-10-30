#include "EventsPanel.h"
#include <visualization/VisualizationSession.h>
#include <imgui.h>
#include <string>

namespace aeroforge::visualization::gui::panels
{
    void drawEventsPanel(bool* open, const VisualizationSession& session)
    {
        if (!ImGui::Begin("Events", open)) { ImGui::End(); return; }
        const auto& events = session.events();
        ImGui::Text("Event annotations: %zu", events.size());
        ImGui::TextDisabled("File > Open event CSV... to load annotations.");
        if (events.empty())
        {
            ImGui::Separator();
            ImGui::TextDisabled("No events loaded.");
            ImGui::End();
            return;
        }

        static char filter[128]{};
        static bool pastOnly = false;
        ImGui::SetNextItemWidth(-1);
        ImGui::InputTextWithHint("##eventFilter", "Filter type or message...", filter, sizeof(filter));
        ImGui::Checkbox("Only events at or before current time", &pastOnly);
        ImGui::Separator();
        const double t = session.currentTimeS();
        ImGui::BeginChild("##eventList", ImVec2(0, 0), false);
        std::size_t matched = 0;
        for (std::size_t i = 0; i < events.size(); ++i)
        {
            const auto& e = events[i];
            const bool past = e.simulationTimeS <= t;
            if (pastOnly && !past) continue;
            if (filter[0] != '\0' &&
                std::string(e.type + " " + e.message).find(filter) == std::string::npos) continue;
            ++matched;
            ImGui::PushID(static_cast<int>(i));
            ImGui::TextColored(past ? ImVec4(0.34f, 0.84f, 0.75f, 1) :
                ImVec4(0.65f, 0.70f, 0.76f, 1), "%.3f s  [%s]", e.simulationTimeS, e.type.c_str());
            ImGui::TextWrapped("%s", e.message.c_str());
            ImGui::Separator();
            ImGui::PopID();
        }
        if (matched == 0) ImGui::TextDisabled("No events match the current filter.");
        ImGui::EndChild();
        ImGui::End();
    }
}
