#include "PlaybackPanel.h"
#include <visualization/VisualizationSession.h>
#include <imgui.h>
namespace aeroforge::visualization::gui::panels
{
    void drawPlaybackPanel(bool* open, VisualizationSession& session)
    {
        if (!ImGui::Begin("Playback", open)) { ImGui::End();return; }
        const bool recorded = session.mode() == VisualizationSession::Mode::Recorded;
        ImGui::TextUnformatted(recorded ? "Recorded timeline" : "Playback requires a recording");
        ImGui::BeginDisabled(!recorded);
        if (ImGui::Button("Play"))session.play();ImGui::SameLine();
        if (ImGui::Button("Pause"))session.pause();ImGui::SameLine();
        if (ImGui::Button("Restart"))session.restart();
        double time = session.currentTimeS();
        const double start = session.startTimeS();
        const double end = session.endTimeS();
        if (ImGui::SliderScalar("Time [s]", ImGuiDataType_Double, &time,
            &start, &end))
            session.seek(time);
        static constexpr double speeds[] = { 0.25,0.5,1.0,2.0,5.0,10.0 };
        const auto display = [](double v)->const char* {
            if (v == 0.25)return "0.25x";if (v == 0.5)return "0.5x";
            if (v == 1)return "1x";if (v == 2)return "2x";
            if (v == 5)return "5x";return "10x";
            };
        if (ImGui::BeginCombo("Speed", display(session.playbackRate())))
        {
            for (const double speed : speeds)
                if (ImGui::Selectable(display(speed), session.playbackRate() == speed))
                    session.setPlaybackRate(speed);
            ImGui::EndCombo();
        }
        ImGui::Text("%.3f / %.3f s", session.currentTimeS(), session.endTimeS());
        ImGui::TextUnformatted(session.isPlaying() ? "Playing" : "Paused");
        ImGui::EndDisabled();
        ImGui::End();
    }
}
