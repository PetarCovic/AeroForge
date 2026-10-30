#include "PlaybackPanel.h"
#include <visualization/VisualizationSession.h>
#include <imgui.h>
#include <algorithm>

namespace aeroforge::visualization::gui::panels
{
    void drawPlaybackPanel(bool* open, VisualizationSession& session)
    {
        if (!ImGui::Begin("Playback", open)) { ImGui::End(); return; }
        const bool recorded = session.mode() == VisualizationSession::Mode::Recorded;
        if (!recorded)
        {
            ImGui::TextWrapped("Playback controls are for recorded telemetry. Live sources run independently of this timeline.");
            ImGui::TextDisabled("File > Open telemetry CSV... to load a recording.");
            ImGui::End();
            return;
        }

        const bool playing = session.isPlaying();
        if (ImGui::Button(playing ? "Pause" : "Play"))
        {
            if (playing) session.pause(); else session.play();
        }
        ImGui::SameLine();
        if (ImGui::Button("Restart")) session.restart();
        ImGui::SameLine();
        ImGui::TextColored(playing ? ImVec4(0.34f, 0.84f, 0.75f, 1) :
            ImVec4(0.75f, 0.78f, 0.84f, 1), playing ? "PLAYING" : "PAUSED");

        const double start = session.startTimeS();
        const double end = session.endTimeS();
        double current = session.currentTimeS();
        if (end > start)
        {
            ImGui::SetNextItemWidth(-1);
            if (ImGui::SliderScalar("##playbackTime", ImGuiDataType_Double,
                &current, &start, &end, "%.3f s")) session.seek(current);
            const float fraction = static_cast<float>(std::clamp(
                (session.currentTimeS() - start) / (end - start), 0.0, 1.0));
            ImGui::ProgressBar(fraction, ImVec2(-1, 4), "");
        }
        else
        {
            ImGui::TextDisabled("Recording contains a single timestamp.");
        }

        static constexpr double speeds[] = { 0.25, 0.5, 1.0, 2.0, 5.0, 10.0 };
        static constexpr const char* labels[] = { "0.25x", "0.5x", "1x", "2x", "5x", "10x" };
        const double rate = session.playbackRate();
        int selected = 2;
        for (int i = 0; i < 6; ++i) if (rate == speeds[i]) selected = i;
        if (ImGui::BeginCombo("Speed", labels[selected]))
        {
            for (int i = 0; i < 6; ++i)
            {
                const bool chosen = rate == speeds[i];
                if (ImGui::Selectable(labels[i], chosen)) session.setPlaybackRate(speeds[i]);
                if (chosen) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        ImGui::TextDisabled("%.3f / %.3f seconds", session.currentTimeS(), end);
        ImGui::End();
    }
}
