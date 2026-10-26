
#include "TrajectoryPanel.h"

#include <visualization/VisualizationSession.h>

#include <imgui.h>
#include <implot.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace aeroforge::visualization::gui::panels
{
    void drawTrajectoryPanel(
        bool* open,
        const VisualizationSession& session)
    {
        if (!ImGui::Begin("Trajectory", open))
        {
            ImGui::End();
            return;
        }

        const auto& samples = session.samples();

        if (samples.empty())
        {
            ImGui::TextDisabled(
                "No trajectory available. Load telemetry first."
            );

            ImGui::End();
            return;
        }

        struct TrajectoryCache
        {
            std::size_t revision =
                static_cast<std::size_t>(-1);

            std::vector<double> east;
            std::vector<double> north;
            std::vector<double> eastEst;
            std::vector<double> northEst;
            std::vector<double> time;
            std::vector<double> altitude;

            bool hasHorizontalMotion = false;
        };

        static TrajectoryCache cache;

        if (cache.revision != session.revision())
        {
            cache.east.clear();
            cache.north.clear();
            cache.eastEst.clear();
            cache.northEst.clear();
            cache.time.clear();
            cache.altitude.clear();

            const std::size_t stride =
                std::max<std::size_t>(
                    1,
                    (samples.size() + 1999) / 2000
                );

            for (std::size_t i = 0;
                i < samples.size();
                i += stride)
            {
                const auto& s = samples[i];

                cache.east.push_back(
                    s.truthPositionN.y
                );

                cache.north.push_back(
                    s.truthPositionN.x
                );

                cache.eastEst.push_back(
                    s.estimatedPositionN.y
                );

                cache.northEst.push_back(
                    s.estimatedPositionN.x
                );

                cache.time.push_back(
                    s.simulationTimeS
                );

                cache.altitude.push_back(
                    -s.truthPositionN.z
                );
            }

            const auto [eastMin, eastMax] =
                std::minmax_element(
                    cache.east.begin(),
                    cache.east.end()
                );

            const auto [northMin, northMax] =
                std::minmax_element(
                    cache.north.begin(),
                    cache.north.end()
                );

            cache.hasHorizontalMotion =
                (*eastMax - *eastMin > 1e-6) ||
                (*northMax - *northMin > 1e-6);

            cache.revision = session.revision();
        }

        const auto* current = session.current();

        ImGui::Text(
            "Simulation time: %.2f s",
            session.currentTimeS()
        );

        ImGui::Separator();

        // Ground track: only show if the recording
        // actually contains horizontal displacement.
        if (cache.hasHorizontalMotion)
        {
            if (ImPlot::BeginPlot(
                "NED ground track",
                ImVec2(-1, 260)))
            {
                ImPlot::SetupAxes(
                    "East [m]",
                    "North [m]",
                    ImPlotAxisFlags_AutoFit,
                    ImPlotAxisFlags_AutoFit
                );

                ImPlot::PlotLine(
                    "Truth",
                    cache.east.data(),
                    cache.north.data(),
                    static_cast<int>(
                        cache.east.size()
                        )
                );

                ImPlot::PlotLine(
                    "Estimate",
                    cache.eastEst.data(),
                    cache.northEst.data(),
                    static_cast<int>(
                        cache.eastEst.size()
                        )
                );

                if (current)
                {
                    const double x =
                        current->truthPositionN.y;

                    const double y =
                        current->truthPositionN.x;

                    ImPlot::PlotScatter(
                        "Current Position",
                        &x,
                        &y,
                        1
                    );
                }

                ImPlot::EndPlot();
            }
        }
        else
        {
            ImGui::TextDisabled(
                "Horizontal trajectory unavailable."
            );

            ImGui::TextWrapped(
                "This recording contains no varying "
                "North/East coordinates. Showing the "
                "vertical flight profile instead."
            );

            ImGui::Separator();
        }

        // Altitude is displayed for both full 3D
        // recordings and vertical-only recordings.
        if (ImPlot::BeginPlot(
            "Altitude vs. Time",
            ImVec2(-1, -1)))
        {
            ImPlot::SetupAxes(
                "Simulation time [s]",
                "Altitude relative to NED origin [m]",
                ImPlotAxisFlags_AutoFit,
                ImPlotAxisFlags_AutoFit
            );

            ImPlot::PlotLine(
                "Altitude",
                cache.time.data(),
                cache.altitude.data(),
                static_cast<int>(
                    cache.time.size()
                    )
            );

            const double playbackTime =
                session.currentTimeS();

            ImPlot::PlotInfLines(
                "Playback Position",
                &playbackTime,
                1
            );

            if (current)
            {
                const double altitude =
                    -current->truthPositionN.z;

                ImPlot::PlotScatter(
                    "Current Altitude",
                    &playbackTime,
                    &altitude,
                    1
                );
            }

            ImPlot::EndPlot();
        }

        ImGui::End();
    }
}
