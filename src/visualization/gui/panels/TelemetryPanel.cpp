
#include "TelemetryPanel.h"

#include <visualization/VisualizationSession.h>
#include <visualization/PlotDataBuilder.h>

#include <imgui.h>
#include <implot.h>

#include <vector>
#include <cstddef>

namespace aeroforge::visualization::gui::panels
{
    void drawTelemetryPanel(
        bool* open,
        const VisualizationSession& session)
    {
        if (!ImGui::Begin("Telemetry", open))
        {
            ImGui::End();
            return;
        }

        const auto& samples = session.samples();

        if (samples.empty())
        {
            ImGui::TextDisabled(
                "Load a recording or connect live telemetry."
            );

            ImGui::End();
            return;
        }

        static int selectedField = 2;

        const char* labels[] =
        {
            "North position [m]",
            "East position [m]",
            "Down position [m]",
            "North velocity [m/s]",
            "East velocity [m/s]",
            "Down velocity [m/s]",
            "Attitude error X [rad]",
            "Attitude error Y [rad]",
            "Attitude error Z [rad]",
            "Moment X [Nm]",
            "Moment Y [Nm]",
            "Moment Z [Nm]"
        };

        ImGui::Combo(
            "Channel",
            &selectedField,
            labels,
            12
        );

        struct PlotCache
        {
            std::size_t revision =
                static_cast<std::size_t>(-1);

            int field = -1;

            std::vector<double> t;
            std::vector<double> a;
            std::vector<double> b;
        };

        static PlotCache cache;

        // Rebuild the plotted data only when the
        // recording or selected channel changes.
        if (cache.revision != session.revision() ||
            cache.field != selectedField)
        {
            const auto indices =
                PlotDataBuilder::decimate(
                    samples,
                    2000,
                    selectedField
                );

            cache.t.clear();
            cache.a.clear();
            cache.b.clear();

            cache.t.reserve(indices.size());
            cache.a.reserve(indices.size());
            cache.b.reserve(indices.size());

            for (const auto i : indices)
            {
                cache.t.push_back(
                    samples[i].simulationTimeS
                );

                cache.a.push_back(
                    PlotDataBuilder::component(
                        samples[i],
                        selectedField,
                        false
                    )
                );

                cache.b.push_back(
                    PlotDataBuilder::component(
                        samples[i],
                        selectedField,
                        true
                    )
                );
            }

            cache.revision = session.revision();
            cache.field = selectedField;
        }

        const auto& t = cache.t;
        const auto& a = cache.a;
        const auto& b = cache.b;

        // Read the current playback time every frame.
        // Do not cache it with the telemetry series.
        const double playbackTime = session.currentTimeS();

        if (ImPlot::BeginPlot(
            "Time series",
            ImVec2(-1, -1)))
        {
            ImPlot::SetupAxes(
                "Simulation time [s]",
                labels[selectedField]
            );

            // Primary telemetry series.
            const char* primaryLabel =
                (selectedField >= 6 && selectedField <= 8)
                ? "Error"
                : (selectedField >= 9)
                ? "Command"
                : "Truth";

            ImPlot::PlotLine(
                primaryLabel,
                t.data(),
                a.data(),
                static_cast<int>(t.size())
            );

            // Secondary series is only applicable to
            // position, velocity, and moment channels.
            if (selectedField < 6 || selectedField >= 9)
            {
                const char* secondaryLabel =
                    selectedField >= 9
                    ? "Actual"
                    : "Estimate";

                ImPlot::PlotLine(
                    secondaryLabel,
                    t.data(),
                    b.data(),
                    static_cast<int>(t.size())
                );
            }

            // Vertical line at the current playback time.
            ImPlot::PlotInfLines(
                "Playback Position",
                &playbackTime,
                1
            );

            // Highlight the current value.
            if (const auto* snapshot = session.current())
            {
                const double currentValue =
                    PlotDataBuilder::component(
                        *snapshot,
                        selectedField,
                        false
                    );

                ImPlot::PlotScatter(
                    "Current Value",
                    &playbackTime,
                    &currentValue,
                    1
                );
            }

            ImPlot::EndPlot();
        }

        ImGui::End();
    }
}
