#include "Vehicle3DPanel.h"
#include "../rendering/VehicleRenderer.h"
#include <visualization/VisualizationSession.h>
#include <visualization/Camera3D.h>
#include <imgui.h>
#include <algorithm>

namespace aeroforge::visualization::gui::panels
{
    void drawVehicle3DPanel(bool* open, const VisualizationSession& session, Camera3D& camera)
    {
        if (!ImGui::Begin("Vehicle 3D", open)) { ImGui::End(); return; }
        ImGui::TextDisabled("Drag: orbit  |  Scroll: zoom  |  Render: wireframe attitude");
        ImGui::SameLine();
        if (ImGui::SmallButton("Reset camera")) camera = Camera3D{};
        const auto* s = session.current();
        if (!s)
        {
            ImGui::Separator();
            ImGui::TextWrapped("No vehicle snapshot available. Start a simulation or open a telemetry recording.");
            ImGui::End();
            return;
        }
        ImVec2 size = ImGui::GetContentRegionAvail();
        size.x = std::max(size.x, 100.0f);
        size.y = std::max(size.y, 100.0f);
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton("VehicleCanvas", size, ImGuiButtonFlags_MouseButtonLeft);
        if (ImGui::IsItemHovered())
        {
            auto& io = ImGui::GetIO();
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left))
                camera.orbit(io.MouseDelta.x * 0.007, io.MouseDelta.y * 0.007);
            if (io.MouseWheel != 0) camera.zoom(io.MouseWheel);
        }
        ImDrawList* draw = ImGui::GetWindowDrawList();
        const ImVec2 end(origin.x + size.x, origin.y + size.y);
        draw->AddRectFilled(origin, end, IM_COL32(12, 20, 30, 255));
        draw->PushClipRect(origin, end, true);
        rendering::drawVehicle(draw, origin, size, camera, *s);
        draw->AddText(ImVec2(origin.x + 10, origin.y + 10),
            IM_COL32(135, 185, 196, 255), "NED vehicle attitude | not to scale");
        draw->PopClipRect();
        draw->AddRect(origin, end, IM_COL32(48, 76, 87, 255));
        ImGui::End();
    }
}
