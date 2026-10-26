#include "Vehicle3DPanel.h"
#include "../rendering/VehicleRenderer.h"
#include <visualization/VisualizationSession.h>
#include <visualization/Camera3D.h>
#include <imgui.h>
namespace aeroforge::visualization::gui::panels
{
    void drawVehicle3DPanel(bool* open, const VisualizationSession& session, Camera3D& camera)
    {
        if (!ImGui::Begin("Vehicle 3D", open)) { ImGui::End();return; }
        ImGui::TextUnformatted("Wireframe 3D attitude: drag to orbit, scroll to zoom");
        const auto* s = session.current();
        if (!s) { ImGui::TextDisabled("No vehicle snapshot loaded.");ImGui::End();return; }
        ImVec2 size = ImGui::GetContentRegionAvail();
        if (size.y < 100)size.y = 100;
        if (size.x < 100)size.x = 100;
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton("VehicleCanvas", size, ImGuiButtonFlags_MouseButtonLeft);
        const bool hovered = ImGui::IsItemHovered();
        if (hovered)
        {
            auto& io = ImGui::GetIO();
            if (ImGui::IsMouseDragging(ImGuiMouseButton_Left))
                camera.orbit(io.MouseDelta.x * 0.007, io.MouseDelta.y * 0.007);
            if (io.MouseWheel != 0) camera.zoom(io.MouseWheel);
        }
        ImDrawList* draw = ImGui::GetWindowDrawList();
        draw->AddRectFilled(origin, ImVec2(origin.x + size.x, origin.y + size.y),
            IM_COL32(12, 20, 30, 255));
        draw->PushClipRect(origin, ImVec2(origin.x + size.x, origin.y + size.y), true);
        rendering::drawVehicle(draw, origin, size, camera, *s);
        draw->PopClipRect();
        ImGui::End();
    }
}
