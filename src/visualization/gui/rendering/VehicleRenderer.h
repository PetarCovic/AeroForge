#pragma once
#include <visualization/Camera3D.h>
#include <visualization/VisualizationSnapshot.h>
#include <imgui.h>

namespace aeroforge::visualization::gui::rendering
{
    // Projects a 3D wireframe rocket using the simulation's q_NB.
    // Draw-list rendering uses ImGui's OpenGL backend (no custom GL loader).
    void drawVehicle(ImDrawList* list, ImVec2 origin, ImVec2 size, const Camera3D& camera, const VisualizationSnapshot& snapshot);
}