#include "GuiTheme.h"

#include <imgui.h>

namespace aeroforge::visualization::gui
{
    void applyAeroForgeTheme()
    {
        ImGui::StyleColorsDark();
        ImGuiStyle& style = ImGui::GetStyle();
        style.WindowRounding = 5.0f;
        style.ChildRounding = 4.0f;
        style.FrameRounding = 4.0f;
        style.GrabRounding = 3.0f;
        style.TabRounding = 4.0f;
        style.WindowPadding = ImVec2(12.0f, 10.0f);
        style.FramePadding = ImVec2(8.0f, 5.0f);
        style.ItemSpacing = ImVec2(8.0f, 7.0f);
        style.WindowBorderSize = 1.0f;

        ImVec4* colors = style.Colors;
        colors[ImGuiCol_WindowBg] = ImVec4(0.065f, 0.080f, 0.104f, 1.0f);
        colors[ImGuiCol_ChildBg] = ImVec4(0.075f, 0.090f, 0.120f, 1.0f);
        colors[ImGuiCol_PopupBg] = ImVec4(0.095f, 0.115f, 0.145f, 0.98f);
        colors[ImGuiCol_TitleBg] = ImVec4(0.080f, 0.105f, 0.135f, 1.0f);
        colors[ImGuiCol_TitleBgActive] = ImVec4(0.100f, 0.140f, 0.180f, 1.0f);
        colors[ImGuiCol_Header] = ImVec4(0.14f, 0.22f, 0.28f, 1.0f);
        colors[ImGuiCol_HeaderHovered] = ImVec4(0.20f, 0.33f, 0.40f, 1.0f);
        colors[ImGuiCol_HeaderActive] = ImVec4(0.15f, 0.36f, 0.44f, 1.0f);
        colors[ImGuiCol_Button] = ImVec4(0.13f, 0.29f, 0.37f, 1.0f);
        colors[ImGuiCol_ButtonHovered] = ImVec4(0.17f, 0.40f, 0.48f, 1.0f);
        colors[ImGuiCol_ButtonActive] = ImVec4(0.15f, 0.47f, 0.51f, 1.0f);
        colors[ImGuiCol_Tab] = ImVec4(0.12f, 0.17f, 0.22f, 1.0f);
        colors[ImGuiCol_TabHovered] = ImVec4(0.18f, 0.38f, 0.45f, 1.0f);
        colors[ImGuiCol_TabSelected] = ImVec4(0.12f, 0.33f, 0.40f, 1.0f);
        colors[ImGuiCol_CheckMark] = ImVec4(0.34f, 0.84f, 0.75f, 1.0f);
    }
}
