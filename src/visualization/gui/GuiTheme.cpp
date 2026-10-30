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
        style.PopupRounding = 4.0f;
        style.GrabRounding = 3.0f;
        style.TabRounding = 4.0f;
        style.ScrollbarRounding = 5.0f;
        style.WindowPadding = ImVec2(12.0f, 10.0f);
        style.FramePadding = ImVec2(8.0f, 5.0f);
        style.ItemSpacing = ImVec2(8.0f, 7.0f);
        style.ItemInnerSpacing = ImVec2(6.0f, 5.0f);
        style.WindowBorderSize = 1.0f;
        style.FrameBorderSize = 0.0f;
        style.SeparatorTextBorderSize = 1.0f;

        ImVec4* c = style.Colors;
        c[ImGuiCol_Text] = ImVec4(0.91f, 0.94f, 0.97f, 1.00f);
        c[ImGuiCol_TextDisabled] = ImVec4(0.60f, 0.68f, 0.75f, 1.00f);
        c[ImGuiCol_WindowBg] = ImVec4(0.065f, 0.080f, 0.104f, 1.00f);
        c[ImGuiCol_ChildBg] = ImVec4(0.075f, 0.090f, 0.120f, 1.00f);
        c[ImGuiCol_PopupBg] = ImVec4(0.095f, 0.115f, 0.145f, 0.99f);
        c[ImGuiCol_Border] = ImVec4(0.20f, 0.28f, 0.34f, 0.80f);
        c[ImGuiCol_FrameBg] = ImVec4(0.105f, 0.140f, 0.185f, 1.00f);
        c[ImGuiCol_FrameBgHovered] = ImVec4(0.145f, 0.220f, 0.280f, 1.00f);
        c[ImGuiCol_FrameBgActive] = ImVec4(0.155f, 0.265f, 0.325f, 1.00f);
        c[ImGuiCol_TitleBg] = ImVec4(0.080f, 0.105f, 0.135f, 1.00f);
        c[ImGuiCol_TitleBgActive] = ImVec4(0.100f, 0.140f, 0.180f, 1.00f);
        c[ImGuiCol_MenuBarBg] = ImVec4(0.080f, 0.105f, 0.135f, 1.00f);
        c[ImGuiCol_Header] = ImVec4(0.14f, 0.22f, 0.28f, 1.00f);
        c[ImGuiCol_HeaderHovered] = ImVec4(0.20f, 0.33f, 0.40f, 1.00f);
        c[ImGuiCol_HeaderActive] = ImVec4(0.15f, 0.36f, 0.44f, 1.00f);
        c[ImGuiCol_Button] = ImVec4(0.13f, 0.29f, 0.37f, 1.00f);
        c[ImGuiCol_ButtonHovered] = ImVec4(0.17f, 0.40f, 0.48f, 1.00f);
        c[ImGuiCol_ButtonActive] = ImVec4(0.15f, 0.47f, 0.51f, 1.00f);
        c[ImGuiCol_Tab] = ImVec4(0.12f, 0.17f, 0.22f, 1.00f);
        c[ImGuiCol_TabHovered] = ImVec4(0.18f, 0.38f, 0.45f, 1.00f);
        c[ImGuiCol_TabSelected] = ImVec4(0.12f, 0.33f, 0.40f, 1.00f);
        c[ImGuiCol_CheckMark] = ImVec4(0.34f, 0.84f, 0.75f, 1.00f);
        c[ImGuiCol_SliderGrab] = ImVec4(0.28f, 0.70f, 0.72f, 1.00f);
        c[ImGuiCol_SliderGrabActive] = ImVec4(0.38f, 0.86f, 0.76f, 1.00f);
        c[ImGuiCol_Separator] = ImVec4(0.22f, 0.31f, 0.36f, 0.75f);
        c[ImGuiCol_ResizeGrip] = ImVec4(0.25f, 0.55f, 0.62f, 0.25f);
        c[ImGuiCol_ResizeGripHovered] = ImVec4(0.25f, 0.68f, 0.72f, 0.60f);
        c[ImGuiCol_ResizeGripActive] = ImVec4(0.28f, 0.75f, 0.76f, 0.90f);
        c[ImGuiCol_PlotLines] = ImVec4(0.32f, 0.78f, 0.78f, 1.00f);
        c[ImGuiCol_NavHighlight] = ImVec4(0.34f, 0.84f, 0.75f, 1.00f);
        c[ImGuiCol_DockingPreview] = ImVec4(0.22f, 0.67f, 0.70f, 0.65f);
    }
}
