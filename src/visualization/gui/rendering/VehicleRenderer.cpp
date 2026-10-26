#include "VehicleRenderer.h"
#include <array>
#include <cmath>
namespace aeroforge::visualization::gui::rendering
{
    namespace
    {
        bool drawSegment(ImDrawList* list, ImVec2 offset, ImVec2 size, const Camera3D& c,
            const math::Vector3& a, const math::Vector3& b, ImU32 color, float width = 1.5f)
        {
            auto p = c.project(a, size.x, size.y), q = c.project(b, size.x, size.y);
            if (!p.visible || !q.visible)return false;
            list->AddLine(ImVec2(offset.x + static_cast<float>(p.x), offset.y + static_cast<float>(p.y)),
                ImVec2(offset.x + static_cast<float>(q.x), offset.y + static_cast<float>(q.y)),
                color, width);
            return true;
        }
    }
    void drawVehicle(ImDrawList* list, ImVec2 origin, ImVec2 size,
        const Camera3D& camera, const VisualizationSnapshot& snapshot)
    {
        // Navigation frame: N=+x (red), E=+y (green), D=+z (blue).
        drawSegment(list, origin, size, camera, { 0,0,0 }, { 4,0,0 }, IM_COL32(250, 105, 105, 255), 2);
        drawSegment(list, origin, size, camera, { 0,0,0 }, { 0,4,0 }, IM_COL32(100, 230, 125, 255), 2);
        drawSegment(list, origin, size, camera, { 0,0,0 }, { 0,0,4 }, IM_COL32(100, 160, 250, 255), 2);
        // Rocket nose: +X in body FRD; tail: -X. Display geometry is illustrative.
        constexpr int sides = 12;
        const math::Vector3 nose(3, 0, 0);
        const math::Quaternion q = snapshot.truthAttitudeNB;
        for (int i = 0;i < sides;++i)
        {
            constexpr double pi = 3.14159265358979323846;
            const double a = 2 * pi * i / sides;
            const double b = 2 * pi * (i + 1) / sides;
            const math::Vector3 front(1.8, 0.55 * std::cos(a), 0.55 * std::sin(a));
            const math::Vector3 frontNext(1.8, 0.55 * std::cos(b), 0.55 * std::sin(b));
            const math::Vector3 rear(-2.5, 0.55 * std::cos(a), 0.55 * std::sin(a));
            const math::Vector3 rearNext(-2.5, 0.55 * std::cos(b), 0.55 * std::sin(b));
            const auto blue = IM_COL32(200, 220, 240, 255);
            drawSegment(list, origin, size, camera, q.rotate(front), q.rotate(frontNext), blue);
            drawSegment(list, origin, size, camera, q.rotate(rear), q.rotate(rearNext), blue);
            drawSegment(list, origin, size, camera, q.rotate(front), q.rotate(rear), blue);
            drawSegment(list, origin, size, camera, q.rotate(front), q.rotate(nose), IM_COL32(240, 180, 100, 255));
        }
        // Body x-axis (FRD forward) emphasizes attitude.
        drawSegment(list, origin, size, camera, { 0,0,0 }, q.rotate({ 4,0,0 }),
            IM_COL32(250, 245, 120, 255), 3);
        list->AddText(ImVec2(origin.x + 10, origin.y + 10), IM_COL32(235, 235, 235, 255),
            "N (red), E (green), D (blue), body forward (yellow)");
    }
}