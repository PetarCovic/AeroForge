#pragma once

namespace aeroforge::visualization::gui
{
    struct PanelVisibility
    {
        bool dashboard = true;
        bool playback = true;
        bool telemetry = true;
        bool trajectory = true;
        bool vehicle3D = true;
        bool events = true;
        bool performance = true;
        bool hilMonitor = true;
    };
}
