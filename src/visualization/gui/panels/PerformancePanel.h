#pragma once
namespace aeroforge::visualization 
{ 
    class VisualizationSession;

    class FrameTimingStats; 
}

namespace aeroforge::visualization::gui::panels
{
    void drawPerformancePanel(bool* open, const VisualizationSession& session, const FrameTimingStats& stats);
}
