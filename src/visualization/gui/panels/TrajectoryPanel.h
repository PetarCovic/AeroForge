#pragma once
namespace aeroforge::visualization 
{
	class VisualizationSession;
}

namespace aeroforge::visualization::gui::panels
{
	void drawTrajectoryPanel(bool* open, const VisualizationSession& session);
}