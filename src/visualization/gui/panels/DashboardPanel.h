#pragma once
namespace aeroforge::visualization 
{ 
	class VisualizationSession; 
}

namespace aeroforge::visualization::gui::panels
{
	void drawDashboardPanel(bool* open, const VisualizationSession& session);
}