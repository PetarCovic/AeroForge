#pragma once
namespace aeroforge::visualization 
{ 
	class VisualizationSession; 
}

namespace aeroforge::visualization::gui::panels
{
	void drawTelemetryPanel(bool* open, const VisualizationSession& session);
}
