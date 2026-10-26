#pragma once

namespace aeroforge::visualization 
{ 
	class VisualizationSession; 
}

namespace aeroforge::visualization::gui::panels
{
	void drawEventsPanel(bool* open, const VisualizationSession& session);
}
