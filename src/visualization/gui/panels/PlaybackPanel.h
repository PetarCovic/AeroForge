#pragma once
namespace aeroforge::visualization 
{ 
	class VisualizationSession; 
}

namespace aeroforge::visualization::gui::panels
{
	void drawPlaybackPanel(bool* open, VisualizationSession& session);
}