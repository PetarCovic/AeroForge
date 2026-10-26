#pragma once
namespace aeroforge::visualization 
{ 
	class VisualizationSession;
	
	class Camera3D; 
}

namespace aeroforge::visualization::gui::panels
{
	void drawVehicle3DPanel(bool* open, const VisualizationSession& session, Camera3D& camera);
}