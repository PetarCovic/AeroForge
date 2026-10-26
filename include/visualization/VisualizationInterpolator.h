#pragma once

#include <visualization/VisualizationSnapshot.h>

namespace aeroforge::visualization
{
	class VisualizationInterpolator
	{
	public:
		static VisualizationSnapshot interpolate(
			const VisualizationSnapshot& first,
			const VisualizationSnapshot& second,
			double simulationTimeS);
	};
}