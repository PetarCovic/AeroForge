#pragma once

#include <cstddef>

namespace aeroforge::simulation
{
	struct RealtimeExecutionResult
	{
		double simulatedTimeS;
		double wallTimeS;
		double realtimeFactor;
		std::size_t stepCount;
		std::size_t deadlineMissCount;
		double maximumOverrunS;
	};
}