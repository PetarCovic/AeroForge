#pragma once

#include <cstddef>
#include <string>

namespace aeroforge::performance
{
	struct RealtimeBenchmarkResult
	{
		std::string name;

		double simulatedTimeS;

		double wallTimeS;

		double realtimeFactor;

		std::size_t stepCount;

		std::size_t deadlineMissCount;

		double maximumOverrunS;

		bool hasStabilityMetrics = false;

		double firstWindowMeanStepTimeS = 0.0;
		double finalWindowMeanStepTimeS = 0.0;
		double timingDegradationFraction = 0.0;
	};
}