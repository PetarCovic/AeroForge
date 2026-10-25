#pragma once

#include <telemetry/GncMetrics.h>
#include <telemetry/TelemetrySample.h>

#include <vector>

namespace aeroforge::telemetry
{
	class GncAnalyzer
	{
	public:
		static GncMetrics analyze(const std::vector<TelemetrySample>& samples, double settlingThresholdRad);
	};
}