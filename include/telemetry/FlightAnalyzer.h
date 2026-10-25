#pragma once

#include <telemetry/FlightMetrics.h>
#include <telemetry/TelemetrySample.h>

#include <vector>

namespace aeroforge::telemetry
{
	class FlightAnalyzer
	{
	public:
		static FlightMetrics analyze(const std::vector<TelemetrySample>& samples);
	};
}