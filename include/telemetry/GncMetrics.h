#pragma once

#include <cstddef>
#include <optional>

namespace aeroforge::telemetry
{
	struct GncMetrics
	{
		std::size_t sampleCount;
		double rmsAttitudeErrorRad;
		double peakAttitudeErrorRad;
		double meanCommandedMomentNm;
		double maxCommandedMomentNm;
		double saturationFraction;
		std::optional<double> settlingTimeS;
	};
}