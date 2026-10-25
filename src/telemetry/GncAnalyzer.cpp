#include <telemetry/GncAnalyzer.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aeroforge::telemetry
{
	GncMetrics GncAnalyzer::analyze(const std::vector<TelemetrySample>& samples, double settlingThresholdRad)
	{
		if (samples.empty())
		{
			throw std::invalid_argument("samples cannot be empty");
		}

		if (!std::isfinite(settlingThresholdRad) || settlingThresholdRad<=0)
		{
			throw std::invalid_argument("settlingThresholdRad must be finite and positive");
		}

		GncMetrics metric;
		metric.sampleCount = samples.size();
		
		double squaredErrorSum = 0.0;
		double commandedMomentSum = 0.0;

		metric.peakAttitudeErrorRad = 0.0;
		metric.maxCommandedMomentNm = 0.0;

		std::size_t saturatedCount = 0;

		for (const TelemetrySample& sample : samples)
		{
			double attitudeErrorMagnitude = sample.attitudeErrorB.magnitude();

			squaredErrorSum +=attitudeErrorMagnitude * attitudeErrorMagnitude;

			metric.peakAttitudeErrorRad = std::max(metric.peakAttitudeErrorRad, attitudeErrorMagnitude);


			double commandedMomentMagnitude =sample.commandedMomentB.magnitude();

			commandedMomentSum +=commandedMomentMagnitude;

			metric.maxCommandedMomentNm = std::max(metric.maxCommandedMomentNm, commandedMomentMagnitude);

			if (sample.controllerSaturated)
			{
				saturatedCount++;
			}
		}

		metric.rmsAttitudeErrorRad = std::sqrt(squaredErrorSum / static_cast<double>(samples.size()));

		metric.meanCommandedMomentNm = commandedMomentSum / static_cast<double>(samples.size());

		metric.saturationFraction = static_cast<double>(saturatedCount) / static_cast<double>(samples.size());

		metric.settlingTimeS = std::nullopt;

		if (samples.back().attitudeErrorB.magnitude() <= settlingThresholdRad)
		{
			std::size_t settlingIndex = samples.size() - 1;

			while (settlingIndex > 0)
			{
				double previousError = samples[settlingIndex - 1].attitudeErrorB.magnitude();

				if (previousError > settlingThresholdRad)
				{
					break;
				}

				settlingIndex--;
			}

			metric.settlingTimeS = samples[settlingIndex].timeS;
		}

		return metric;
	}
}