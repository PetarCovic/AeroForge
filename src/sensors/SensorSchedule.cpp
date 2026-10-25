#include <sensors/SensorSchedule.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::sensors
{
	SensorSchedule::SensorSchedule(double sampleRateHz, double startTimeS)
		: sampleRateHz(sampleRateHz), nextSampleTimeS(startTimeS)
	{
		if (!std::isfinite(sampleRateHz) || sampleRateHz <= 0)
		{
			throw std::invalid_argument("sampleRateHz must be finite and positive");
		}

		if (!std::isfinite(startTimeS) || startTimeS < 0)
		{
			throw std::invalid_argument("startTimeS must be finite and non-negative");
		}

		samplePeriodS = 1.0 / sampleRateHz;
	}

	double SensorSchedule::getSamplePeriodS() const
	{
		return samplePeriodS;
	}

	bool SensorSchedule::shouldSample(double timeS)
	{
		if (!std::isfinite(timeS) || timeS < 0)
		{
			throw std::invalid_argument("timeS must be finite and non-negative");
		}

		if (timeS < nextSampleTimeS)
		{
			return false;
		}

		while (nextSampleTimeS <= timeS)
		{
			nextSampleTimeS += samplePeriodS;
		}

		return true;
	}
}