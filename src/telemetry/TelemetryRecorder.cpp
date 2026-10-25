#include <telemetry/TelemetryRecorder.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::telemetry
{
	void TelemetryRecorder::record(const TelemetrySample& sample)
	{
		if (!std::isfinite(sample.timeS) || sample.timeS<0)
		{
			throw std::invalid_argument("timeS must be finite and non-negative");
		}

		if (!sample.positionN.isFinite())
		{
			throw std::invalid_argument("sample.positionN must be finite");
		}

		if (!sample.velocityN.isFinite())
		{
			throw std::invalid_argument("sample.velocityN must be finite");
		}

		if (!sample.attitudeNB.isFinite())
		{
			throw std::invalid_argument("sample.attitudeNB must be finite");
		}

		if (!sample.angularVelocityB.isFinite())
		{
			throw std::invalid_argument("sample.angularVelocityB must be finite");
		}

		if (!sample.estimatedPositionN.isFinite())
		{
			throw std::invalid_argument("sample.estimatedPositionN must be finite");
		}

		if (!sample.estimatedVelocityN.isFinite())
		{
			throw std::invalid_argument("sample.estimatedVelocityN must be finite");
		}

		if (!sample.estimatedAttitudeNB.isFinite())
		{
			throw std::invalid_argument("sample.estimatedAttitudeNB must be finite");
		}

		if (!sample.estimatedAngularVelocityB.isFinite())
		{
			throw std::invalid_argument("sample.estimatedAngularVelocityB must be finite");
		}

		if (!sample.attitudeErrorB.isFinite())
		{
			throw std::invalid_argument("sample.attitudeErrorB must be finite");
		}

		if (!sample.commandedMomentB.isFinite())
		{
			throw std::invalid_argument("sample.commandedMomentB must be finite");
		}

		if (!sample.actualMomentB.isFinite())
		{
			throw std::invalid_argument("sample.actualMomentB must be finite");
		}

		if (!std::isfinite(sample.throttle))
		{
			throw std::invalid_argument("throttle must be finite");
		}

		if (!std::isfinite(sample.gimbalPitchRad))
		{
			throw std::invalid_argument("gimbalPitchRad must be finite");
		}

		if (!std::isfinite(sample.gimbalYawRad))
		{
			throw std::invalid_argument("gimbalYawRad must be finite");
		}

		if (!samples.empty())
		{
			if (sample.timeS<=samples.back().timeS)
			{
				throw std::invalid_argument("sample.timeS must be after the previous sample's timeS");
			}
		}

		samples.push_back(sample);
	}

	const std::vector<TelemetrySample>& TelemetryRecorder::getSamples() const
	{
		return samples;
	}

	std::size_t TelemetryRecorder::size() const
	{
		return samples.size();
	}

	bool TelemetryRecorder::empty() const
	{
		return samples.empty();
	}

	void TelemetryRecorder::clear()
	{
		samples.clear();
	}
}