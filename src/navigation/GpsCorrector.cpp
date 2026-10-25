#include <navigation/GpsCorrector.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::navigation
{
	GpsCorrector::GpsCorrector(double positionGain, double velocityGain)
		: positionGain(positionGain), velocityGain(velocityGain)
	{
		if (!std::isfinite(positionGain) || positionGain < 0 || positionGain>1)
		{
			throw std::invalid_argument("positionGain must be finite and in [0, 1]");
		}

		if (!std::isfinite(velocityGain) || velocityGain < 0 || velocityGain>1)
		{
			throw std::invalid_argument("velocityGain must be finite and in [0, 1]");
		}
	}

	NavigationState GpsCorrector::correct(
		const NavigationState& state,
		const sensors::GpsMeasurement& gpsMeasurement
	) const
	{
		if (!std::isfinite(gpsMeasurement.timeS) || gpsMeasurement.timeS<0)
		{
			throw std::invalid_argument("gpsMeasurement.timeS must be finite and non-negative");
		}

		if (!gpsMeasurement.positionN.isFinite())
		{
			throw std::invalid_argument("gpsMeasurement.positionN must be finite");
		}

		if (!gpsMeasurement.velocityN.isFinite())
		{
			throw std::invalid_argument("gpsMeasurement.velocityN must be finite");
		}

		math::Vector3 correctedPositionN = state.getPositionN() + positionGain * (gpsMeasurement.positionN - state.getPositionN());
		math::Vector3 correctedVelocityN = state.getVelocityN() + velocityGain * (gpsMeasurement.velocityN - state.getVelocityN());

		return NavigationState{ correctedPositionN, correctedVelocityN, state.getAttitudeNB(), state.getAngularVelocityB()};
	}
}