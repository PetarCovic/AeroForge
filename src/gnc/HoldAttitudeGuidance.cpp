#include <gnc/HoldAttitudeGuidance.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::gnc
{
	HoldAttitudeGuidance::HoldAttitudeGuidance(const AttitudeSetpoint& setpoint)
		: setpoint(setpoint)
	{
	}

	AttitudeSetpoint HoldAttitudeGuidance::getSetpoint(double timeS) const
	{
		if (!std::isfinite(timeS) || timeS < 0)
		{
			throw std::invalid_argument("timeS must be finite and non-negative");
		}

		return setpoint;
	}
}