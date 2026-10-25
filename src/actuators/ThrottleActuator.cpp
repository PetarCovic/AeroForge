#include <actuators/ThrottleActuator.h>

#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace aeroforge::actuators
{
	ThrottleActuator::ThrottleActuator(double initialThrottle, double maxRatePerS)
		: throttle(initialThrottle), maxRatePerS(maxRatePerS)
	{
		if (!std::isfinite(initialThrottle) || initialThrottle < 0 || initialThrottle>1)
		{
			throw std::invalid_argument("initialThrottle must be finite and in [0, 1]");
		}

		if (!std::isfinite(maxRatePerS) || maxRatePerS <= 0)
		{
			throw std::invalid_argument("maxRatePerS must be finite and positive");
		}
	}

	double ThrottleActuator::update(double commandedThrottle, double dtS)
	{
		if (!std::isfinite(commandedThrottle))
		{
			throw std::invalid_argument("commandedThrottle must be finite");
		}

		if (!std::isfinite(dtS) || dtS < 0)
		{
			throw std::invalid_argument("dtS must be finite and non-negative");
		}

		double clampedCommand = std::clamp(commandedThrottle, 0.0, 1.0);

		double error = clampedCommand - throttle;

		double maxDelta = maxRatePerS * dtS;

		double limitDelta = std::clamp(error, -maxDelta, maxDelta);

		throttle += limitDelta;

		throttle = std::clamp(throttle, 0.0, 1.0);

		if (!std::isfinite(throttle))
		{
			throw std::overflow_error("throttle must be finite");
		}

		return throttle;
	}

	double ThrottleActuator::getThrottle() const
	{
		return throttle;
	}
}