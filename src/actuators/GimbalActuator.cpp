#include <actuators/GimbalActuator.h>

#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace aeroforge::actuators
{
	GimbalActuator::GimbalActuator(double maxAngleRad, double maxRateRadPerS)
		: maxAngleRad(maxAngleRad), maxRateRadPerS(maxRateRadPerS)
	{
		if (!std::isfinite(maxAngleRad) || maxAngleRad <= 0)
		{
			throw std::invalid_argument("maxAngleRad must be finite and positive");
		}

		if (!std::isfinite(maxRateRadPerS) || maxRateRadPerS <= 0)
		{
			throw std::invalid_argument("maxRateRadPerS must be finite and positive");
		}

		angles.pitchRad = 0;
		angles.yawRad = 0;
	}

	GimbalAngles GimbalActuator::update(double commandedPitchRad, double commandedYawRad, double dtS)
	{
		if (!std::isfinite(commandedPitchRad))
		{
			throw std::invalid_argument("commandedPitchRad must be finite");
		}

		if (!std::isfinite(commandedYawRad))
		{
			throw std::invalid_argument("commandedYawRad must be finite");
		}

		if (!std::isfinite(dtS) || dtS < 0)
		{
			throw std::invalid_argument("dtS must be finite and non-negative");
		}

		double clampedCommandedPitchRad = std::clamp(commandedPitchRad, -maxAngleRad, maxAngleRad);
		double clampedCommandedYawRad = std::clamp(commandedYawRad, -maxAngleRad, maxAngleRad);

		double pitchError = clampedCommandedPitchRad - angles.pitchRad;
		double yawError = clampedCommandedYawRad - angles.yawRad;

		double maxDelta = maxRateRadPerS * dtS;

		double pitchLimitDelta = std::clamp(pitchError, -maxDelta, maxDelta);
		double yawLimitDelta = std::clamp(yawError, -maxDelta, maxDelta);

		angles.pitchRad += pitchLimitDelta;
		angles.yawRad += yawLimitDelta;

		angles.pitchRad = std::clamp(angles.pitchRad, -maxAngleRad, maxAngleRad);
		angles.yawRad = std::clamp(angles.yawRad, -maxAngleRad, maxAngleRad);

		if (!std::isfinite(angles.pitchRad))
		{
			throw std::overflow_error("angles.pitchRad must be finite");
		}

		if (!std::isfinite(angles.yawRad))
		{
			throw std::overflow_error("angles.yawRad must be finite");
		}

		return angles;
	}

	const GimbalAngles& GimbalActuator::getAngles() const
	{
		return angles;
	}

	math::Vector3 GimbalActuator::getThrustDirectionB() const
	{
		return math::Vector3{ std::cos(angles.pitchRad) * std::cos(angles.yawRad), std::cos(angles.pitchRad) * std::sin(angles.yawRad), std::sin(angles.pitchRad) };
	}
}