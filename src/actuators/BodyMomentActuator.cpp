#include <actuators/BodyMomentActuator.h>

#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace aeroforge::actuators
{
	BodyMomentActuator::BodyMomentActuator(
		const math::Vector3& maxAbsMomentB,
		const math::Vector3& maxRateB
	)
		: maxAbsMomentB(maxAbsMomentB), maxRateB(maxRateB)
	{
		if (!maxAbsMomentB.isFinite())
		{
			throw std::invalid_argument("maxAbsMomentB must be finite");
		}

		if (maxAbsMomentB.x<=0 || maxAbsMomentB.y<=0 || maxAbsMomentB.z<=0)
		{
			throw std::invalid_argument("maxAbsMomentB must contain only positive values");
		}

		if (!maxRateB.isFinite())
		{
			throw std::invalid_argument("maxRateB must be finite");
		}

		if (maxRateB.x <= 0 || maxRateB.y <= 0 || maxRateB.z <= 0)
		{
			throw std::invalid_argument("maxRateB must contain only positive values");
		}

		actualMomentB.x = 0;
		actualMomentB.y = 0;
		actualMomentB.z = 0;
	}

	math::Vector3 BodyMomentActuator::update(
		const math::Vector3& commandedMomentB,
		double dtS
	)
	{
		if (!commandedMomentB.isFinite())
		{
			throw std::invalid_argument("commandedMomentB must be finite");
		}

		if (!std::isfinite(dtS) || dtS < 0)
		{
			throw std::invalid_argument("dtS must be finite and non-negative");
		}

		double clampedX = std::clamp(commandedMomentB.x, -maxAbsMomentB.x, maxAbsMomentB.x);
		double clampedY = std::clamp(commandedMomentB.y, -maxAbsMomentB.y, maxAbsMomentB.y);
		double clampedZ = std::clamp(commandedMomentB.z, -maxAbsMomentB.z, maxAbsMomentB.z);

		math::Vector3 clampedCommand(clampedX, clampedY, clampedZ);

		math::Vector3 error = clampedCommand - actualMomentB;

		math::Vector3 maxDelta(maxRateB.x * dtS, maxRateB.y * dtS, maxRateB.z * dtS);

		error.x = std::clamp(error.x, -maxDelta.x, maxDelta.x);
		error.y = std::clamp(error.y, -maxDelta.y, maxDelta.y);
		error.z = std::clamp(error.z, -maxDelta.z, maxDelta.z);

		actualMomentB += error;

		actualMomentB.x = std::clamp(actualMomentB.x, -maxAbsMomentB.x, maxAbsMomentB.x);
		actualMomentB.y = std::clamp(actualMomentB.y, -maxAbsMomentB.y, maxAbsMomentB.y);
		actualMomentB.z = std::clamp(actualMomentB.z, -maxAbsMomentB.z, maxAbsMomentB.z);

		if (!actualMomentB.isFinite())
		{
			throw std::overflow_error("actualMomentB must be finite");
		}

		return actualMomentB;
	}

	const math::Vector3& BodyMomentActuator::getActualMomentB() const
	{
		return actualMomentB;
	}
}