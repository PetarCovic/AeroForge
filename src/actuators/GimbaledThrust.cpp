#include <actuators/GimbaledThrust.h>

#include <stdexcept>

namespace aeroforge::actuators
{
	math::Vector3 computeGimbaledThrustForceB(
		const rocket::RocketEngine& engine,
		double timeS,
		double throttle,
		const GimbalActuator& gimbal
	)
	{
		math::Vector3 directionB = gimbal.getThrustDirectionB();

		double thrustN = engine.getThrustN(timeS, throttle);

		math::Vector3 forceB = directionB * thrustN;

		if (!forceB.isFinite())
		{
			throw std::overflow_error("forceB must be finite");
		}

		return forceB;
	}
}