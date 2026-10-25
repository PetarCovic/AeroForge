#pragma once

#include <actuators/GimbalActuator.h>
#include <rocket/RocketEngine.h>
#include <math/Vector3.h>

namespace aeroforge::actuators
{
	math::Vector3 computeGimbaledThrustForceB(
		const rocket::RocketEngine& engine,
		double timeS,
		double throttle,
		const GimbalActuator& gimbal
	);
}