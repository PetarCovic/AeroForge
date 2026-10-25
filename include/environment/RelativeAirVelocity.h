#pragma once

#include <math/Vector3.h>

namespace aeroforge::environment
{
	math::Vector3 computeRelativeAirVelocityN(
		const math::Vector3& vehicleVelocityN,
		const math::Vector3& windVelocityN);
}