#pragma once

#include <math/Vector3.h>

namespace aeroforge::aerodynamics
{
	double computeAngleOfAttackRad(const math::Vector3& relativeAirVelocityB);

	double computeSideslipAngleRad(const math::Vector3& relativeAirVelocityB);
}