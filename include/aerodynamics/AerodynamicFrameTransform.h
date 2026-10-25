#pragma once

#include <math/Vector3.h>

namespace aeroforge::aerodynamics
{
	math::Vector3 transformWindToBody(
		const math::Vector3& vectorWind,
		double alphaRad,
		double betaRad);
}