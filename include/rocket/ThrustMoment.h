#pragma once

#include <math/Vector3.h>

namespace aeroforge::rocket
{
	math::Vector3 computeThrustMomentB(
		const math::Vector3& enginePositionB,
		const math::Vector3& centerOfMassB,
		const math::Vector3& thrustForceB
	);
}