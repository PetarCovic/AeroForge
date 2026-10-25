#pragma once

#include <dynamics/InertiaProperties.h>
#include <math/Vector3.h>

namespace aeroforge::dynamics
{
	math::Vector3 computeAngularAccelerationBody(
		const InertiaProperties& inertia,
		const math::Vector3& totalMomentB,
		const math::Vector3& angularVelocityB
	);
}