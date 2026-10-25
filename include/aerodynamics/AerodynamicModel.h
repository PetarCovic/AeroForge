#pragma once

#include <environment/AtmosphereState.h>
#include <math/Vector3.h>

namespace aeroforge::aerodynamics
{
	class AerodynamicModel
	{
	public:
		virtual ~AerodynamicModel() = default;

		virtual math::Vector3 computeForceBody(
			const environment::AtmosphereState& atmosphere,
			const math::Vector3& relativeAirVelocityB) const = 0;
	};
} // namespace aeroforge::aerodynamics