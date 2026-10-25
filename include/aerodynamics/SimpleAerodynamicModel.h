#pragma once

#include <aerodynamics/AerodynamicModel.h>
#include <aerodynamics/AerodynamicReference.h>
#include <aerodynamics/AerodynamicCoefficients.h>

namespace aeroforge::aerodynamics
{
	class SimpleAerodynamicModel : public AerodynamicModel
	{
	public:
		SimpleAerodynamicModel(
			const AerodynamicReference& reference,
			const AerodynamicCoefficients& coefficients);

		math::Vector3 computeForceBody(
			const environment::AtmosphereState& atmosphere,
			const math::Vector3& relativeAirVelocityB) const override;

	private:
		AerodynamicReference reference;
		AerodynamicCoefficients coefficients;
	};
}