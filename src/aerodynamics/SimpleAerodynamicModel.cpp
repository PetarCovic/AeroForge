#include <aerodynamics/SimpleAerodynamicModel.h>
#include <aerodynamics/AirData.h>
#include <aerodynamics/AerodynamicAngles.h>
#include <aerodynamics/AerodynamicFrameTransform.h>

#include <stdexcept>

namespace aeroforge::aerodynamics
{
	SimpleAerodynamicModel::SimpleAerodynamicModel(
		const AerodynamicReference& reference,
		const AerodynamicCoefficients& coefficients)
		: reference(reference), coefficients(coefficients)
	{
	}

	math::Vector3 SimpleAerodynamicModel::computeForceBody(
		const environment::AtmosphereState& atmosphere,
		const math::Vector3& relativeAirVelocityB) const
	{
		if (!relativeAirVelocityB.isFinite())
		{
			throw std::invalid_argument("relativeAirVelocityB must be finite");
		}

		AirData airData = computeAirData(atmosphere, relativeAirVelocityB);

		double q = airData.getDynamicPressurePa();
		double S = reference.getReferenceAreaM2();

		double qS = q * S;

		double D = qS * coefficients.getDragCoefficient();
		double L = qS * coefficients.getLiftCoefficient();
		double Y = qS * coefficients.getSideForceCoefficient();

		math::Vector3 forceWind(-D, Y, -L);

		if (!forceWind.isFinite())
		{
			throw std::overflow_error("forceWind must be finite");
		}

		double alpha = computeAngleOfAttackRad(relativeAirVelocityB);
		double beta = computeSideslipAngleRad(relativeAirVelocityB);

		math::Vector3 bodyVector = transformWindToBody(forceWind, alpha, beta);

		if (!bodyVector.isFinite())
		{
			throw std::overflow_error("bodyVector must be finite");
		}

		return bodyVector;
	}
}