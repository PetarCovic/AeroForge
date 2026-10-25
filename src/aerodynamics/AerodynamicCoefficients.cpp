#include <aerodynamics/AerodynamicCoefficients.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::aerodynamics
{
	AerodynamicCoefficients::AerodynamicCoefficients(
		double dragCoefficient,
		double liftCoefficient,
		double sideForceCoefficient)
		: dragCoefficient(dragCoefficient),
		liftCoefficient(liftCoefficient),
		sideForceCoefficient(sideForceCoefficient)
	{
		if (dragCoefficient < 0.0 || !std::isfinite(dragCoefficient))
		{
			throw std::invalid_argument(
				"Drag coefficient must be a finite, non-negative value.");
		}

		if (!std::isfinite(liftCoefficient))
		{
			throw std::invalid_argument(
				"Lift coefficient must be a finite value.");
		}

		if (!std::isfinite(sideForceCoefficient))
		{
			throw std::invalid_argument(
				"Side force coefficient must be a finite value.");
		}
	}

	double AerodynamicCoefficients::getDragCoefficient() const
	{
		return dragCoefficient;
	}

	double AerodynamicCoefficients::getLiftCoefficient() const
	{
		return liftCoefficient;
	}

	double AerodynamicCoefficients::getSideForceCoefficient() const
	{
		return sideForceCoefficient;
	}
}