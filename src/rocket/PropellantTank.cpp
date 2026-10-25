#include <rocket/PropellantTank.h>

#include <stdexcept>
#include <cmath>
#include <algorithm>

namespace aeroforge::rocket
{
	PropellantTank::PropellantTank(double capacityKg, double initialPropellantMassKg)
		: capacityKg(capacityKg), propellantMassKg(initialPropellantMassKg)
	{
		if (!std::isfinite(capacityKg) || capacityKg <= 0)
		{
			throw std::invalid_argument("capacityKg must be finite and positive");
		}

		if (!std::isfinite(initialPropellantMassKg) || initialPropellantMassKg < 0)
		{
			throw std::invalid_argument("initialPropellantMassKg must be finite and non-negative");
		}

		if (initialPropellantMassKg > capacityKg)
		{
			throw std::invalid_argument("initialPropellantMassKg must be between 0 and capacityKg");
		}
	}

	double PropellantTank::getCapacityKg() const
	{
		return capacityKg;
	}

	double PropellantTank::getPropellantMassKg() const
	{
		return propellantMassKg;
	}

	double PropellantTank::getPropellantFraction() const
	{
		return propellantMassKg / capacityKg;
	}

	bool PropellantTank::isEmpty() const
	{
		return propellantMassKg == 0.0;
	}

	double PropellantTank::consume(double massKg)
	{
		if (!std::isfinite(massKg) || massKg < 0)
		{
			throw std::invalid_argument("massKg must be finite and non-negative");
		}

		double consumedMass = std::min(massKg, propellantMassKg);
		propellantMassKg -= consumedMass;

		return consumedMass;
	}
}