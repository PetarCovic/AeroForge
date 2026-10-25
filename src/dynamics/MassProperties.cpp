#include <dynamics/MassProperties.h>

#include <cmath>
#include <stdexcept>

namespace aeroforge::dynamics
{
	MassProperties::MassProperties(double massKg)
		: massKg(massKg)
	{
		if (massKg <= 0.0 || !std::isfinite(massKg))
		{
			throw std::invalid_argument("Mass must be a positive finite value.");
		}
	}
	double MassProperties::getMassKg() const
	{
		return massKg;
	}
	void MassProperties::setMassKg(double massKg)
	{
		if (massKg <= 0.0 || !std::isfinite(massKg))
		{
			throw std::invalid_argument("Mass must be a positive finite value.");
		}
		this->massKg = massKg;
	}
} // namespace aeroforge::dynamics