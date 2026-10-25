#include <aerodynamics/AirData.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::aerodynamics
{
	AirData::AirData(double airspeedMps, double dynamicPressurePa, double machNumber)
		: airspeedMps(airspeedMps), dynamicPressurePa(dynamicPressurePa), machNumber(machNumber)
	{
		if (!std::isfinite(airspeedMps) || airspeedMps < 0.0)
		{
			throw std::invalid_argument("Airspeed must be a finite non-negative value.");
		}

		if (!std::isfinite(dynamicPressurePa) || dynamicPressurePa < 0.0)
		{
			throw std::invalid_argument("Dynamic pressure must be a finite non-negative value.");
		}

		if (!std::isfinite(machNumber) || machNumber < 0.0)
		{
			throw std::invalid_argument("Mach number must be a finite non-negative value.");
		}
	}

	double AirData::getAirspeedMps() const
	{
		return airspeedMps;
	}

	double AirData::getDynamicPressurePa() const
	{
		return dynamicPressurePa;
	}

	double AirData::getMachNumber() const
	{
		return machNumber;
	}

	AirData computeAirData(const environment::AtmosphereState& atmosphere, const math::Vector3& relativeAirVelocity)
	{
		if(atmosphere.getDensityKgPerM3() <= 0.0 || !std::isfinite(atmosphere.getDensityKgPerM3()))
		{
			throw std::invalid_argument("Atmospheric density must be a finite positive value.");
		}

		if (atmosphere.getSpeedOfSoundMps() <= 0.0 || !std::isfinite(atmosphere.getSpeedOfSoundMps()))
		{
			throw std::invalid_argument("Speed of sound must be a finite positive value.");
		}

		if(!relativeAirVelocity.isFinite())
		{
			throw std::invalid_argument("Relative air velocity must be a finite vector.");
		}

		double V = relativeAirVelocity.magnitude();
		double q = 0.5 * atmosphere.getDensityKgPerM3() * V * V;
		double M = V / atmosphere.getSpeedOfSoundMps();

		if (!std::isfinite(V) || V < 0.0)
		{
			throw std::overflow_error("Computed airspeed is not a finite non-negative value.");
		}

		if (!std::isfinite(q) || q < 0.0)
		{
			throw std::overflow_error("Computed dynamic pressure is not a finite non-negative value.");
		}

		if (!std::isfinite(M) || M < 0.0)
		{
			throw std::overflow_error("Computed mach number is not a finite non-negative value.");
		}

		return AirData(V, q, M);
	}
}