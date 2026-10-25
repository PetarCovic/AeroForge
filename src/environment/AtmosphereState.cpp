#include <environment/AtmosphereState.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::environment
{
	AtmosphereState::AtmosphereState(
		double temperatureK,
		double pressurePa,
		double densityKgPerM3,
		double speedOfSoundMps)
		: temperatureK(temperatureK),
		pressurePa(pressurePa),
		densityKgPerM3(densityKgPerM3),
		speedOfSoundMps(speedOfSoundMps)
	{
		if (temperatureK <= 0.0 || !std::isfinite(temperatureK))
		{
			throw std::invalid_argument("Temperature must be a positive finite value.");
		}

		if (pressurePa <= 0.0 || !std::isfinite(pressurePa))
		{
			throw std::invalid_argument("Pressure must be a positive finite value.");
		}

		if (densityKgPerM3 <= 0.0 || !std::isfinite(densityKgPerM3))
		{
			throw std::invalid_argument("Density must be a positive finite value.");
		}

		if (speedOfSoundMps <= 0.0 || !std::isfinite(speedOfSoundMps))
		{
			throw std::invalid_argument("Speed of sound must be a positive finite value.");
		}
	}

	double AtmosphereState::getTemperatureK() const
	{
		return temperatureK;
	}

	double AtmosphereState::getPressurePa() const
	{
		return pressurePa;
	}

	double AtmosphereState::getDensityKgPerM3() const
	{
		return densityKgPerM3;
	}

	double AtmosphereState::getSpeedOfSoundMps() const
	{
		return speedOfSoundMps;
	}
}