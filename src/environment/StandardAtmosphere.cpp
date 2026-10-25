#include <environment/StandardAtmosphere.h>

#include <cmath>
#include <stdexcept>

namespace aeroforge::environment
{
	AtmosphereState StandardAtmosphere::getAtmosphere(double altitudeM) const
	{
		if (altitudeM < 0.0 || altitudeM > 20000.0 || !std::isfinite(altitudeM))
		{
			throw std::invalid_argument("Altitude must be between 0 and 20,000 meters.");
		}

		double seaLevelTemperature = 288.15;
		double seaLevelPressure = 101325;
		double lapseRate = -0.0065;
		double g = 9.80665;
		double R = 287.05287;
		double gamma = 1.4;

		double temperatureK;
		double pressurePa;
		double densityKgPerM3;
		double speedOfSoundMps;
		if (altitudeM <= 11000.0)
		{
			temperatureK = seaLevelTemperature + lapseRate * altitudeM;
			pressurePa = seaLevelPressure * std::pow(temperatureK / seaLevelTemperature, 5.255877);
			densityKgPerM3 = pressurePa / (R * temperatureK);
			speedOfSoundMps = std::sqrt(gamma * R * temperatureK);
		}
		else if (altitudeM <= 20000.0)
		{
			temperatureK = 216.65;
			pressurePa = 22632.1 * std::exp(-1.0 * (g / (R * temperatureK)) * (altitudeM - 11000.0));
			densityKgPerM3 = pressurePa / (R * temperatureK);
			speedOfSoundMps = std::sqrt(gamma * R * temperatureK);
		}
		else
		{
			throw std::invalid_argument("Altitude above 20,000 meters is not supported in this model.");
		}
		return AtmosphereState(temperatureK, pressurePa, densityKgPerM3, speedOfSoundMps);
	}
} // namespace aeroforge::environment