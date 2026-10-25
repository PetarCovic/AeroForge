#pragma once

namespace aeroforge::environment
{
	class AtmosphereState
	{
	public:
		explicit AtmosphereState(
			double temperatureK,
			double pressurePa,
			double densityKgPerM3,
			double speedOfSoundMps);

		double getTemperatureK() const;

		double getPressurePa() const;

		double getDensityKgPerM3() const;

		double getSpeedOfSoundMps() const;

	private:
		double temperatureK;
		double pressurePa;
		double densityKgPerM3;
		double speedOfSoundMps;
	};
}