#pragma once

#include <navigation/NavigationState.h>
#include <sensors/GpsModel.h>

namespace aeroforge::navigation
{
	class GpsCorrector
	{
	public:
		GpsCorrector(double positionGain, double velocityGain);

		NavigationState correct(
			const NavigationState& state,
			const sensors::GpsMeasurement& gpsMeasurement
		) const;

	private:
		double positionGain;
		double velocityGain;
	};
}