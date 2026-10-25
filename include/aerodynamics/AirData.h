#pragma once

#include <environment/AtmosphereState.h>
#include <math/Vector3.h>

namespace aeroforge::aerodynamics
{
	class AirData
	{
	public:
		AirData(
			double airspeedMps,
			double dynamicPressurePa,
			double machNumber);

		double getAirspeedMps() const;

		double getDynamicPressurePa() const;

		double getMachNumber() const;

	private:
		double airspeedMps;
		double dynamicPressurePa;
		double machNumber;
	};

	AirData computeAirData(
		const environment::AtmosphereState& atmosphere,
		const math::Vector3& relativeAirVelocity);
}