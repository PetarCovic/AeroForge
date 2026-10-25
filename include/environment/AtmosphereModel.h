#pragma once

#include <environment/AtmosphereState.h>

namespace aeroforge::environment
{
	class AtmosphereModel
	{
	public:
		virtual ~AtmosphereModel() = default;

		virtual AtmosphereState getAtmosphere(double altitudeM) const = 0;
	};
}