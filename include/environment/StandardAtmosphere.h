#pragma once

#include <environment/AtmosphereModel.h>

namespace aeroforge::environment
{
	class StandardAtmosphere : public AtmosphereModel
	{
	public:
		AtmosphereState getAtmosphere(double altitudeM) const override;
	};
}