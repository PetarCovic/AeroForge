#pragma once

#include <math/Vector3.h>

namespace aeroforge::environment
{
	class WindModel
	{
	public:

		virtual ~WindModel() = default;

		virtual math::Vector3 getWindVelocityN(double altitudeM, double timeS) const = 0;
	};
}