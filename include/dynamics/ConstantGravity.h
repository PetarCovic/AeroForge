#pragma once

#include <dynamics/MassProperties.h>
#include <math/Vector3.h>

namespace aeroforge::dynamics
{
	class ConstantGravity
	{
	public:
		explicit ConstantGravity(double gravityMagnitude = 9.80665);

		double getGravityMagnitude() const;

		math::Vector3 computeForce(const MassProperties& massProperties) const;

	private:
		double gravityMagnitude;
	};
}