#include <dynamics/ConstantGravity.h>
#include <stdexcept>
#include <cmath>

namespace aeroforge::dynamics
{
	ConstantGravity::ConstantGravity(double gravityMagnitude)
		: gravityMagnitude(gravityMagnitude)
	{
		if (gravityMagnitude <= 0.0 || !std::isfinite(gravityMagnitude))
		{
			throw std::invalid_argument("Gravity magnitude must be a positive finite value.");
		}
	}

	double ConstantGravity::getGravityMagnitude() const
	{
		return gravityMagnitude;
	}

	math::Vector3 ConstantGravity::computeForce(const MassProperties& massProperties) const
	{
		const double massKg = massProperties.getMassKg();
		const double gravityForceZ = massKg * gravityMagnitude;

		if (gravityForceZ <= 0.0 || !std::isfinite(gravityForceZ))
		{
			throw std::runtime_error("Computed gravity force is not a positive finite value.");
		}

		return math::Vector3(0.0, 0.0, gravityForceZ);
	}
} // namespace aeroforge::dynamics