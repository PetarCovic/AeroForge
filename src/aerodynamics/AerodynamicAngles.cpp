#include <aerodynamics/AerodynamicAngles.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::aerodynamics
{
	double computeAngleOfAttackRad(const math::Vector3& relativeAirVelocityB)
	{
		if (!relativeAirVelocityB.isFinite())
		{
			throw std::invalid_argument("Relative air velocity vector is not finite.");
		}

		if (relativeAirVelocityB.magnitude() == 0.0)
		{
			return 0.0;
		}

		return std::atan2(relativeAirVelocityB.z, relativeAirVelocityB.x);
	}

	double computeSideslipAngleRad(const math::Vector3& relativeAirVelocityB)
	{
		if (!relativeAirVelocityB.isFinite())
		{
			throw std::invalid_argument("Relative air velocity vector is not finite.");
		}

		if (relativeAirVelocityB.magnitude() == 0.0)
		{
			return 0.0;
		}

		return std::atan2(relativeAirVelocityB.y, std::sqrt(relativeAirVelocityB.x * relativeAirVelocityB.x + relativeAirVelocityB.z * relativeAirVelocityB.z));
	}
}