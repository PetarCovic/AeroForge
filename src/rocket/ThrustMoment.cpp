#include <rocket/ThrustMoment.h>

#include <stdexcept>

namespace aeroforge::rocket
{
	math::Vector3 computeThrustMomentB(
		const math::Vector3& enginePositionB,
		const math::Vector3& centerOfMassB,
		const math::Vector3& thrustForceB
	)
	{
		if (!enginePositionB.isFinite())
		{
			throw std::invalid_argument("enginePositionB must be finite");
		}

		if (!centerOfMassB.isFinite())
		{
			throw std::invalid_argument("centerOfMassB must be finite");
		}

		if (!thrustForceB.isFinite())
		{
			throw std::invalid_argument("thrustForceB must be finite");
		}

		math::Vector3 momentArmB = enginePositionB - centerOfMassB;
		math::Vector3 momentB = momentArmB.cross(thrustForceB);

		if (!momentB.isFinite())
		{
			throw std::overflow_error("momentB must be finite");
		}

		return momentB;
	}
}