#include <dynamics/RotationalDynamics.h>

#include <stdexcept>

namespace aeroforge::dynamics
{
	math::Vector3 computeAngularAccelerationBody(
		const InertiaProperties& inertia,
		const math::Vector3& totalMomentB,
		const math::Vector3& angularVelocityB
	)
	{
		if (!totalMomentB.isFinite())
		{
			throw std::invalid_argument("totalMomentB must be finite");
		}

		if (!angularVelocityB.isFinite())
		{
			throw std::invalid_argument("angularVelocityB must be finite");
		}

		math::Vector3 angularMomentumB = inertia.getInertiaTensorBody()*angularVelocityB;

		math::Vector3 gyroscopicTerm = angularVelocityB.cross(angularMomentumB);

		math::Vector3 netRotationalTerm = totalMomentB - gyroscopicTerm;

		math::Vector3 angularAccelerationB = inertia.getInverseInertiaTensorBody()*netRotationalTerm;

		if (!angularAccelerationB.isFinite())
		{
			throw std::overflow_error("angularAccelerationB must be finite");
		}

		return angularAccelerationB;
	}
}