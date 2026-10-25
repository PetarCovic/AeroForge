#include <gnc/AttitudeSetpoint.h>

#include <stdexcept>

namespace aeroforge::gnc
{
	AttitudeSetpoint::AttitudeSetpoint(
		const math::Quaternion& desiredAttitudeNB,
		const math::Vector3& desiredAngularVelocityB
	)
	{
		if (!desiredAttitudeNB.isFinite() || desiredAttitudeNB.isZero())
		{
			throw std::invalid_argument("desiredAttitudeNB must be finite and non-zero");
		}

		if (!desiredAngularVelocityB.isFinite())
		{
			throw std::invalid_argument("desiredAngularVelocityB must be finite");
		}

		attitudeNB = desiredAttitudeNB.normalized();
		angularVelocityB = desiredAngularVelocityB;
	}

	const math::Quaternion& AttitudeSetpoint::getAttitudeNB() const
	{
		return attitudeNB;
	}

	const math::Vector3& AttitudeSetpoint::getAngularVelocityB() const
	{
		return angularVelocityB;
	}
}