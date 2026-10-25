#include <navigation/NavigationState.h>

#include <stdexcept>

namespace aeroforge::navigation
{
	NavigationState::NavigationState(
		const math::Vector3& positionN,
		const math::Vector3& velocityN,
		const math::Quaternion& attitudeNB,
		const math::Vector3& angularVelocityB
	)
		: positionN(positionN), velocityN(velocityN), attitudeNB(attitudeNB), angularVelocityB(angularVelocityB)
	{
		if (!positionN.isFinite())
		{
			throw std::invalid_argument("positionN must be finite");
		}

		if (!velocityN.isFinite())
		{
			throw std::invalid_argument("velocityN must be finite");
		}

		if (!attitudeNB.isFinite() || attitudeNB.isZero())
		{
			throw std::invalid_argument("attitudeNB must be finite and cannot be the zero quaternion");
		}

		this->attitudeNB = attitudeNB.normalized();

		if (!angularVelocityB.isFinite())
		{
			throw std::invalid_argument("angularVelocityB must be finite");
		}
	}

	const math::Vector3& NavigationState::getPositionN() const
	{
		return positionN;
	}

	const math::Vector3& NavigationState::getVelocityN() const
	{
		return velocityN;
	}

	const math::Quaternion& NavigationState::getAttitudeNB() const
	{
		return attitudeNB;
	}

	const math::Vector3& NavigationState::getAngularVelocityB() const
	{
		return angularVelocityB;
	}
}