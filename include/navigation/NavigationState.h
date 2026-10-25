#pragma once

#include <math/Vector3.h>
#include <math/Quaternion.h>

namespace aeroforge::navigation
{
	class NavigationState
	{
	public:
		NavigationState(
			const math::Vector3& positionN,
			const math::Vector3& velocityN,
			const math::Quaternion& attitudeNB,
			const math::Vector3& angularVelocityB
		);

		const math::Vector3& getPositionN() const;

		const math::Vector3& getVelocityN() const;

		const math::Quaternion& getAttitudeNB() const;

		const math::Vector3& getAngularVelocityB() const;

	private:
		math::Vector3 positionN;
		math::Vector3 velocityN;
		math::Quaternion attitudeNB;
		math::Vector3 angularVelocityB;
	};
}