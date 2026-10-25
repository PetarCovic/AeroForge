#pragma once

#include <math/Quaternion.h>
#include <math/Vector3.h>

namespace aeroforge::gnc
{
	class AttitudeSetpoint
	{
	public:
		AttitudeSetpoint(
			const math::Quaternion& desiredAttitudeNB,
			const math::Vector3& desiredAngularVelocityB
		);

		const math::Quaternion& getAttitudeNB() const;

		const math::Vector3& getAngularVelocityB() const;

	private:
		math::Quaternion attitudeNB;
		math::Vector3 angularVelocityB;
	};
}