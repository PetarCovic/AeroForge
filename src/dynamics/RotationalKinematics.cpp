#include <dynamics/RotationalKinematics.h>

#include <stdexcept>

namespace aeroforge::dynamics
{
	math::Quaternion computeQuaternionDerivativeNB(const math::Quaternion& attitudeNB, const math::Vector3& angularVelocityB)
	{
		if (!attitudeNB.isFinite() || attitudeNB.isZero())
		{
			throw std::invalid_argument("attitudeNB must be finite and non-zero");
		}

		if (!angularVelocityB.isFinite())
		{
			throw std::invalid_argument("angularVelocityB must be finite");
		}

		math::Quaternion omegaQuaternion(0, angularVelocityB.x, angularVelocityB.y, angularVelocityB.z);
		math::Quaternion q_dot = .5 * (attitudeNB * omegaQuaternion);

		if (!q_dot.isFinite())
		{
			throw std::overflow_error("q_dot must be finite");
		}

		return q_dot;
	}
}