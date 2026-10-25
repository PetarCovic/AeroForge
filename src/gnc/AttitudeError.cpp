#include <gnc/AttitudeError.h>

#include <stdexcept>

namespace aeroforge::gnc
{
	math::Vector3 computeAttitudeErrorVectorB(
		const math::Quaternion& desiredAttitudeNB,
		const math::Quaternion& currentAttitudeNB
	)
	{
		if (!desiredAttitudeNB.isFinite() || desiredAttitudeNB.isZero())
		{
			throw std::invalid_argument("desiredAttitudeNB must be finite and non-zero");
		}

		if (!currentAttitudeNB.isFinite() || currentAttitudeNB.isZero())
		{
			throw std::invalid_argument("currentAttitudeNB must be finite and non-zero");
		}

		math::Quaternion normalizedDesiredAttitude = desiredAttitudeNB.normalized();
		math::Quaternion normalizedCurrentAttitude = currentAttitudeNB.normalized();

		math::Quaternion q_error = normalizedCurrentAttitude.inverse() * normalizedDesiredAttitude;

		math::Quaternion normalized_q_error = q_error.normalized();

		if (normalized_q_error.w < 0)
		{
			normalized_q_error.w = -normalized_q_error.w;
			normalized_q_error.x = -normalized_q_error.x;
			normalized_q_error.y = -normalized_q_error.y;
			normalized_q_error.z = -normalized_q_error.z;
		}

		math::Vector3 controllerError(2 * normalized_q_error.x, 2 * normalized_q_error.y, 2 * normalized_q_error.z);

		if (!controllerError.isFinite())
		{
			throw std::overflow_error("controllerError must be finite");
		}

		return controllerError;
	}
}