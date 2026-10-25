#include <dynamics/InertiaProperties.h>

#include <stdexcept>

namespace aeroforge::dynamics 
{
	InertiaProperties::InertiaProperties(const math::Matrix& inertiaTensorBody)
		: inertiaTensorBody(inertiaTensorBody), inverseInertiaTensorBody(3, 3, 0.0)
	{
		if (inertiaTensorBody.columns() != 3 || inertiaTensorBody.rows() != 3)
		{
			throw std::invalid_argument("inertiaTensorBody must be 3x3");
		}

		if (!inertiaTensorBody.isFinite())
		{
			throw std::invalid_argument("inertiaTensorBody must be finite");
		}

		if (!inertiaTensorBody.isSymmetric())
		{
			throw std::invalid_argument("inertiaTensorBody must be symmetric");
		}

		if (!inertiaTensorBody.isPositiveDefinite())
		{
			throw std::invalid_argument("inertiaTensorBody must be positive definite");
		}

		inverseInertiaTensorBody = inertiaTensorBody.inverse();

		if (!inverseInertiaTensorBody.isFinite())
		{
			throw std::overflow_error("inverseInertiaTensorBody must be finite");
		}
	}

	const math::Matrix& InertiaProperties::getInertiaTensorBody() const
	{
		return inertiaTensorBody;
	}

	const math::Matrix& InertiaProperties::getInverseInertiaTensorBody() const
	{
		return inverseInertiaTensorBody;
	}
} // namespace aeroforge::dynamics