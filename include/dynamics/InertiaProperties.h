#pragma once

#include <math/Matrix.h>

namespace aeroforge::dynamics
{
	class InertiaProperties
	{
	public:
		InertiaProperties(const math::Matrix& inertiaTensorBody);

		const math::Matrix& getInertiaTensorBody() const;

		const math::Matrix& getInverseInertiaTensorBody() const;

	private:
		math::Matrix inertiaTensorBody;
		math::Matrix inverseInertiaTensorBody;
	};
}