#pragma once

#include <math/Quaternion.h>
#include <math/Vector3.h>

namespace aeroforge::dynamics
{
	math::Quaternion computeQuaternionDerivativeNB(const math::Quaternion& attitudeNB, const math::Vector3& angularVelocityB);
}