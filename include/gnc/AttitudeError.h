#pragma once

#include <math/Quaternion.h>
#include <math/Vector3.h>

namespace aeroforge::gnc
{
	math::Vector3 computeAttitudeErrorVectorB(
		const math::Quaternion& desiredAttitudeNB,
		const math::Quaternion& currentAttitudeNB
	);
}