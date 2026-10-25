#pragma once

#include <navigation/NavigationState.h>
#include <sensors/ImuModel.h>
#include <math/Vector3.h>

namespace aeroforge::navigation
{
	class InertialNavigator
	{
	public:
		InertialNavigator();

		NavigationState propagate(
			const NavigationState& state,
			const sensors::ImuMeasurement& imuMeasurement,
			const math::Vector3& gravityN,
			double dtS
		) const;
	};
}