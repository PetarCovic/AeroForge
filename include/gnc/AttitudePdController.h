#pragma once

#include <gnc/AttitudeSetpoint.h>
#include <navigation/NavigationState.h>
#include <math/Vector3.h>

namespace aeroforge::gnc
{
	class AttitudePdController
	{
	public:
		AttitudePdController(
			const math::Vector3& proportionalGain,
			const math::Vector3& derivativeGain,
			const math::Vector3& maxAbsMomentB
		);

		math::Vector3 computeMomentCommandB(
			const AttitudeSetpoint& setpoint,
			const navigation::NavigationState& navigationState
		) const;

	private:
		math::Vector3 proportionalGain;
		math::Vector3 derivativeGain;
		math::Vector3 maxAbsMomentB;
	};
}