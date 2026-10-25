#include <gnc/AttitudePdController.h>
#include <gnc/AttitudeError.h>

#include <stdexcept>
#include <algorithm>

namespace aeroforge::gnc
{
	AttitudePdController::AttitudePdController(
		const math::Vector3& proportionalGain,
		const math::Vector3& derivativeGain,
		const math::Vector3& maxAbsMomentB
	)
		: proportionalGain(proportionalGain), derivativeGain(derivativeGain), maxAbsMomentB(maxAbsMomentB)
	{
		if(!proportionalGain.isFinite())
		{
			throw std::invalid_argument("proportionalGain must be finite");
		}

		if(proportionalGain.x<0 || proportionalGain.y<0 || proportionalGain.z<0)
		{
			throw std::invalid_argument("proportionalGain must only contain non-negatives");
		}

		if (!derivativeGain.isFinite())
		{
			throw std::invalid_argument("derivativeGain must be finite");
		}

		if (derivativeGain.x < 0 || derivativeGain.y < 0 || derivativeGain.z < 0)
		{
			throw std::invalid_argument("derivativeGain must only contain non-negatives");
		}

		if (!maxAbsMomentB.isFinite())
		{
			throw std::invalid_argument("maxAbsMomentB must be finite");
		}

		if (maxAbsMomentB.x <= 0 || maxAbsMomentB.y <= 0 || maxAbsMomentB.z <= 0)
		{
			throw std::invalid_argument("maxAbsMomentB must only contain positive values");
		}
	}

	math::Vector3 AttitudePdController::computeMomentCommandB(
		const AttitudeSetpoint& setpoint,
		const navigation::NavigationState& navigationState
	) const
	{
		math::Vector3 attitudeErrorB = computeAttitudeErrorVectorB(setpoint.getAttitudeNB(), navigationState.getAttitudeNB());

		math::Vector3 angularVelocityErrorB = setpoint.getAngularVelocityB() - navigationState.getAngularVelocityB();

		math::Vector3 moment(
			proportionalGain.x* attitudeErrorB.x+derivativeGain.x*angularVelocityErrorB.x,
			proportionalGain.y * attitudeErrorB.y + derivativeGain.y * angularVelocityErrorB.y,
			proportionalGain.z * attitudeErrorB.z + derivativeGain.z * angularVelocityErrorB.z
			);

		moment.x = std::clamp(moment.x, -maxAbsMomentB.x, maxAbsMomentB.x);
		moment.y = std::clamp(moment.y, -maxAbsMomentB.y, maxAbsMomentB.y);
		moment.z = std::clamp(moment.z, -maxAbsMomentB.z, maxAbsMomentB.z);

		if (!moment.isFinite())
		{
			throw std::overflow_error("moment must be finite");
		}

		return moment;
	}
}