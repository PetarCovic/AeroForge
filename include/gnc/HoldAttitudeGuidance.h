#pragma once

#include <gnc/AttitudeSetpoint.h>

namespace aeroforge::gnc
{
	class HoldAttitudeGuidance
	{
	public:
		HoldAttitudeGuidance(const AttitudeSetpoint& setpoint);

		AttitudeSetpoint getSetpoint(double timeS) const;

	private:
		AttitudeSetpoint setpoint;
	};
}