#pragma once

#include <math/Vector3.h>
#include <failures/FaultSchedule.h>

#include <optional>

namespace aeroforge::failures
{
	enum class ActuatorFaultType
	{
		LossOfEffectiveness,
		StuckAt,
		HardOver
	};

	class ActuatorFaultInjector
	{
	public:
		ActuatorFaultInjector(FaultSchedule schedule, ActuatorFaultType faultType, math::Vector3 faultParameterB);

		math::Vector3 apply(double timeS, const math::Vector3& nominalOutputB);

	private:
		FaultSchedule schedule;
		ActuatorFaultType faultType;
		math::Vector3 faultParameterB;
		std::optional<math::Vector3> stuckOutputB;
	};
}