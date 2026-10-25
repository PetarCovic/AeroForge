#include <failures/ActuatorFaultInjector.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::failures
{
	ActuatorFaultInjector::ActuatorFaultInjector(FaultSchedule schedule, ActuatorFaultType faultType, math::Vector3 faultParameterB)
		: schedule(schedule), faultType(faultType), faultParameterB(faultParameterB)
	{
		if (!faultParameterB.isFinite())
		{
			throw std::invalid_argument("faultParameterB must be finite");
		}


		if (faultType == ActuatorFaultType::LossOfEffectiveness)
		{
			if (faultParameterB.x < 0 || faultParameterB.x>1
				|| faultParameterB.y < 0 || faultParameterB.y>1
				|| faultParameterB.z < 0 || faultParameterB.z>1)
			{
				throw std::invalid_argument("faultParameterB values must be in [0, 1]");
			}
		}
	}

	math::Vector3 ActuatorFaultInjector::apply(double timeS, const math::Vector3& nominalOutputB)
	{
		if (!std::isfinite(timeS) || timeS < 0)
		{
			throw std::invalid_argument("timeS must be finite and non-negative");
		}

		if (!nominalOutputB.isFinite())
		{
			throw std::invalid_argument("nominalOutputB must be finite");
		}

		math::Vector3 output;

		if (!schedule.isActive(timeS))
		{
			stuckOutputB.reset();
			return nominalOutputB;
		}

		if (faultType == ActuatorFaultType::LossOfEffectiveness)
		{
			output.x = nominalOutputB.x * faultParameterB.x;
			output.y = nominalOutputB.y * faultParameterB.y;
			output.z = nominalOutputB.z * faultParameterB.z;
		}

		if (faultType == ActuatorFaultType::StuckAt)
		{
			if (!stuckOutputB.has_value())
			{
				stuckOutputB = nominalOutputB;
			}

			return *stuckOutputB;
		}

		if (faultType == ActuatorFaultType::HardOver)
		{
			return faultParameterB;
		}

		if (!output.isFinite())
		{
			throw std::overflow_error("output must be finite");
		}

		return output;
	}
}