#include <failures/SensorFaultInjector.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::failures
{
	SensorFaultInjector::SensorFaultInjector(FaultSchedule schedule, SensorFaultType faultType, math::Vector3 faultParameter)
		: schedule(schedule), faultType(faultType), faultParameter(faultParameter)
	{
		if (!faultParameter.isFinite())
		{
			throw std::invalid_argument("faultParameter must be finite");
		}
	}

	std::optional<math::Vector3> SensorFaultInjector::apply(double timeS, const math::Vector3& measurement)
	{
		if (!std::isfinite(timeS) || timeS<0)
		{
			throw std::invalid_argument("timeS must be finite and non-negative");
		}

		if (!measurement.isFinite())
		{
			throw std::invalid_argument("measurement must be finite");
		}

		math::Vector3 output;
		
		if (!schedule.isActive(timeS))
		{
			stuckMeasurement.reset();
			return measurement;
		}

		if (faultType == SensorFaultType::Bias)
		{
			output = measurement + faultParameter;
		}

		if (faultType == SensorFaultType::StuckAt)
		{
			if (!stuckMeasurement.has_value())
			{
				stuckMeasurement = measurement;
			}

			return stuckMeasurement;
		}

		if (faultType == SensorFaultType::Dropout)
		{
			return std::nullopt;
		}

		if (!output.isFinite())
		{
			throw std::overflow_error("output must be finite");
		}

		return output;
	}
}