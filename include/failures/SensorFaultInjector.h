#pragma once 

#include <math/Vector3.h>
#include <failures/FaultSchedule.h>

#include <optional>

namespace aeroforge::failures
{
	enum class SensorFaultType
	{
		Bias,
		StuckAt,
		Dropout
	};

	class SensorFaultInjector
	{
	public:
		SensorFaultInjector(FaultSchedule schedule, SensorFaultType faultType, math::Vector3 faultParameter);

		std::optional<math::Vector3> apply(double timeS, const math::Vector3& measurement);

	private:
		FaultSchedule schedule;
		SensorFaultType faultType;
		math::Vector3 faultParameter;
		std::optional<math::Vector3> stuckMeasurement;
	};
}