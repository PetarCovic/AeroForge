#pragma once

#include <math/Vector3.h>
#include <failures/FaultSchedule.h>
#include <sensors/GpsModel.h>

#include <optional>

namespace aeroforge::failures
{
	enum class GpsFaultType
	{
		Bias,
		StuckAt,
		Dropout
	};

	class GpsFaultInjector
	{
	public:
		GpsFaultInjector(FaultSchedule schedule, GpsFaultType faultType, math::Vector3 positionFaultN, math::Vector3 velocityFaultN);

		std::optional<sensors::GpsMeasurement> apply(const sensors::GpsMeasurement& gpsMeasurement);

	private:
		FaultSchedule schedule;
		GpsFaultType faultType;
		math::Vector3 positionFaultN;
		math::Vector3 velocityFaultN;
		std::optional<sensors::GpsMeasurement> stuckMeasurement;
	};
}  