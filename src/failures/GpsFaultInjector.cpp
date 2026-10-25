#include <failures/GpsFaultInjector.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::failures
{
	GpsFaultInjector::GpsFaultInjector(FaultSchedule schedule, GpsFaultType faultType, math::Vector3 positionFaultN, math::Vector3 velocityFaultN)
		: schedule(schedule), faultType(faultType), positionFaultN(positionFaultN), velocityFaultN(velocityFaultN)
	{
		if (!positionFaultN.isFinite())
		{
			throw std::invalid_argument("positionFaultN must be finite");
		}

		if (!velocityFaultN.isFinite())
		{
			throw std::invalid_argument("velocityFaultN must be finite");
		}
	}

	std::optional<sensors::GpsMeasurement> GpsFaultInjector::apply(const sensors::GpsMeasurement& gpsMeasurement)
	{
		if (!gpsMeasurement.positionN.isFinite())
		{
			throw std::invalid_argument("gpsMeasurement.positionN must be finite");
		}

		if (!gpsMeasurement.velocityN.isFinite())
		{
			throw std::invalid_argument("gpsMeasurement.velocityN must be finite");
		}

		if (!std::isfinite(gpsMeasurement.timeS) || gpsMeasurement.timeS<0)
		{
			throw std::invalid_argument("gpsMeasurement.timeS must be finite and non-negative");
		}

		math::Vector3 positionOutput;
		math::Vector3 velocityOutput;

		if (!schedule.isActive(gpsMeasurement.timeS))
		{
			stuckMeasurement.reset();
			return gpsMeasurement;
		}

		if (faultType == GpsFaultType::Bias)
		{
			positionOutput = gpsMeasurement.positionN + positionFaultN;
			velocityOutput = gpsMeasurement.velocityN + velocityFaultN;
		}

		if (faultType == GpsFaultType::StuckAt)
		{
			if (!stuckMeasurement.has_value())
			{
				stuckMeasurement = gpsMeasurement;
			}

			return stuckMeasurement;
		}

		if (faultType == GpsFaultType::Dropout)
		{
			return std::nullopt;
		}

		if (!positionOutput.isFinite())
		{
			throw std::overflow_error("positionOutput must be finite");
		}

		if (!velocityOutput.isFinite())
		{
			throw std::overflow_error("velocityOutput must be finite");
		}

		sensors::GpsMeasurement measurement{ gpsMeasurement.timeS, positionOutput, velocityOutput };

		return measurement;
	}
}