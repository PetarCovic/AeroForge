#include <failures/FaultSchedule.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::failures
{
	FaultSchedule::FaultSchedule(double startTimeS, double endTimeS)
		: startTimeS(startTimeS), endTimeS(endTimeS)
	{
		if (!std::isfinite(startTimeS) || startTimeS < 0)
		{
			throw std::invalid_argument("startTimeS must be finite and non-negative");
		}

		if (!std::isfinite(endTimeS) || endTimeS <= startTimeS)
		{
			throw std::invalid_argument("endTimeS must be finite and greater than startTimeS");
		}
	}

	bool FaultSchedule::isActive(double timeS) const
	{
		if (!std::isfinite(timeS) || timeS < 0)
		{
			throw std::invalid_argument("timeS must be finite and non-negative");
		}

		if (timeS >= startTimeS && timeS < endTimeS)
		{
			return true;
		}

		return false;
	}

	double FaultSchedule::getStartTimeS() const
	{
		return startTimeS;
	}

	double FaultSchedule::getEndTimeS() const
	{
		return endTimeS;
	}
}