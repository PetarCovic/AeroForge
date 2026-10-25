#include <telemetry/TelemetrySchedule.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::telemetry
{
	TelemetrySchedule::TelemetrySchedule(double periodS)
		: periodS(periodS), nextRecordTimeS(0.0), lastObservedTimeS(0.0), hasObservedTime(false)
	{
		if (!std::isfinite(periodS) || periodS<=0)
		{
			throw std::invalid_argument("periodS must be finite and positive");
		}
	}

	bool TelemetrySchedule::shouldRecord(double timeS)
	{
		if (!std::isfinite(timeS) || timeS < 0)
		{
			throw std::invalid_argument("timeS must be finite and non-negative");
		}

		if (hasObservedTime && timeS < lastObservedTimeS)
		{
			throw std::invalid_argument("hasObservedTime and timeS < lastObservedTimeS cannot both be true simultaneously");
		}

		lastObservedTimeS = timeS;
		hasObservedTime = true;

		if (timeS < nextRecordTimeS)
		{
			return false;
		}

		while (nextRecordTimeS <= timeS)
		{
			nextRecordTimeS += periodS;
		}

		return true;
	}

	void TelemetrySchedule::reset()
	{
		nextRecordTimeS = 0.0;
		lastObservedTimeS = 0.0;
		hasObservedTime = false;
	}

	double TelemetrySchedule::getPeriodS() const
	{
		return periodS;
	}
}