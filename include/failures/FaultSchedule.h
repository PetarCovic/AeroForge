#pragma once

namespace aeroforge::failures
{
	class FaultSchedule
	{
	public:
		FaultSchedule(double startTimeS, double endTimeS);

		bool isActive(double timeS) const;

		double getStartTimeS() const;

		double getEndTimeS() const;

	private:
		double startTimeS;
		double endTimeS;
	};
}