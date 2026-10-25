#pragma once

namespace aeroforge::telemetry
{
	class TelemetrySchedule
	{
	public:
		TelemetrySchedule(double periodS);

		bool shouldRecord(double timeS);

		void reset();

		double getPeriodS() const;

	private:
		double periodS;
		double nextRecordTimeS;
		double lastObservedTimeS;
		bool hasObservedTime;
	};
}