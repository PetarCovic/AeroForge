#pragma once

namespace aeroforge::sensors
{
	class SensorSchedule
	{
	public:
		SensorSchedule(double sampleRateHz, double startTimeS);

		double getSamplePeriodS() const;

		bool shouldSample(double timeS);

	private:
		double sampleRateHz;
		double samplePeriodS;
		double nextSampleTimeS;
	};
}