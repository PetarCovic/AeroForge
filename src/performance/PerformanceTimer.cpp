#include <performance/PerformanceTimer.h>

#include <stdexcept>

namespace aeroforge::performance
{
	PerformanceTimer::PerformanceTimer()
		: elapsedTimeS(0.0), running(false)
	{
		
	}

	void PerformanceTimer::start()
	{
		startTimeS = std::chrono::steady_clock::now();
		running = true;
	}

	double PerformanceTimer::stop()
	{
		if (!running)
		{
			throw std::logic_error("PerformanceTimer is not running");
		}

		std::chrono::steady_clock::time_point endTime = std::chrono::steady_clock::now();

		elapsedTimeS = std::chrono::duration<double>(endTime-startTimeS).count();

		running = false;

		return elapsedTimeS;
	}

	double PerformanceTimer::elapsedSeconds() const
	{
		if (running)
		{
			return std::chrono::duration<double>(std::chrono::steady_clock::now()-startTimeS).count();
		}

		return elapsedTimeS;
	}
}