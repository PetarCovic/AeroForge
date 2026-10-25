#pragma once

#include <chrono>

namespace aeroforge::performance
{
	class PerformanceTimer
	{
	public:
		PerformanceTimer();

		void start();

		double stop();

		double elapsedSeconds() const;

	private:
		std::chrono::steady_clock::time_point startTimeS;
		double elapsedTimeS;
		bool running;
	};
}