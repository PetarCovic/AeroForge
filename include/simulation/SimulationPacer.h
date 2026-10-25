#pragma once

#include <chrono>

namespace aeroforge::simulation
{
	class SimulationPacer
	{
	public:
		SimulationPacer();

		void start();

		double paceTo(double simulatedElapsedS);

		double getElapsedWallTimeS() const;

	private:
		std::chrono::steady_clock::time_point start_clock;
		bool running;
	};
}