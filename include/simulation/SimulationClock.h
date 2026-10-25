#pragma once

#include <cstdint>

namespace aeroforge::simulation
{
	class SimulationClock
	{
	public:
		explicit SimulationClock(
			std::uint64_t startStep = 0,
			double timeStep = 0.01);

		double getCurrentTime() const;

		std::uint64_t getCurrentStep() const;

		double getTimeStep() const;

		void advance();

		void reset(std::uint64_t startStep = 0);

	private:
		std::uint64_t currentStep;
		double dt;
	};
}