#include <simulation/SimulationClock.h>

#include <cmath>
#include <stdexcept>

namespace aeroforge::simulation
{
	SimulationClock::SimulationClock(
		std::uint64_t startStep,
		double timeStep)
		: currentStep(startStep),
		dt(timeStep)
	{
		if (!std::isfinite(timeStep) || timeStep <= 0.0)
		{
			throw std::invalid_argument(
				"Time step must be finite and positive.");
		}
	}

	double SimulationClock::getCurrentTime() const
	{
		return static_cast<double>(currentStep) * dt;
	}

	std::uint64_t SimulationClock::getCurrentStep() const
	{
		return currentStep;
	}

	double SimulationClock::getTimeStep() const
	{
		return dt;
	}

	void SimulationClock::advance()
	{
		++currentStep;
	}

	void SimulationClock::reset(std::uint64_t startStep)
	{
		currentStep = startStep;
	}
}