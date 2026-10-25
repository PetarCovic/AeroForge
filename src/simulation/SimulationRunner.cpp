#include <simulation/SimulationRunner.h>
#include <stdexcept>

namespace aeroforge::simulation
{
	SimulationRunner::SimulationRunner(
		const DynamicsModel& model,
		const Integrator& integrator,
		const State& initialState,
		double timeStep)
		: model(model),
		integrator(integrator),
		state(initialState),
		clock(0, timeStep)
	{
		if(!initialState.isFinite())
		{
			throw std::invalid_argument("Initial state is not finite");
		}
	}

	const State& SimulationRunner::getState() const
	{
		return state;
	}

	const SimulationClock& SimulationRunner::getClock() const
	{
		return clock;
	}

	double SimulationRunner::getCurrentTime() const
	{
		return clock.getCurrentTime();
	}

	std::uint64_t SimulationRunner::getCurrentStep() const
	{
		return clock.getCurrentStep();
	}

	void SimulationRunner::step()
	{
		state = integrator.step(model, clock.getCurrentTime(), state, clock.getTimeStep());
		clock.advance();
	}

	void SimulationRunner::runSteps(std::uint64_t steps)
	{
		for (std::uint64_t i = 0; i < steps; ++i)
		{
			step();
		}
	}

	void SimulationRunner::reset(const State& initialState, std::uint64_t startStep)
	{
		if (!initialState.isFinite())
		{
			throw std::invalid_argument("Initial state is not finite");
		}

		state = initialState;
		clock.reset(startStep);
	}
}