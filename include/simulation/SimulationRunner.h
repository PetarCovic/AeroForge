#pragma once

#include  <simulation/DynamicsModel.h>
#include <simulation/Integrator.h>
#include <simulation/SimulationClock.h>
#include <simulation/State.h>
#include <cstdint>

namespace aeroforge::simulation
{
	class SimulationRunner
	{
	public:
		SimulationRunner(
			const DynamicsModel& model,
			const Integrator& integrator,
			const State& initialState,
			double timeStep);

		const State& getState() const;
		const SimulationClock& getClock() const;

		double getCurrentTime() const;
		std::uint64_t getCurrentStep() const;
		
		void step();
		void runSteps(std::uint64_t steps);

		void reset(const State& initialState, std::uint64_t startStep = 0);
		
	private:
		const DynamicsModel& model;
		const Integrator& integrator;

		State state;
		SimulationClock clock;
	};
}