#pragma once

#include <simulation/DynamicsModel.h>

namespace aeroforge::simulation
{
	class Integrator
	{
	public:
		virtual ~Integrator() = default;

		virtual State step(const DynamicsModel& model, double time, const State& state, double dt) const = 0;
	};
}