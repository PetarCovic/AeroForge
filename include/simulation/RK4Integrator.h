#pragma once

#include <simulation/Integrator.h>

namespace aeroforge::simulation
{
	class RK4Integrator : public Integrator
	{
	public:
		State step(const DynamicsModel& model, double time, const State& state, double dt) const override;
	};
}