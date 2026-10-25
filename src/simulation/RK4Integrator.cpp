#include <simulation/RK4Integrator.h>
#include <cmath>
#include <stdexcept>

namespace aeroforge::simulation
{
	State RK4Integrator::step(const DynamicsModel& model, double time, const State& state, double dt) const
	{
		if (!std::isfinite(dt) || dt <= 0.0)
		{
			throw std::invalid_argument("Time step must be finite and positive.");
		}

		if (!state.isFinite())
		{
			throw std::invalid_argument("State must be finite.");
		}

		if (!std::isfinite(time))
		{
			throw std::invalid_argument("Time must be finite.");
		}

		StateDerivative k1 = model.evaluate(time, state);

		if (k1.size() != state.size() || !k1.isFinite())
		{
			throw std::invalid_argument("State derivative size must match state size and be finite.");
		}

		State intermediate = state;
		intermediate.addScaled(k1, dt / 2.0);
		StateDerivative k2 = model.evaluate(time + dt / 2.0, intermediate);

		if (k2.size() != state.size() || !k2.isFinite())
		{
			throw std::invalid_argument("State derivative size must match state size and be finite.");
		}

		intermediate = state;
		intermediate.addScaled(k2, dt / 2.0);
		StateDerivative k3 = model.evaluate(time + dt / 2.0, intermediate);

		if (k3.size() != state.size() || !k3.isFinite())
		{
			throw std::invalid_argument("State derivative size must match state size and be finite.");
		}

		intermediate = state;
		intermediate.addScaled(k3, dt);
		StateDerivative k4 = model.evaluate(time + dt, intermediate);

		if (k4.size() != state.size() || !k4.isFinite())
		{
			throw std::invalid_argument("State derivative size must match state size and be finite.");
		}

		State result = state;
		result.addScaled(k1, dt / 6.0);
		result.addScaled(k2, dt / 3.0);
		result.addScaled(k3, dt / 3.0);
		result.addScaled(k4, dt / 6.0);

		return result;
	}
}