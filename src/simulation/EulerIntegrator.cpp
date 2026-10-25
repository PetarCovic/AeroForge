#include <simulation/EulerIntegrator.h>
#include <cmath>
#include <stdexcept>

namespace aeroforge::simulation
{
	State EulerIntegrator::step(const DynamicsModel& model, double time, const State& state, double dt) const
	{
		if (!std::isfinite(dt) || dt<=0.0)
		{
			throw std::invalid_argument("Time step must be finite and positive.");
		}

		if(!state.isFinite())
		{
			throw std::invalid_argument("State must be finite.");
		}

		if (!std::isfinite(time))
		{
			throw std::invalid_argument("Time must be finite.");
		}

		StateDerivative derivative = model.evaluate(time, state);

		if (derivative.size() != state.size())
		{
			throw std::invalid_argument("State derivative size must match state size.");
		}

		if (!derivative.isFinite())
		{
			throw std::invalid_argument("State derivative must be finite.");
		}

		return state + derivative * dt;
	}
}