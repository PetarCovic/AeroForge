#include <rocket/PropellantDepletion.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::rocket
{
	double consumePropellantForStep(
		PropellantTank& tank,
		const RocketEngine& engine,
		double timeS,
		double dtS,
		double throttle
	)
	{
		if (!std::isfinite(dtS) || dtS < 0)
		{
			throw std::invalid_argument("dtS must be finite and non-negative");
		}

		double massFlow = engine.computeMassFlowRateKgPerS(timeS, throttle);

		double requestedMass = massFlow * dtS;

		if (!std::isfinite(requestedMass))
		{
			throw std::overflow_error("requestedMass must be finite");
		}

		return tank.consume(requestedMass);
	}
}