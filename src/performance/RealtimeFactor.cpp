#include <performance/RealtimeFactor.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::performance
{
	double RealtimeFactor::compute(double simulatedTimeS, double wallClockTimeS)
	{
		if (!std::isfinite(simulatedTimeS) || simulatedTimeS < 0)
		{
			throw std::invalid_argument("simulatedTimeS must be finite and non-negative");
		}

		if (!std::isfinite(wallClockTimeS) || wallClockTimeS <= 0)
		{
			throw std::invalid_argument("wallClockTimeS must be finite and positive");
		}

		return simulatedTimeS / wallClockTimeS;
	}
}