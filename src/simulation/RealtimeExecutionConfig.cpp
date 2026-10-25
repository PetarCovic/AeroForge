#include <simulation/RealtimeExecutionConfig.h>

#include <stdexcept>
#include <cmath>

namespace aeroforge::simulation
{
	RealtimeExecutionConfig::RealtimeExecutionConfig(
		ExecutionMode mode,
		double simulationDtS,
		double durationS,
		double targetRateHz)
		: mode(mode), simulationDtS(simulationDtS), durationS(durationS), targetRateHz(targetRateHz)
	{
		if (mode == ExecutionMode::FixedRate)
		{

			if (targetRateHz <= 0)
			{
				throw std::invalid_argument("targetRateHz must be positive in FixedRate ExecutionMode");
			}
		}

		if (!std::isfinite(simulationDtS) || simulationDtS <= 0)
		{
			throw std::invalid_argument("simulationDtS must be finite and positive");
		}

		if (!std::isfinite(durationS) || durationS <= 0)
		{
			throw std::invalid_argument("durationS must be finite and positive");
		}

		if (!std::isfinite(targetRateHz))
		{
			throw std::invalid_argument("targetRateHz must be finite");
		}
	}

	ExecutionMode RealtimeExecutionConfig::getMode() const
	{
		return mode;
	}

	double RealtimeExecutionConfig::getSimulationDtS() const
	{
		return simulationDtS;
	}

	double RealtimeExecutionConfig::getDurationS() const
	{
		return durationS;
	}

	double RealtimeExecutionConfig::getTargetRateHz() const
	{
		return targetRateHz;
	}
}