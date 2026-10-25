#pragma once

#include <simulation/ExecutionMode.h>

namespace aeroforge::simulation
{
	class RealtimeExecutionConfig
	{
	public:
		RealtimeExecutionConfig(
			ExecutionMode mode,
			double simulationDtS,
			double durationS,
			double targetRateHz);

		ExecutionMode getMode() const;

		double getSimulationDtS() const;

		double getDurationS() const;

		double getTargetRateHz() const;

	private:
		ExecutionMode mode;
		double simulationDtS;
		double durationS;
		double targetRateHz;
	};
}