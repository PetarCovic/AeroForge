#include <simulation/RealtimeSimulationRunner.h>

#include <simulation/ExecutionMode.h>
#include <simulation/SimulationPacer.h>

#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <limits>

namespace aeroforge::simulation
{
	RealtimeSimulationRunner::RealtimeSimulationRunner(
		const RealtimeExecutionConfig& config)
		: config(config)
	{
	}

	RealtimeExecutionResult RealtimeSimulationRunner::run(
		const StepFunction& stepFunction) const
	{
		if (!stepFunction)
		{
			throw std::invalid_argument(
				"Step function must not be empty");
		}

		std::size_t stepCount = 0;
		std::size_t deadlineMissCount = 0;

		double maximumOverrunS = 0.0;

		SimulationPacer pacer;
		pacer.start();

		const double durationS =
			config.getDurationS();

		const double simulationDtS =
			config.getSimulationDtS();

		const double timeToleranceS =
			std::numeric_limits<double>::epsilon() *
			std::max(
				std::abs(durationS),
				std::abs(simulationDtS)) *
			16.0;

		while (true)
		{
			const double simulationTimeS =
				static_cast<double>(stepCount) *
				simulationDtS;

			if (
				simulationTimeS >=
				durationS - timeToleranceS)
			{
				break;
			}

			const double remainingTimeS =
				durationS -
				simulationTimeS;

			const double actualDtS =
				std::min(
					simulationDtS,
					remainingTimeS);

			stepFunction(
				simulationTimeS,
				actualDtS);

			++stepCount;

			const double completedSimulationTimeS =
				std::min(
					durationS,
					simulationTimeS +
					actualDtS);

			double targetElapsedS = 0.0;
			bool shouldPace = false;

			if (
				config.getMode() ==
				ExecutionMode::Realtime)
			{
				targetElapsedS =
					completedSimulationTimeS;

				shouldPace = true;
			}
			else if (
				config.getMode() ==
				ExecutionMode::FixedRate)
			{
				targetElapsedS =
					static_cast<double>(stepCount) /
					config.getTargetRateHz();

				shouldPace = true;
			}

			if (shouldPace)
			{
				const double overrunS =
					pacer.paceTo(targetElapsedS);

				if (overrunS > 0.0)
				{
					++deadlineMissCount;

					maximumOverrunS =
						std::max(
							maximumOverrunS,
							overrunS);
				}
			}
		}

		const double simulatedTimeS =
			durationS;

		const double wallTimeS =
			pacer.getElapsedWallTimeS();

		const double realtimeFactor =
			simulatedTimeS /
			wallTimeS;

		return RealtimeExecutionResult{
			simulatedTimeS,
			wallTimeS,
			realtimeFactor,
			stepCount,
			deadlineMissCount,
			maximumOverrunS
		};
	}
}