#include "BenchmarkSuites.h"

#include <performance/PerformanceTimer.h>

#include <simulation/DynamicsModel.h>
#include <simulation/EulerIntegrator.h>
#include <simulation/RK4Integrator.h>
#include <simulation/State.h>
#include <simulation/StateDerivative.h>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace aeroforge::performance
{
	namespace
	{
		constexpr std::size_t BATCH_COUNT = 10;
		constexpr std::size_t ITERATIONS_PER_BATCH = 100'000;
		constexpr double DT_S = 0.001;

		volatile double benchmarkSink = 0.0;

		class HarmonicOscillatorModel final
			: public simulation::DynamicsModel
		{
		public:
			simulation::StateDerivative evaluate(
				double,
				const simulation::State& state) const override
			{
				return simulation::StateDerivative{
					state[1],
					-state[0]
				};
			}
		};

		template <typename Operation>
		BenchmarkResult runBatchedBenchmark(
			const std::string& name,
			Operation operation)
		{
			double totalTimeS = 0.0;
			double minTimeS =
				std::numeric_limits<double>::max();
			double maxTimeS = 0.0;

			for (std::size_t batch = 0;
				batch < BATCH_COUNT;
				++batch)
			{
				PerformanceTimer timer;

				timer.start();

				const double sinkValue =
					operation(ITERATIONS_PER_BATCH);

				const double batchTimeS =
					timer.stop();

				benchmarkSink = sinkValue;

				const double batchMeanTimeS =
					batchTimeS /
					static_cast<double>(
						ITERATIONS_PER_BATCH);

				totalTimeS += batchTimeS;

				minTimeS =
					std::min(
						minTimeS,
						batchMeanTimeS);

				maxTimeS =
					std::max(
						maxTimeS,
						batchMeanTimeS);
			}

			const std::size_t totalIterations =
				BATCH_COUNT *
				ITERATIONS_PER_BATCH;

			const double meanTimeS =
				totalTimeS /
				static_cast<double>(
					totalIterations);

			const double operationsPerSecond =
				totalTimeS > 0.0
				? static_cast<double>(
					totalIterations) /
				totalTimeS
				: 0.0;

			return BenchmarkResult{
				name,
				totalIterations,
				totalTimeS,
				meanTimeS,
				minTimeS,
				maxTimeS,
				operationsPerSecond
			};
		}

		BenchmarkResult benchmarkEulerIntegrator()
		{
			return runBatchedBenchmark(
				"EulerIntegratorStep",
				[](std::size_t iterations)
				{
					HarmonicOscillatorModel model;
					simulation::EulerIntegrator integrator;

					simulation::State state{
						1.0,
						0.0
					};

					double timeS = 0.0;

					for (std::size_t i = 0;
						i < iterations;
						++i)
					{
						state =
							integrator.step(
								model,
								timeS,
								state,
								DT_S);

						timeS += DT_S;
					}

					return
						state[0] +
						state[1];
				});
		}

		BenchmarkResult benchmarkRK4Integrator()
		{
			return runBatchedBenchmark(
				"RK4IntegratorStep",
				[](std::size_t iterations)
				{
					HarmonicOscillatorModel model;
					simulation::RK4Integrator integrator;

					simulation::State state{
						1.0,
						0.0
					};

					double timeS = 0.0;

					for (std::size_t i = 0;
						i < iterations;
						++i)
					{
						state =
							integrator.step(
								model,
								timeS,
								state,
								DT_S);

						timeS += DT_S;
					}

					return
						state[0] +
						state[1];
				});
		}
	}

	std::vector<BenchmarkResult>
		runIntegratorBenchmarks()
	{
		std::vector<BenchmarkResult> results;

		results.reserve(2);

		results.push_back(
			benchmarkEulerIntegrator());

		results.push_back(
			benchmarkRK4Integrator());

		return results;
	}
}