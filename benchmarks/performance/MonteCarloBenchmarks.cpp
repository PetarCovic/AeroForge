#include "BenchmarkSuites.h"

#include <performance/PerformanceTimer.h>

#include <montecarlo/MonteCarloConfig.h>
#include <montecarlo/MonteCarloRunner.h>
#include <montecarlo/MonteCarloTypes.h>
#include <montecarlo/ParameterDistribution.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace aeroforge::performance
{
	namespace
	{
		volatile double benchmarkSink = 0.0;

		template <typename Operation>
		BenchmarkResult runBatchedBenchmark(
			const std::string& name,
			std::size_t batchCount,
			std::size_t iterationsPerBatch,
			Operation operation)
		{
			double totalTimeS = 0.0;
			double minTimeS =
				std::numeric_limits<double>::max();
			double maxTimeS = 0.0;

			for (std::size_t batch = 0;
				batch < batchCount;
				++batch)
			{
				PerformanceTimer timer;

				timer.start();

				const double sinkValue =
					operation(
						iterationsPerBatch);

				const double batchTimeS =
					timer.stop();

				benchmarkSink = sinkValue;

				const double batchMeanTimeS =
					batchTimeS /
					static_cast<double>(
						iterationsPerBatch);

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
				batchCount *
				iterationsPerBatch;

			return BenchmarkResult{
				name,
				totalIterations,
				totalTimeS,
				totalTimeS /
					static_cast<double>(
						totalIterations),
				minTimeS,
				maxTimeS,
				totalTimeS > 0.0
					? static_cast<double>(
						totalIterations) /
						totalTimeS
					: 0.0
			};
		}

		BenchmarkResult benchmarkParameterSampling()
		{
			return runBatchedBenchmark(
				"MonteCarloParameterSampling",
				10,
				100'000,
				[](std::size_t iterations)
				{
					std::mt19937 randomEngine(
						123456U);

					const montecarlo::ParameterDistribution
						distribution =
						montecarlo::
						ParameterDistribution::
						normal(
							100.0,
							5.0);

					double accumulator = 0.0;

					for (std::size_t i = 0;
						i < iterations;
						++i)
					{
						accumulator +=
							distribution.sample(
								randomEngine);
					}

					return accumulator;
				});
		}

		BenchmarkResult benchmarkMonteCarloRunner()
		{
			return runBatchedBenchmark(
				"MonteCarloRunner",
				10,
				1'000,
				[](std::size_t trialCount)
				{
					montecarlo::MonteCarloConfig config(
						trialCount,
						123456U);

					config.addParameter(
						"massKg",
						montecarlo::
						ParameterDistribution::
						normal(
							100.0,
							5.0));

					config.addParameter(
						"thrustScale",
						montecarlo::
						ParameterDistribution::
						uniform(
							0.95,
							1.05));

					config.addParameter(
						"dragScale",
						montecarlo::
						ParameterDistribution::
						uniform(
							0.9,
							1.1));

					montecarlo::MonteCarloRunner runner(
						config);

					const std::vector<
						montecarlo::MonteCarloResult>
						results =
						runner.run(
							[](
								const montecarlo::
								MonteCarloTrial&
								trial)
							{
								const double massKg =
									trial.parameters.at(
										"massKg");

								const double thrustScale =
									trial.parameters.at(
										"thrustScale");

								const double dragScale =
									trial.parameters.at(
										"dragScale");

								const double metric =
									(thrustScale /
										dragScale) *
									massKg;

								return montecarlo::
									MonteCarloResult{
										trial.trialIndex,
										true,
										{
											{
												"performanceMetric",
												metric
											}
										}
								};
							});

					if (results.empty())
					{
						return 0.0;
					}

					return
						results.back()
						.metrics.at(
							"performanceMetric") +
						static_cast<double>(
							results.size());
				});
		}
	}

	std::vector<BenchmarkResult>
		runMonteCarloBenchmarks()
	{
		std::vector<BenchmarkResult> results;

		results.reserve(2);

		results.push_back(
			benchmarkParameterSampling());

		results.push_back(
			benchmarkMonteCarloRunner());

		return results;
	}
}