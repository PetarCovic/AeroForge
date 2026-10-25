#include "BenchmarkSuites.h"

#include <performance/PerformanceTimer.h>

#include <dynamics/ForceAccumulator.h>
#include <dynamics/InertiaProperties.h>
#include <dynamics/MassProperties.h>
#include <dynamics/MomentAccumulator.h>
#include <dynamics/RigidBodyState.h>
#include <dynamics/SixDoFDynamicsModel.h>

#include <math/Matrix.h>
#include <math/Quaternion.h>
#include <math/Vector3.h>

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

		volatile double benchmarkSink = 0.0;

		template <typename Operation>
		BenchmarkResult runBatchedBenchmark(
			const std::string& name,
			std::size_t iterationsPerBatch,
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
					operation(iterationsPerBatch);

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
				BATCH_COUNT *
				iterationsPerBatch;

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

		BenchmarkResult benchmarkSixDoFEvaluate()
		{
			return runBatchedBenchmark(
				"SixDoFDynamicsEvaluate",
				100'000,
				[](std::size_t iterations)
				{
					dynamics::ForceAccumulator forces;

					forces.addForce(
						math::Vector3(
							25.0,
							-3.0,
							-98.1));

					dynamics::MomentAccumulator moments;

					moments.addMoment(
						math::Vector3(
							0.5,
							-0.25,
							0.1));

					dynamics::MassProperties massProperties(
						10.0);

					const math::Matrix inertiaTensor =
						math::Matrix::diagonal(
							2.0,
							3);

					dynamics::InertiaProperties
						inertiaProperties(
							inertiaTensor);

					dynamics::SixDoFDynamicsModel model(
						forces,
						moments,
						massProperties,
						inertiaProperties);

					dynamics::RigidBodyState rigidBodyState(
						math::Vector3(
							0.0,
							0.0,
							-1000.0),
						math::Vector3(
							120.0,
							5.0,
							-2.0),
						math::Quaternion(
							1.0,
							0.0,
							0.0,
							0.0),
						math::Vector3(
							0.01,
							-0.02,
							0.03));

					simulation::State state =
						rigidBodyState.toStateVector();

					double accumulator = 0.0;

					for (std::size_t i = 0;
						i < iterations;
						++i)
					{
						const simulation::StateDerivative derivative =
							model.evaluate(
								static_cast<double>(i) *
								0.001,
								state);

						accumulator +=
							derivative[0] +
							derivative[3] +
							derivative[6] +
							derivative[10];

						// Make later evaluations depend on
						// previous calculations.
						state[3] +=
							derivative[3] *
							1.0e-12;

						state[10] +=
							derivative[10] *
							1.0e-12;
					}

					return accumulator;
				});
		}

		BenchmarkResult benchmarkSixDoFRK4Step()
		{
			return runBatchedBenchmark(
				"SixDoFRK4Step",
				10'000,
				[](std::size_t iterations)
				{
					dynamics::ForceAccumulator forces;

					forces.addForce(
						math::Vector3(
							25.0,
							-3.0,
							-98.1));

					dynamics::MomentAccumulator moments;

					moments.addMoment(
						math::Vector3(
							0.5,
							-0.25,
							0.1));

					dynamics::MassProperties massProperties(
						10.0);

					const math::Matrix inertiaTensor =
						math::Matrix::diagonal(
							2.0,
							3);

					dynamics::InertiaProperties
						inertiaProperties(
							inertiaTensor);

					dynamics::SixDoFDynamicsModel model(
						forces,
						moments,
						massProperties,
						inertiaProperties);

					simulation::RK4Integrator integrator;

					dynamics::RigidBodyState rigidBodyState(
						math::Vector3(
							0.0,
							0.0,
							-1000.0),
						math::Vector3(
							120.0,
							5.0,
							-2.0),
						math::Quaternion(
							1.0,
							0.0,
							0.0,
							0.0),
						math::Vector3(
							0.01,
							-0.02,
							0.03));

					simulation::State state =
						rigidBodyState.toStateVector();

					double timeS = 0.0;

					constexpr double dtS = 0.0001;

					for (std::size_t i = 0;
						i < iterations;
						++i)
					{
						state =
							integrator.step(
								model,
								timeS,
								state,
								dtS);

						timeS += dtS;
					}

					return
						state[0] +
						state[3] +
						state[6] +
						state[10];
				});
		}
	}

	std::vector<BenchmarkResult>
		runSixDoFBenchmarks()
	{
		std::vector<BenchmarkResult> results;

		results.reserve(2);

		results.push_back(
			benchmarkSixDoFEvaluate());

		results.push_back(
			benchmarkSixDoFRK4Step());

		return results;
	}
}