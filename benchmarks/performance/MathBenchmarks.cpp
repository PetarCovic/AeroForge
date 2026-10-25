#include "BenchmarkSuites.h"

#include <performance/PerformanceTimer.h>

#include <math/Vector3.h>
#include <math/Quaternion.h>

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
		constexpr std::size_t ITERATIONS_PER_BATCH = 1'000'000;

		volatile double benchmarkSink = 0.0;

		template <typename Operation>
		BenchmarkResult runBatchedBenchmark(
			const std::string& name,
			Operation operation)
		{
			double totalTimeS = 0.0;
			double minTimeS = std::numeric_limits<double>::max();
			double maxTimeS = 0.0;

			for (std::size_t batch = 0; batch < BATCH_COUNT; ++batch)
			{
				PerformanceTimer timer;

				timer.start();

				const double sinkValue = operation(ITERATIONS_PER_BATCH);

				const double batchTimeS = timer.stop();

				// Prevent the optimizer from removing the benchmarked work.
				benchmarkSink = sinkValue;

				const double batchMeanTimeS =
					batchTimeS /
					static_cast<double>(ITERATIONS_PER_BATCH);

				totalTimeS += batchTimeS;

				minTimeS = std::min(
					minTimeS,
					batchMeanTimeS);

				maxTimeS = std::max(
					maxTimeS,
					batchMeanTimeS);
			}

			const std::size_t totalIterations =
				BATCH_COUNT * ITERATIONS_PER_BATCH;

			const double meanTimeS =
				totalTimeS /
				static_cast<double>(totalIterations);

			const double operationsPerSecond =
				totalTimeS > 0.0
				? static_cast<double>(totalIterations) / totalTimeS
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

		BenchmarkResult benchmarkVector3Magnitude()
		{
			return runBatchedBenchmark(
				"Vector3Magnitude",
				[](std::size_t iterations)
				{
					math::Vector3 vector(
						1.25,
						-4.8,
						9.1);

					double accumulator = 0.0;

					for (std::size_t i = 0; i < iterations; ++i)
					{
						const double magnitude = vector.magnitude();

						accumulator += magnitude;

						// Small state dependency prevents the optimizer
						// from replacing the loop with one constant result.
						vector.x += magnitude * 1.0e-12;
					}

					return accumulator;
				});
		}

		BenchmarkResult benchmarkVector3CrossProduct()
		{
			return runBatchedBenchmark(
				"Vector3CrossProduct",
				[](std::size_t iterations)
				{
					math::Vector3 a(
						1.25,
						-4.8,
						9.1);

					const math::Vector3 b(
						-2.7,
						8.3,
						0.65);

					double accumulator = 0.0;

					for (std::size_t i = 0; i < iterations; ++i)
					{
						const math::Vector3 cross = a.cross(b);

						accumulator +=
							cross.x +
							cross.y +
							cross.z;

						// Keep subsequent operations dependent on
						// previous results.
						a.x += cross.x * 1.0e-12;
						a.y += cross.y * 1.0e-12;
						a.z += cross.z * 1.0e-12;
					}

					return accumulator;
				});
		}

		BenchmarkResult benchmarkQuaternionMultiplication()
		{
			return runBatchedBenchmark(
				"QuaternionMultiplication",
				[](std::size_t iterations)
				{
					math::Quaternion quaternion(
						0.9238795325,
						0.0,
						0.3826834324,
						0.0);

					const math::Quaternion rotation(
						0.9999995,
						0.0003,
						0.0007,
						0.0006);

					for (std::size_t i = 0; i < iterations; ++i)
					{
						quaternion = quaternion * rotation;
					}

					return
						quaternion.w +
						quaternion.x +
						quaternion.y +
						quaternion.z;
				});
		}

		BenchmarkResult benchmarkQuaternionRotation()
		{
			return runBatchedBenchmark(
				"QuaternionRotation",
				[](std::size_t iterations)
				{
					const math::Quaternion quaternion =
						math::Quaternion(
							0.9238795325,
							0.0,
							0.3826834324,
							0.0)
						.normalized();

					math::Vector3 vector(
						15.0,
						-8.0,
						3.5);

					for (std::size_t i = 0; i < iterations; ++i)
					{
						vector = quaternion.rotate(vector);
					}

					return
						vector.x +
						vector.y +
						vector.z;
				});
		}
	}

	std::vector<BenchmarkResult> runMathBenchmarks()
	{
		std::vector<BenchmarkResult> results;

		results.reserve(4);

		results.push_back(
			benchmarkVector3Magnitude());

		results.push_back(
			benchmarkVector3CrossProduct());

		results.push_back(
			benchmarkQuaternionMultiplication());

		results.push_back(
			benchmarkQuaternionRotation());

		return results;
	}
}