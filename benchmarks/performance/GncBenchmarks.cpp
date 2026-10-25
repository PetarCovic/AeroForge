#include "BenchmarkSuites.h"

#include <performance/PerformanceTimer.h>

#include <gnc/AttitudeError.h>
#include <gnc/AttitudePdController.h>
#include <gnc/AttitudeSetpoint.h>
#include <gnc/HoldAttitudeGuidance.h>

#include <navigation/InertialNavigator.h>
#include <navigation/NavigationState.h>

#include <sensors/ImuModel.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

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
		constexpr std::size_t ITERATIONS_PER_BATCH =
			100'000;

		volatile double benchmarkSink = 0.0;

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
					operation(
						ITERATIONS_PER_BATCH);

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

		BenchmarkResult benchmarkAttitudeError()
		{
			return runBatchedBenchmark(
				"AttitudeError",
				[](std::size_t iterations)
				{
					const math::Quaternion desiredAttitude =
						math::Quaternion(
							0.9238795325,
							0.0,
							0.3826834324,
							0.0)
						.normalized();

					math::Quaternion currentAttitude =
						math::Quaternion(
							0.9914448614,
							0.1305261922,
							0.0,
							0.0)
						.normalized();

					double accumulator = 0.0;

					for (std::size_t i = 0;
						i < iterations;
						++i)
					{
						const math::Vector3 error =
							gnc::computeAttitudeErrorVectorB(
								desiredAttitude,
								currentAttitude);

						accumulator +=
							error.x +
							error.y +
							error.z;

						currentAttitude =
							math::Quaternion(
								currentAttitude.w,
								currentAttitude.x +
								error.x *
								1.0e-12,
								currentAttitude.y +
								error.y *
								1.0e-12,
								currentAttitude.z +
								error.z *
								1.0e-12)
							.normalized();
					}

					return accumulator;
				});
		}

		BenchmarkResult benchmarkAttitudeController()
		{
			return runBatchedBenchmark(
				"AttitudePdController",
				[](std::size_t iterations)
				{
					const gnc::AttitudeSetpoint setpoint(
						math::Quaternion(
							0.9238795325,
							0.0,
							0.3826834324,
							0.0)
						.normalized(),
						math::Vector3(
							0.0,
							0.0,
							0.0));

					const gnc::AttitudePdController controller(
						math::Vector3(
							8.0,
							8.0,
							5.0),
						math::Vector3(
							1.5,
							1.5,
							1.0),
						math::Vector3(
							20.0,
							20.0,
							10.0));

					navigation::NavigationState state(
						math::Vector3(
							0.0,
							0.0,
							-500.0),
						math::Vector3(
							100.0,
							0.0,
							0.0),
						math::Quaternion(
							1.0,
							0.0,
							0.0,
							0.0),
						math::Vector3(
							0.01,
							-0.02,
							0.03));

					double accumulator = 0.0;

					for (std::size_t i = 0;
						i < iterations;
						++i)
					{
						const math::Vector3 command =
							controller.computeMomentCommandB(
								setpoint,
								state);

						accumulator +=
							command.x +
							command.y +
							command.z;

						const math::Vector3 omega =
							state.getAngularVelocityB();

						state =
							navigation::NavigationState(
								state.getPositionN(),
								state.getVelocityN(),
								state.getAttitudeNB(),
								math::Vector3(
									omega.x +
									command.x *
									1.0e-12,
									omega.y +
									command.y *
									1.0e-12,
									omega.z +
									command.z *
									1.0e-12));
					}

					return accumulator;
				});
		}

		BenchmarkResult benchmarkInertialNavigator()
		{
			return runBatchedBenchmark(
				"InertialNavigator",
				[](std::size_t iterations)
				{
					const navigation::InertialNavigator
						navigator;

					navigation::NavigationState state(
						math::Vector3(
							0.0,
							0.0,
							-500.0),
						math::Vector3(
							100.0,
							2.0,
							-1.0),
						math::Quaternion(
							1.0,
							0.0,
							0.0,
							0.0),
						math::Vector3(
							0.01,
							-0.02,
							0.03));

					const sensors::ImuMeasurement imu{
						0.0,
						math::Vector3(
							0.2,
							-0.1,
							-9.7),
						math::Vector3(
							0.01,
							-0.02,
							0.03)
					};

					const math::Vector3 gravityN(
						0.0,
						0.0,
						9.80665);

					constexpr double dtS = 0.001;

					for (std::size_t i = 0;
						i < iterations;
						++i)
					{
						state =
							navigator.propagate(
								state,
								imu,
								gravityN,
								dtS);
					}

					return
						state.getPositionN().x +
						state.getVelocityN().x +
						state.getAttitudeNB().w;
				});
		}

		BenchmarkResult benchmarkGncCycle()
		{
			return runBatchedBenchmark(
				"GncCycle",
				[](std::size_t iterations)
				{
					const gnc::AttitudeSetpoint setpoint(
						math::Quaternion(
							0.9238795325,
							0.0,
							0.3826834324,
							0.0)
						.normalized(),
						math::Vector3(
							0.0,
							0.0,
							0.0));

					const gnc::HoldAttitudeGuidance guidance(
						setpoint);

					const gnc::AttitudePdController controller(
						math::Vector3(
							8.0,
							8.0,
							5.0),
						math::Vector3(
							1.5,
							1.5,
							1.0),
						math::Vector3(
							20.0,
							20.0,
							10.0));

					const navigation::InertialNavigator navigator;

					navigation::NavigationState state(
						math::Vector3(
							0.0,
							0.0,
							-500.0),
						math::Vector3(
							100.0,
							2.0,
							-1.0),
						math::Quaternion(
							1.0,
							0.0,
							0.0,
							0.0),
						math::Vector3(
							0.01,
							-0.02,
							0.03));

					const sensors::ImuMeasurement imu{
						0.0,
						math::Vector3(
							0.2,
							-0.1,
							-9.7),
						math::Vector3(
							0.01,
							-0.02,
							0.03)
					};

					const math::Vector3 gravityN(
						0.0,
						0.0,
						9.80665);

					constexpr double dtS = 0.001;

					double timeS = 0.0;
					double accumulator = 0.0;

					for (std::size_t i = 0;
						i < iterations;
						++i)
					{
						const gnc::AttitudeSetpoint
							currentSetpoint =
							guidance.getSetpoint(
								timeS);

						const math::Vector3 command =
							controller.computeMomentCommandB(
								currentSetpoint,
								state);

						state =
							navigator.propagate(
								state,
								imu,
								gravityN,
								dtS);

						accumulator +=
							command.x +
							command.y +
							command.z;

						timeS += dtS;
					}

					return
						accumulator +
						state.getPositionN().x;
				});
		}
	}

	std::vector<BenchmarkResult>
		runGncBenchmarks()
	{
		std::vector<BenchmarkResult> results;

		results.reserve(4);

		results.push_back(
			benchmarkAttitudeError());

		results.push_back(
			benchmarkAttitudeController());

		results.push_back(
			benchmarkInertialNavigator());

		results.push_back(
			benchmarkGncCycle());

		return results;
	}
}