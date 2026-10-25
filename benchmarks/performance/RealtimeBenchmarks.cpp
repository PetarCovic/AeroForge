#include "BenchmarkSuites.h"

#include <performance/PerformanceTimer.h>

#include <dynamics/ForceAccumulator.h>
#include <dynamics/InertiaProperties.h>
#include <dynamics/MassProperties.h>
#include <dynamics/MomentAccumulator.h>
#include <dynamics/SixDoFDynamicsModel.h>

#include <math/Matrix.h>
#include <math/Vector3.h>

#include <simulation/ExecutionMode.h>
#include <simulation/RealtimeExecutionConfig.h>
#include <simulation/RealtimeSimulationRunner.h>
#include <simulation/RK4Integrator.h>
#include <simulation/State.h>

#include <gnc/AttitudeError.h>
#include <gnc/AttitudePdController.h>
#include <gnc/AttitudeSetpoint.h>
#include <gnc/HoldAttitudeGuidance.h>

#include <navigation/GpsCorrector.h>
#include <navigation/InertialNavigator.h>
#include <navigation/NavigationState.h>

#include <sensors/AccelerometerModel.h>
#include <sensors/GpsModel.h>
#include <sensors/GyroscopeModel.h>
#include <sensors/ImuModel.h>

#include <telemetry/TelemetryRecorder.h>
#include <telemetry/TelemetrySample.h>

#include <actuators/BodyMomentActuator.h>

#include <math/Quaternion.h>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>
#include <chrono>
#include <cmath>
#include <cstdint>

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

				for (std::size_t iteration = 0;
					iteration < iterationsPerBatch;
					++iteration)
				{
					operation();
				}

				const double batchTimeS =
					timer.stop();

				const double timePerOperationS =
					batchTimeS /
					static_cast<double>(iterationsPerBatch);

				totalTimeS += batchTimeS;

				minTimeS =
					std::min(
						minTimeS,
						timePerOperationS);

				maxTimeS =
					std::max(
						maxTimeS,
						timePerOperationS);
			}

			const std::size_t totalIterations =
				BATCH_COUNT *
				iterationsPerBatch;

			const double meanTimeS =
				totalTimeS /
				static_cast<double>(totalIterations);

			const double operationsPerSecond =
				static_cast<double>(totalIterations) /
				totalTimeS;

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

		simulation::State makeInitialSixDoFState()
		{
			return simulation::State{
				0.0,
				0.0,
				-1000.0,

				120.0,
				5.0,
				-2.0,

				1.0,
				0.0,
				0.0,
				0.0,

				0.01,
				-0.02,
				0.03
			};
		}

		enum class NominalStackLevel
		{
			SixDoF = 0,
			Sensors = 1,
			Navigation = 2,
			Gnc = 3,
			Actuators = 4,
			Telemetry = 5
		};

		struct NominalFlightRunResult
		{
			simulation::RealtimeExecutionResult executionResult;

			double sinkValue;

			double firstWindowMeanStepTimeS;
			double finalWindowMeanStepTimeS;
			double timingDegradationFraction;
		};

		bool includesStackLevel(
			NominalStackLevel level,
			NominalStackLevel requiredLevel)
		{
			return
				static_cast<int>(level) >=
				static_cast<int>(requiredLevel);
		}

		simulation::State makeNominalFlightState()
		{
			return simulation::State{
				0.0,
				0.0,
				-1000.0,

				120.0,
				0.0,
				0.0,

				1.0,
				0.0,
				0.0,
				0.0,

				0.0,
				0.0,
				0.0
			};
		}

		math::Vector3 getPositionN(
			const simulation::State& state)
		{
			return math::Vector3(
				state[0],
				state[1],
				state[2]);
		}

		math::Vector3 getVelocityN(
			const simulation::State& state)
		{
			return math::Vector3(
				state[3],
				state[4],
				state[5]);
		}

		math::Quaternion getAttitudeNB(
			const simulation::State& state)
		{
			return math::Quaternion(
				state[6],
				state[7],
				state[8],
				state[9]);
		}

		math::Vector3 getAngularVelocityB(
			const simulation::State& state)
		{
			return math::Vector3(
				state[10],
				state[11],
				state[12]);
		}

		NominalFlightRunResult runNominalFlightStack(
			NominalStackLevel stackLevel,
			double durationS,
			bool collectStabilityMetrics)
		{
			constexpr double PhysicsDtS = 0.001;

			constexpr std::size_t ImuStepInterval = 5;
			constexpr std::size_t GpsStepInterval = 100;
			constexpr std::size_t GncStepInterval = 10;
			constexpr std::size_t TelemetryStepInterval = 10;

			constexpr std::size_t StabilityWindowStepCount = 1000;

			dynamics::ForceAccumulator forces;

			forces.addForce(
				math::Vector3(
					25.0,
					-3.0,
					0.0));

			dynamics::MomentAccumulator moments;

			const dynamics::MassProperties massProperties(
				10.0);

			const math::Matrix inertiaTensor(
				3,
				3,
				{
					2.0, 0.0, 0.0,
					0.0, 3.0, 0.0,
					0.0, 0.0, 4.0
				});

			const dynamics::InertiaProperties inertiaProperties(
				inertiaTensor);

			const dynamics::SixDoFDynamicsModel model(
				forces,
				moments,
				massProperties,
				inertiaProperties);

			const simulation::RK4Integrator integrator;

			const math::Vector3 zeroVector(
				0.0,
				0.0,
				0.0);

			const sensors::AccelerometerModel accelerometer(
				zeroVector,
				zeroVector,
				math::Vector3(
					100.0,
					100.0,
					100.0),
				static_cast<std::uint32_t>(1001));

			const sensors::GyroscopeModel gyroscope(
				zeroVector,
				zeroVector,
				math::Vector3(
					20.0,
					20.0,
					20.0),
				static_cast<std::uint32_t>(1002));

			sensors::ImuModel imu(
				accelerometer,
				gyroscope);

			sensors::GpsModel gps(
				zeroVector,
				zeroVector,
				zeroVector,
				zeroVector,
				static_cast<std::uint32_t>(1003));

			const navigation::InertialNavigator navigator;

			const navigation::GpsCorrector gpsCorrector(
				1.0,
				1.0);

			const gnc::AttitudeSetpoint heldSetpoint(
				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0),
				zeroVector);

			const gnc::HoldAttitudeGuidance guidance(
				heldSetpoint);

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

			actuators::BodyMomentActuator bodyMomentActuator(
				math::Vector3(
					20.0,
					20.0,
					10.0),
				math::Vector3(
					100.0,
					100.0,
					100.0));

			telemetry::TelemetryRecorder telemetryRecorder;

			simulation::State state =
				makeNominalFlightState();

			navigation::NavigationState navigationState(
				getPositionN(state),
				getVelocityN(state),
				getAttitudeNB(state),
				getAngularVelocityB(state));

			const math::Vector3 gravityN(
				0.0,
				0.0,
				9.80665);

			const math::Vector3 trueSpecificForceB(
				2.5,
				-0.3,
				0.0);

			auto latestImuMeasurement =
				imu.sample(
					0.0,
					trueSpecificForceB,
					getAngularVelocityB(state));

			gnc::AttitudeSetpoint activeSetpoint =
				guidance.getSetpoint(0.0);

			math::Vector3 commandedMomentB(
				0.0,
				0.0,
				0.0);

			math::Vector3 actualMomentB(
				0.0,
				0.0,
				0.0);

			std::size_t stepIndex = 0;

			const std::size_t expectedStepCount =
				static_cast<std::size_t>(
					std::llround(
						durationS /
						PhysicsDtS));

			const std::size_t finalWindowStart =
				expectedStepCount >
				StabilityWindowStepCount
				? expectedStepCount -
				StabilityWindowStepCount
				: 0;

			double firstWindowTotalStepTimeS = 0.0;
			double finalWindowTotalStepTimeS = 0.0;

			std::size_t firstWindowSamples = 0;
			std::size_t finalWindowSamples = 0;

			const simulation::RealtimeExecutionConfig config(
				simulation::ExecutionMode::Unpaced,
				PhysicsDtS,
				durationS,
				0.0);

			const simulation::RealtimeSimulationRunner runner(
				config);

			const simulation::RealtimeExecutionResult executionResult =
				runner.run(
					[
						&state,
						&navigationState,
						&latestImuMeasurement,
						&activeSetpoint,
						&commandedMomentB,
						&actualMomentB,
						&stepIndex,
						&firstWindowTotalStepTimeS,
						&finalWindowTotalStepTimeS,
						&firstWindowSamples,
						&finalWindowSamples,
						finalWindowStart,
						collectStabilityMetrics,
						stackLevel,
						&imu,
						&gps,
						&navigator,
						&gpsCorrector,
						&guidance,
						&controller,
						&bodyMomentActuator,
						&telemetryRecorder,
						&moments,
						&integrator,
						&model,
						&gravityN,
						&trueSpecificForceB
					](
						double simulationTimeS,
						double dtS)
			{
				const auto stepStart =
					std::chrono::steady_clock::now();

				const math::Vector3 truthPositionN =
					getPositionN(state);

				const math::Vector3 truthVelocityN =
					getVelocityN(state);

				const math::Vector3 truthAngularVelocityB =
					getAngularVelocityB(state);

				if (
					includesStackLevel(
						stackLevel,
						NominalStackLevel::Sensors))
				{
					if (
						stepIndex %
						ImuStepInterval ==
						0)
					{
						latestImuMeasurement =
							imu.sample(
								simulationTimeS,
								trueSpecificForceB,
								truthAngularVelocityB);

						if (
							includesStackLevel(
								stackLevel,
								NominalStackLevel::Navigation))
						{
							navigationState =
								navigator.propagate(
									navigationState,
									latestImuMeasurement,
									gravityN,
									dtS * static_cast<double>(ImuStepInterval));
						}
					}

					if (
						stepIndex %
						GpsStepInterval ==
						0)
					{
						const auto gpsMeasurement =
							gps.sample(
								simulationTimeS,
								truthPositionN,
								truthVelocityN);

						if (
							includesStackLevel(
								stackLevel,
								NominalStackLevel::Navigation))
						{
							navigationState =
								gpsCorrector.correct(
									navigationState,
									gpsMeasurement);
						}
					}
				}

				if (
					includesStackLevel(
						stackLevel,
						NominalStackLevel::Gnc) &&
					stepIndex %
					GncStepInterval ==
					0)
				{
					activeSetpoint =
						guidance.getSetpoint(
							simulationTimeS);

					commandedMomentB =
						controller.computeMomentCommandB(
							activeSetpoint,
							navigationState);
				}

				if (
					includesStackLevel(
						stackLevel,
						NominalStackLevel::Actuators))
				{
					bodyMomentActuator.update(
						commandedMomentB,
						dtS);

					actualMomentB =
						bodyMomentActuator.getActualMomentB();

					moments.clear();

					moments.addMoment(
						actualMomentB);
				}

				state =
					integrator.step(
						model,
						simulationTimeS,
						state,
						dtS);

				if (
					includesStackLevel(
						stackLevel,
						NominalStackLevel::Telemetry) &&
					stepIndex %
					TelemetryStepInterval ==
					0)
				{
					const math::Vector3 attitudeErrorB =
						gnc::computeAttitudeErrorVectorB(
							activeSetpoint.getAttitudeNB(),
							navigationState.getAttitudeNB());

					telemetryRecorder.record(
						telemetry::TelemetrySample{
							simulationTimeS + dtS,

							getPositionN(state),
							getVelocityN(state),
							getAttitudeNB(state),
							getAngularVelocityB(state),

							navigationState.getPositionN(),
							navigationState.getVelocityN(),
							navigationState.getAttitudeNB(),
							navigationState.getAngularVelocityB(),

							attitudeErrorB,

							commandedMomentB,
							actualMomentB,

							1.0,
							0.0,
							0.0,

							false
						});
				}

				if (collectStabilityMetrics)
				{
					const auto stepEnd =
						std::chrono::steady_clock::now();

					const double stepTimeS =
						std::chrono::duration<double>(
							stepEnd -
							stepStart).count();

					if (
						stepIndex <
						StabilityWindowStepCount)
					{
						firstWindowTotalStepTimeS +=
							stepTimeS;

						++firstWindowSamples;
					}

					if (
						stepIndex >=
						finalWindowStart)
					{
						finalWindowTotalStepTimeS +=
							stepTimeS;

						++finalWindowSamples;
					}
				}

				++stepIndex;
			});

			const double firstWindowMeanStepTimeS =
				firstWindowSamples > 0
				? firstWindowTotalStepTimeS /
				static_cast<double>(
					firstWindowSamples)
				: 0.0;

			const double finalWindowMeanStepTimeS =
				finalWindowSamples > 0
				? finalWindowTotalStepTimeS /
				static_cast<double>(
					finalWindowSamples)
				: 0.0;

			const double timingDegradationFraction =
				firstWindowMeanStepTimeS > 0.0
				? (
					finalWindowMeanStepTimeS -
					firstWindowMeanStepTimeS) /
				firstWindowMeanStepTimeS
				: 0.0;

			const double sinkValue =
				state[0] +
				state[1] +
				state[2] +
				static_cast<double>(
					telemetryRecorder.size()) +
				executionResult.realtimeFactor;

			return NominalFlightRunResult{
				executionResult,
				sinkValue,
				firstWindowMeanStepTimeS,
				finalWindowMeanStepTimeS,
				timingDegradationFraction
			};
		}
	}

	RealtimeBenchmarkSuiteResult runRealtimeBenchmarks()
	{
		RealtimeBenchmarkSuiteResult suiteResult;

		suiteResult.benchmarks.reserve(13);
		suiteResult.realtimeMetrics.reserve(10);

		/*
		 * ------------------------------------------------------------
		 * Unpaced runner overhead
		 *
		 * 100 simulated seconds
		 * dt = 0.001 s
		 * 100,000 callbacks per complete simulation run
		 * ------------------------------------------------------------
		 */
		{
			const simulation::RealtimeExecutionConfig config(
				simulation::ExecutionMode::Unpaced,
				0.001,
				100.0,
				0.0);

			const simulation::RealtimeSimulationRunner runner(
				config);

			suiteResult.benchmarks.push_back(
				runBatchedBenchmark(
					"UnpacedRunnerEmptyStep",
					10,
					[&runner]()
					{
						double localSink = 0.0;

						const simulation::RealtimeExecutionResult result =
							runner.run(
								[&localSink](
									double simulationTimeS,
									double dtS)
								{
									localSink +=
										simulationTimeS *
										dtS;
								});

						benchmarkSink =
							benchmarkSink +
							localSink +
							result.realtimeFactor;
					}));
		}

		/*
		 * ------------------------------------------------------------
		 * Unpaced runner with light deterministic computation
		 *
		 * Same runner configuration as the empty-step benchmark.
		 * ------------------------------------------------------------
		 */
		{
			const simulation::RealtimeExecutionConfig config(
				simulation::ExecutionMode::Unpaced,
				0.001,
				100.0,
				0.0);

			const simulation::RealtimeSimulationRunner runner(
				config);

			suiteResult.benchmarks.push_back(
				runBatchedBenchmark(
					"UnpacedRunnerLightStep",
					10,
					[&runner]()
					{
						double value = 1.0;

						const simulation::RealtimeExecutionResult result =
							runner.run(
								[&value](
									double simulationTimeS,
									double dtS)
								{
									value +=
										0.25 * dtS;

									value *=
										1.0 +
										1.0e-7 *
										simulationTimeS;

									value -=
										0.10 * dtS;
								});

						benchmarkSink =
							benchmarkSink +
							value +
							result.realtimeFactor;
					}));
		}

		/*
		 * ------------------------------------------------------------
		 * Full 6DOF RK4 propagation
		 *
		 * 10 simulated seconds
		 * dt = 0.0001 s
		 * 100,000 RK4 integration steps per complete simulation run
		 * ------------------------------------------------------------
		 */
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

			const dynamics::MassProperties massProperties(
				10.0);

			const math::Matrix inertiaTensor(
				3,
				3,
				{
					2.0, 0.0, 0.0,
					0.0, 3.0, 0.0,
					0.0, 0.0, 4.0
				});

			const dynamics::InertiaProperties inertiaProperties(
				inertiaTensor);

			const dynamics::SixDoFDynamicsModel model(
				forces,
				moments,
				massProperties,
				inertiaProperties);

			const simulation::RK4Integrator integrator;

			const simulation::RealtimeExecutionConfig config(
				simulation::ExecutionMode::Unpaced,
				0.0001,
				10.0,
				0.0);

			const simulation::RealtimeSimulationRunner runner(
				config);

			suiteResult.benchmarks.push_back(
				runBatchedBenchmark(
					"UnpacedRunnerSixDoFRK4",
					1,
					[
						&runner,
						&integrator,
						&model
					]()
					{
						simulation::State state =
							makeInitialSixDoFState();

						const simulation::RealtimeExecutionResult result =
							runner.run(
								[
									&state,
									&integrator,
									&model
								](
									double simulationTimeS,
									double dtS)
								{
									state =
										integrator.step(
											model,
											simulationTimeS,
											state,
											dtS);
								});

						benchmarkSink =
							benchmarkSink +
							state[0] +
							state[1] +
							state[2] +
							result.realtimeFactor;
					}));
		}

		/*
		 * ------------------------------------------------------------
		 * Realtime Runner on Schedule
		 *
		 * 1 simulated second
		 * dt = 0.001 s
		 * 1,000 simulation steps
		 * Lightweight callback
		 * ------------------------------------------------------------
		 */
		{
			const simulation::RealtimeExecutionConfig config(
				simulation::ExecutionMode::Realtime,
				0.001,
				1.0,
				0.0);

			const simulation::RealtimeSimulationRunner runner(config);

			double totalSimulatedTimeS = 0.0;
			double totalWallTimeS = 0.0;

			std::size_t totalStepCount = 0;
			std::size_t totalDeadlineMissCount = 0;

			double maximumOverrunS = 0.0;

			suiteResult.benchmarks.push_back(
				runBatchedBenchmark(
					"RealtimeRunnerOnSchedule",
					1,
					[
						&runner,
						&totalSimulatedTimeS,
						&totalWallTimeS,
						&totalStepCount,
						&totalDeadlineMissCount,
						&maximumOverrunS
					]()
					{
						const simulation::RealtimeExecutionResult executionResult =
							runner.run(
								[](double simulationTimeS, double dtS)
								{
									benchmarkSink =
										benchmarkSink +
										simulationTimeS * dtS;
								});

						totalSimulatedTimeS +=
							executionResult.simulatedTimeS;

						totalWallTimeS +=
							executionResult.wallTimeS;

						totalStepCount +=
							executionResult.stepCount;

						totalDeadlineMissCount +=
							executionResult.deadlineMissCount;

						maximumOverrunS =
							std::max(
								maximumOverrunS,
								executionResult.maximumOverrunS);

						benchmarkSink =
							benchmarkSink +
							executionResult.realtimeFactor +
							executionResult.maximumOverrunS;
					}));

			suiteResult.realtimeMetrics.push_back(
				RealtimeBenchmarkResult{
					"RealtimeRunnerOnSchedule",
					totalSimulatedTimeS,
					totalWallTimeS,
					totalSimulatedTimeS / totalWallTimeS,
					totalStepCount,
					totalDeadlineMissCount,
					maximumOverrunS
				});
		}

		/*
		 * ------------------------------------------------------------
		 * Realtime Runner Overloaded
		 *
		 * 0.1 simulated seconds
		 * dt = 0.001 s
		 * 100 simulation steps
		 * Each step deliberately takes about 2 ms
		 * Step budget is only 1 ms
		 *
		 * Expected behavior:
		 * - simulation falls behind real time
		 * - deadlineMissCount > 0
		 * - maximumOverrunS > 0
		 * - realtimeFactor < 1.0
		 * ------------------------------------------------------------
		 */
		{
			const simulation::RealtimeExecutionConfig config(
				simulation::ExecutionMode::Realtime,
				0.001,
				0.1,
				0.0);

			const simulation::RealtimeSimulationRunner runner(config);

			double totalSimulatedTimeS = 0.0;
			double totalWallTimeS = 0.0;

			std::size_t totalStepCount = 0;
			std::size_t totalDeadlineMissCount = 0;

			double maximumOverrunS = 0.0;

			suiteResult.benchmarks.push_back(
				runBatchedBenchmark(
					"RealtimeRunnerOverloaded",
					1,
					[
						&runner,
						&totalSimulatedTimeS,
						&totalWallTimeS,
						&totalStepCount,
						&totalDeadlineMissCount,
						&maximumOverrunS
					]()
					{
						const simulation::RealtimeExecutionResult executionResult =
							runner.run(
								[](double simulationTimeS, double dtS)
								{
									benchmarkSink =
										benchmarkSink +
										simulationTimeS * dtS;

									const auto workEnd =
										std::chrono::steady_clock::now() +
										std::chrono::milliseconds(2);

									while (std::chrono::steady_clock::now() < workEnd)
									{
										// Deliberately consume the 2 ms execution budget.
									}
								});

						totalSimulatedTimeS +=
							executionResult.simulatedTimeS;

						totalWallTimeS +=
							executionResult.wallTimeS;

						totalStepCount +=
							executionResult.stepCount;

						totalDeadlineMissCount +=
							executionResult.deadlineMissCount;

						maximumOverrunS =
							std::max(
								maximumOverrunS,
								executionResult.maximumOverrunS);

						benchmarkSink =
							benchmarkSink +
							executionResult.realtimeFactor +
							executionResult.maximumOverrunS;
					}));

			suiteResult.realtimeMetrics.push_back(
				RealtimeBenchmarkResult{
					"RealtimeRunnerOverloaded",
					totalSimulatedTimeS,
					totalWallTimeS,
					totalSimulatedTimeS / totalWallTimeS,
					totalStepCount,
					totalDeadlineMissCount,
					maximumOverrunS
				});
		}

		/*
		* ------------------------------------------------------------
		* Fixed-Rate Runner at 1000 Hz
 		*
 		* 0.1 simulated seconds
 		* simulation dt = 0.0001 s
 		* 1,000 simulation steps
 		* target wall-clock rate = 1000 Hz
 		*
 		* Expected behavior:
 		* - runner executes 1,000 steps
 		* - wall-clock duration is about 1.0 s
 		* - simulated duration is only 0.1 s
		* - realtimeFactor is about 0.1
		* - demonstrates that simulation dt and wall-clock
		*   execution rate are independent
		* ------------------------------------------------------------
		*/
		{
			const simulation::RealtimeExecutionConfig config(
				simulation::ExecutionMode::FixedRate,
				0.0001,
				0.1,
				1000.0);

			const simulation::RealtimeSimulationRunner runner(config);

			double totalSimulatedTimeS = 0.0;
			double totalWallTimeS = 0.0;

			std::size_t totalStepCount = 0;
			std::size_t totalDeadlineMissCount = 0;

			double maximumOverrunS = 0.0;

			suiteResult.benchmarks.push_back(
				runBatchedBenchmark(
					"FixedRateRunner1000Hz",
					1,
					[
						&runner,
						&totalSimulatedTimeS,
						&totalWallTimeS,
						&totalStepCount,
						&totalDeadlineMissCount,
						&maximumOverrunS
					]()
					{
						const simulation::RealtimeExecutionResult executionResult =
							runner.run(
								[](double simulationTimeS, double dtS)
								{
									benchmarkSink =
										benchmarkSink +
										simulationTimeS * dtS;
								});

						totalSimulatedTimeS +=
							executionResult.simulatedTimeS;

						totalWallTimeS +=
							executionResult.wallTimeS;

						totalStepCount +=
							executionResult.stepCount;

						totalDeadlineMissCount +=
							executionResult.deadlineMissCount;

						maximumOverrunS =
							std::max(
								maximumOverrunS,
								executionResult.maximumOverrunS);

						benchmarkSink =
							benchmarkSink +
							executionResult.realtimeFactor +
							executionResult.maximumOverrunS;
					}));

			suiteResult.realtimeMetrics.push_back(
				RealtimeBenchmarkResult{
					"FixedRateRunner1000Hz",
					totalSimulatedTimeS,
					totalWallTimeS,
					totalSimulatedTimeS / totalWallTimeS,
					totalStepCount,
					totalDeadlineMissCount,
					maximumOverrunS
				});
		}

		auto addUnpacedStackBenchmark =
			[
				&suiteResult
			](
				const std::string& name,
				NominalStackLevel stackLevel,
				double durationS)
			{
				double totalSimulatedTimeS = 0.0;
				double totalWallTimeS = 0.0;

				std::size_t totalStepCount = 0;
				std::size_t totalDeadlineMissCount = 0;

				double maximumOverrunS = 0.0;

				suiteResult.benchmarks.push_back(
					runBatchedBenchmark(
						name,
						1,
						[
							&totalSimulatedTimeS,
							&totalWallTimeS,
							&totalStepCount,
							&totalDeadlineMissCount,
							&maximumOverrunS,
							stackLevel,
							durationS
						]()
						{
							const NominalFlightRunResult runResult =
								runNominalFlightStack(
									stackLevel,
									durationS,
									false);

							totalSimulatedTimeS +=
								runResult.executionResult.simulatedTimeS;

							totalWallTimeS +=
								runResult.executionResult.wallTimeS;

							totalStepCount +=
								runResult.executionResult.stepCount;

							totalDeadlineMissCount +=
								runResult.executionResult.deadlineMissCount;

							maximumOverrunS =
								std::max(
									maximumOverrunS,
									runResult.executionResult.maximumOverrunS);

							benchmarkSink =
								benchmarkSink +
								runResult.sinkValue;
						}));

				suiteResult.realtimeMetrics.push_back(
					RealtimeBenchmarkResult{
						name,
						totalSimulatedTimeS,
						totalWallTimeS,
						totalSimulatedTimeS /
							totalWallTimeS,
						totalStepCount,
						totalDeadlineMissCount,
						maximumOverrunS
					}
				);
			};

		/*
		* ------------------------------------------------------------
		* End-to-end nominal flight loop
		*
		* Physics:       1000 Hz
		* IMU:            200 Hz
		* GPS:             10 Hz
		* GNC:            100 Hz
		* Telemetry:      100 Hz
		*
		* 10 simulated seconds per run.
		* ------------------------------------------------------------
		*/
		addUnpacedStackBenchmark(
			"EndToEndNominalFlightLoop",
			NominalStackLevel::Telemetry,
			10.0);

		/*
		 * ------------------------------------------------------------
		 * Progressive-stack RTF analysis
		 *
		 * Identical timestep and duration are used so results are
		 * directly comparable.
		 * ------------------------------------------------------------
		 */
		addUnpacedStackBenchmark(
			"ProgressiveSixDoF",
			NominalStackLevel::SixDoF,
			10.0);

		addUnpacedStackBenchmark(
			"ProgressiveSensors",
			NominalStackLevel::Sensors,
			10.0);

		addUnpacedStackBenchmark(
			"ProgressiveNavigation",
			NominalStackLevel::Navigation,
			10.0);

		addUnpacedStackBenchmark(
			"ProgressiveGnc",
			NominalStackLevel::Gnc,
			10.0);

		addUnpacedStackBenchmark(
			"ProgressiveActuators",
			NominalStackLevel::Actuators,
			10.0);

		/*
		* ------------------------------------------------------------
		* Long-duration stability
		*
		* One continuous 300 simulated second run.
		*
		* First and final 1000 physics steps are compared to detect
		* meaningful timing degradation.
		* ------------------------------------------------------------
		*/
		{
			const NominalFlightRunResult longRun =
				runNominalFlightStack(
					NominalStackLevel::Telemetry,
					300.0,
					true);

			const double wallTimeS =
				longRun.executionResult.wallTimeS;

			suiteResult.benchmarks.push_back(
				BenchmarkResult{
					"LongDurationNominalFlight300s",
					1,
					wallTimeS,
					wallTimeS,
					wallTimeS,
					wallTimeS,
					1.0 / wallTimeS
				});

			suiteResult.realtimeMetrics.push_back(
				RealtimeBenchmarkResult{
					"LongDurationNominalFlight300s",

					longRun.executionResult.simulatedTimeS,
					longRun.executionResult.wallTimeS,
					longRun.executionResult.realtimeFactor,

					longRun.executionResult.stepCount,
					longRun.executionResult.deadlineMissCount,

					longRun.executionResult.maximumOverrunS,

					true,

					longRun.firstWindowMeanStepTimeS,
					longRun.finalWindowMeanStepTimeS,
					longRun.timingDegradationFraction
				});

			benchmarkSink =
				benchmarkSink +
				longRun.sinkValue;
		}

		return suiteResult;
	}
}