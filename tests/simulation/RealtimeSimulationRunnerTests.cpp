#include <simulation/ExecutionMode.h>
#include <simulation/RealtimeExecutionConfig.h>
#include <simulation/RealtimeExecutionResult.h>
#include <simulation/RealtimeSimulationRunner.h>

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace aeroforge::simulation
{
	TEST(
		RealtimeSimulationRunnerTests,
		RejectsEmptyStepFunction)
	{
		const RealtimeExecutionConfig config(
			ExecutionMode::Unpaced,
			0.01,
			1.0,
			0.0);

		const RealtimeSimulationRunner runner(config);

		const StepFunction emptyStepFunction;

		EXPECT_THROW(
			runner.run(emptyStepFunction),
			std::invalid_argument);
	}

	TEST(
		RealtimeSimulationRunnerTests,
		UnpacedModeExecutesExpectedNumberOfSteps)
	{
		const RealtimeExecutionConfig config(
			ExecutionMode::Unpaced,
			0.01,
			1.0,
			0.0);

		const RealtimeSimulationRunner runner(config);

		std::size_t callbackCount = 0;

		const RealtimeExecutionResult result =
			runner.run(
				[&callbackCount](
					double,
					double)
				{
					++callbackCount;
				});

		EXPECT_EQ(
			callbackCount,
			100u);

		EXPECT_EQ(
			result.stepCount,
			100u);

		EXPECT_NEAR(
			result.simulatedTimeS,
			1.0,
			1e-12);
	}

	TEST(
		RealtimeSimulationRunnerTests,
		StepFunctionReceivesExpectedSimulationTimes)
	{
		const RealtimeExecutionConfig config(
			ExecutionMode::Unpaced,
			0.25,
			1.0,
			0.0);

		const RealtimeSimulationRunner runner(config);

		std::vector<double> simulationTimes;

		runner.run(
			[&simulationTimes](
				double simulationTimeS,
				double)
			{
				simulationTimes.push_back(
					simulationTimeS);
			});

		ASSERT_EQ(
			simulationTimes.size(),
			4u);

		EXPECT_NEAR(
			simulationTimes[0],
			0.0,
			1e-12);

		EXPECT_NEAR(
			simulationTimes[1],
			0.25,
			1e-12);

		EXPECT_NEAR(
			simulationTimes[2],
			0.50,
			1e-12);

		EXPECT_NEAR(
			simulationTimes[3],
			0.75,
			1e-12);
	}

	TEST(
		RealtimeSimulationRunnerTests,
		UsesPartialFinalTimeStepWithoutOvershootingDuration)
	{
		const RealtimeExecutionConfig config(
			ExecutionMode::Unpaced,
			0.3,
			1.0,
			0.0);

		const RealtimeSimulationRunner runner(config);

		std::vector<double> timeSteps;

		const RealtimeExecutionResult result =
			runner.run(
				[&timeSteps](
					double,
					double dtS)
				{
					timeSteps.push_back(dtS);
				});

		ASSERT_EQ(
			timeSteps.size(),
			4u);

		EXPECT_NEAR(
			timeSteps[0],
			0.3,
			1e-12);

		EXPECT_NEAR(
			timeSteps[1],
			0.3,
			1e-12);

		EXPECT_NEAR(
			timeSteps[2],
			0.3,
			1e-12);

		EXPECT_NEAR(
			timeSteps[3],
			0.1,
			1e-12);

		EXPECT_EQ(
			result.stepCount,
			4u);

		EXPECT_NEAR(
			result.simulatedTimeS,
			1.0,
			1e-12);
	}

	TEST(
		RealtimeSimulationRunnerTests,
		UnpacedModeReportsNoDeadlineMisses)
	{
		const RealtimeExecutionConfig config(
			ExecutionMode::Unpaced,
			0.001,
			0.1,
			0.0);

		const RealtimeSimulationRunner runner(config);

		const RealtimeExecutionResult result =
			runner.run(
				[](
					double,
					double)
				{
				});

		EXPECT_EQ(
			result.deadlineMissCount,
			0u);

		EXPECT_DOUBLE_EQ(
			result.maximumOverrunS,
			0.0);
	}

	TEST(
		RealtimeSimulationRunnerTests,
		ResultContainsPositiveWallTimeAndRealtimeFactor)
	{
		const RealtimeExecutionConfig config(
			ExecutionMode::Unpaced,
			0.001,
			0.1,
			0.0);

		const RealtimeSimulationRunner runner(config);

		const RealtimeExecutionResult result =
			runner.run(
				[](
					double,
					double)
				{
				});

		EXPECT_GT(
			result.wallTimeS,
			0.0);

		EXPECT_TRUE(
			std::isfinite(result.wallTimeS));

		EXPECT_GT(
			result.realtimeFactor,
			0.0);

		EXPECT_TRUE(
			std::isfinite(result.realtimeFactor));
	}

	TEST(
		RealtimeSimulationRunnerTests,
		RealtimeModeApproximatelyMatchesSimulatedDuration)
	{
		constexpr double durationS = 0.05;

		const RealtimeExecutionConfig config(
			ExecutionMode::Realtime,
			0.01,
			durationS,
			0.0);

		const RealtimeSimulationRunner runner(config);

		const RealtimeExecutionResult result =
			runner.run(
				[](
					double,
					double)
				{
				});

		EXPECT_NEAR(
			result.simulatedTimeS,
			durationS,
			1e-12);

		EXPECT_GE(
			result.wallTimeS,
			0.045);

		EXPECT_LT(
			result.wallTimeS,
			0.25);
	}

	TEST(
		RealtimeSimulationRunnerTests,
		RealtimeModeProducesRealtimeFactorNearOne)
	{
		constexpr double durationS = 0.05;

		const RealtimeExecutionConfig config(
			ExecutionMode::Realtime,
			0.01,
			durationS,
			0.0);

		const RealtimeSimulationRunner runner(config);

		const RealtimeExecutionResult result =
			runner.run(
				[](
					double,
					double)
				{
				});

		EXPECT_GT(
			result.realtimeFactor,
			0.5);

		EXPECT_LT(
			result.realtimeFactor,
			1.5);
	}

	TEST(
		RealtimeSimulationRunnerTests,
		FixedRateModePacesAccordingToTargetRate)
	{
		constexpr double targetRateHz = 100.0;

		const RealtimeExecutionConfig config(
			ExecutionMode::FixedRate,
			0.01,
			0.05,
			targetRateHz);

		const RealtimeSimulationRunner runner(config);

		const RealtimeExecutionResult result =
			runner.run(
				[](
					double,
					double)
				{
				});

		ASSERT_EQ(
			result.stepCount,
			5u);

		const double expectedWallTimeS =
			static_cast<double>(result.stepCount) /
			targetRateHz;

		EXPECT_GE(
			result.wallTimeS,
			expectedWallTimeS - 0.005);

		EXPECT_LT(
			result.wallTimeS,
			expectedWallTimeS + 0.20);
	}

	TEST(
		RealtimeSimulationRunnerTests,
		FixedRateCanDifferFromSimulationRate)
	{
		const RealtimeExecutionConfig config(
			ExecutionMode::FixedRate,
			0.1,
			0.5,
			100.0);

		const RealtimeSimulationRunner runner(config);

		const RealtimeExecutionResult result =
			runner.run(
				[](
					double,
					double)
				{
				});

		EXPECT_EQ(
			result.stepCount,
			5u);

		EXPECT_NEAR(
			result.simulatedTimeS,
			0.5,
			1e-12);

		EXPECT_GE(
			result.wallTimeS,
			0.045);

		EXPECT_LT(
			result.wallTimeS,
			0.25);

		EXPECT_GT(
			result.realtimeFactor,
			1.0);
	}

	TEST(
		RealtimeSimulationRunnerTests,
		SlowRealtimeStepProducesDeadlineMiss)
	{
		const RealtimeExecutionConfig config(
			ExecutionMode::Realtime,
			0.001,
			0.005,
			0.0);

		const RealtimeSimulationRunner runner(config);

		const RealtimeExecutionResult result =
			runner.run(
				[](
					double,
					double)
				{
					const auto workEnd =
						std::chrono::steady_clock::now() +
						std::chrono::milliseconds(3);

					while (
						std::chrono::steady_clock::now() <
						workEnd)
					{
					}
				});

		EXPECT_GT(
			result.deadlineMissCount,
			0u);

		EXPECT_GT(
			result.maximumOverrunS,
			0.0);
	}

	TEST(
		RealtimeSimulationRunnerTests,
		SlowFixedRateStepProducesDeadlineMiss)
	{
		const RealtimeExecutionConfig config(
			ExecutionMode::FixedRate,
			0.001,
			0.005,
			1000.0);

		const RealtimeSimulationRunner runner(config);

		const RealtimeExecutionResult result =
			runner.run(
				[](
					double,
					double)
				{
					const auto workEnd =
						std::chrono::steady_clock::now() +
						std::chrono::milliseconds(3);

					while (
						std::chrono::steady_clock::now() <
						workEnd)
					{
					}
				});

		EXPECT_GT(
			result.deadlineMissCount,
			0u);

		EXPECT_GT(
			result.maximumOverrunS,
			0.0);
	}

	TEST(
		RealtimeSimulationRunnerTests,
		MaximumOverrunRemainsZeroWhenUnpaced)
	{
		const RealtimeExecutionConfig config(
			ExecutionMode::Unpaced,
			0.001,
			0.005,
			0.0);

		const RealtimeSimulationRunner runner(config);

		const RealtimeExecutionResult result =
			runner.run(
				[](
					double,
					double)
				{
					const auto workEnd =
						std::chrono::steady_clock::now() +
						std::chrono::milliseconds(3);

					while (
						std::chrono::steady_clock::now() <
						workEnd)
					{
					}
				});

		EXPECT_EQ(
			result.deadlineMissCount,
			0u);

		EXPECT_DOUBLE_EQ(
			result.maximumOverrunS,
			0.0);
	}

	TEST(
		RealtimeSimulationRunnerTests,
		StepCountMatchesCeilingOfDurationDividedByTimeStep)
	{
		const RealtimeExecutionConfig config(
			ExecutionMode::Unpaced,
			0.07,
			0.50,
			0.0);

		const RealtimeSimulationRunner runner(config);

		const RealtimeExecutionResult result =
			runner.run(
				[](
					double,
					double)
				{
				});

		EXPECT_EQ(
			result.stepCount,
			8u);

		EXPECT_NEAR(
			result.simulatedTimeS,
			0.50,
			1e-12);
	}
}