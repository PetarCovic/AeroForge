#include <performance/PerformanceTimer.h>

#include <simulation/DynamicsModel.h>
#include <simulation/RK4Integrator.h>
#include <simulation/State.h>
#include <simulation/StateDerivative.h>
#include <simulation/ExecutionMode.h>
#include <simulation/RealtimeExecutionConfig.h>
#include <simulation/RealtimeSimulationRunner.h>

#include <telemetry/CsvTelemetryWriter.h>
#include <telemetry/TelemetryRecorder.h>
#include <telemetry/TelemetrySample.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <gtest/gtest.h>

#include <cstddef>
#include <sstream>
#include <vector>
#include <cmath>

namespace aeroforge::performance
{
	namespace
	{
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

		telemetry::TelemetrySample makeTelemetrySample(
			double timeS)
		{
			return telemetry::TelemetrySample{
				timeS,

				math::Vector3(
					timeS * 100.0,
					timeS * 2.0,
					-100.0 - timeS * 10.0),

				math::Vector3(
					100.0,
					2.0,
					-10.0),

				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0),

				math::Vector3(
					0.01,
					-0.02,
					0.03),

				math::Vector3(
					timeS * 100.0 + 0.1,
					timeS * 2.0 - 0.1,
					-100.0 - timeS * 10.0),

				math::Vector3(
					100.1,
					1.9,
					-10.0),

				math::Quaternion(
					0.999999,
					0.0005,
					0.0005,
					0.0005)
				.normalized(),

				math::Vector3(
					0.011,
					-0.019,
					0.031),

				math::Vector3(
					0.01,
					-0.02,
					0.015),

				math::Vector3(
					1.0,
					-0.5,
					0.25),

				math::Vector3(
					0.95,
					-0.48,
					0.24),

				0.75,
				0.01,
				-0.02,
				false
			};
		}
	}

	TEST(
		PerformanceRegressionTests,
		RK4CompletesLargeWorkloadWithinBroadBudget)
	{
		constexpr std::size_t iterationCount =
			100'000;

		constexpr double dtS =
			0.001;

		HarmonicOscillatorModel model;

		simulation::RK4Integrator integrator;

		simulation::State state{
			1.0,
			0.0
		};

		double timeS = 0.0;

		PerformanceTimer timer;

		timer.start();

		for (std::size_t i = 0;
			i < iterationCount;
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

		const double elapsedS =
			timer.stop();

		EXPECT_LT(
			elapsedS,
			2.0);

		EXPECT_TRUE(
			state.isFinite());
	}

	TEST(
		PerformanceRegressionTests,
		CsvSerializationCompletesWithinBroadBudget)
	{
		constexpr std::size_t sampleCount =
			1'000;

		std::vector<telemetry::TelemetrySample>
			samples;

		samples.reserve(sampleCount);

		for (std::size_t i = 0;
			i < sampleCount;
			++i)
		{
			samples.push_back(
				makeTelemetrySample(
					static_cast<double>(i) *
					0.01));
		}

		std::ostringstream output;

		PerformanceTimer timer;

		timer.start();

		telemetry::CsvTelemetryWriter::write(
			output,
			samples);

		const double elapsedS =
			timer.stop();

		EXPECT_LT(
			elapsedS,
			2.0);

		EXPECT_GT(
			output.tellp(),
			std::streampos(0));
	}

	TEST(
		PerformanceRegressionTests,
		TelemetryRecorderCompletesLargeWorkloadWithinBroadBudget)
	{
		constexpr std::size_t sampleCount =
			100'000;

		telemetry::TelemetryRecorder recorder;

		PerformanceTimer timer;

		timer.start();

		for (std::size_t i = 0;
			i < sampleCount;
			++i)
		{
			recorder.record(
				makeTelemetrySample(
					static_cast<double>(i) *
					0.001));
		}

		const double elapsedS =
			timer.stop();

		EXPECT_LT(
			elapsedS,
			2.0);

		EXPECT_EQ(
			recorder.size(),
			sampleCount);
	}

	TEST(
		PerformanceRegressionTests,
		UnpacedRunnerProducesValidPerformanceMetrics)
	{
		const aeroforge::simulation::RealtimeExecutionConfig config(
			aeroforge::simulation::ExecutionMode::Unpaced,
			0.001,
			10.0,
			0.0);

		const aeroforge::simulation::RealtimeSimulationRunner runner(
			config);

		volatile double sink = 0.0;

		const auto result =
			runner.run(
				[&sink](
					double simulationTimeS,
					double dtS)
				{
					sink =
						sink +
						simulationTimeS *
						dtS;
				});

		EXPECT_EQ(
			result.stepCount,
			10000u);

		EXPECT_EQ(
			result.deadlineMissCount,
			0u);

		EXPECT_DOUBLE_EQ(
			result.maximumOverrunS,
			0.0);

		/*
		 * Intentionally broad regression threshold.
		 *
		 * This is not intended to fail due to normal machine-to-machine
		 * benchmark variation. It only detects a severe execution-speed
		 * regression.
		 */
		EXPECT_GT(
			result.realtimeFactor,
			10.0);

		EXPECT_TRUE(
			std::isfinite(
				sink));
	}
}