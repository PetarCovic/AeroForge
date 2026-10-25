#include <simulation/SimulationPacer.h>

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <cstddef>
#include <thread>
#include <stdexcept>

namespace aeroforge::simulation
{
	TEST(SimulationPacerTests, StartAllowsElapsedTimeMeasurement)
	{
		SimulationPacer pacer;

		pacer.start();

		const double elapsedS =
			pacer.getElapsedWallTimeS();

		EXPECT_GE(elapsedS, 0.0);
	}

	TEST(SimulationPacerTests, ElapsedTimeIncreasesAfterSleeping)
	{
		SimulationPacer pacer;

		pacer.start();

		std::this_thread::sleep_for(
			std::chrono::milliseconds(10));

		const double elapsedS =
			pacer.getElapsedWallTimeS();

		EXPECT_GE(elapsedS, 0.005);
	}

	TEST(SimulationPacerTests, GetElapsedWallTimeThrowsBeforeStart)
	{
		SimulationPacer pacer;

		EXPECT_THROW(
			pacer.getElapsedWallTimeS(),
			std::logic_error);
	}

	TEST(SimulationPacerTests, PaceToThrowsBeforeStart)
	{
		SimulationPacer pacer;

		EXPECT_THROW(
			pacer.paceTo(0.01),
			std::logic_error);
	}

	TEST(SimulationPacerTests, PaceToRejectsNegativeSimulatedElapsedTime)
	{
		SimulationPacer pacer;

		pacer.start();

		EXPECT_THROW(
			pacer.paceTo(-0.001),
			std::invalid_argument);
	}

	TEST(SimulationPacerTests, PaceToRejectsPositiveInfinity)
	{
		SimulationPacer pacer;

		pacer.start();

		EXPECT_THROW(
			pacer.paceTo(
				std::numeric_limits<double>::infinity()),
			std::invalid_argument);
	}

	TEST(SimulationPacerTests, PaceToRejectsNegativeInfinity)
	{
		SimulationPacer pacer;

		pacer.start();

		EXPECT_THROW(
			pacer.paceTo(
				-std::numeric_limits<double>::infinity()),
			std::invalid_argument);
	}

	TEST(SimulationPacerTests, PaceToRejectsNaN)
	{
		SimulationPacer pacer;

		pacer.start();

		EXPECT_THROW(
			pacer.paceTo(
				std::numeric_limits<double>::quiet_NaN()),
			std::invalid_argument);
	}

	TEST(SimulationPacerTests, PaceToZeroDoesNotSleepForMeaningfulTime)
	{
		SimulationPacer pacer;

		pacer.start();

		const auto wallStart =
			std::chrono::steady_clock::now();

		const double overrunS =
			pacer.paceTo(0.0);

		const auto wallEnd =
			std::chrono::steady_clock::now();

		const double elapsedS =
			std::chrono::duration<double>(
				wallEnd - wallStart).count();

		EXPECT_GE(overrunS, 0.0);

		EXPECT_LT(elapsedS, 0.1);
	}

	TEST(SimulationPacerTests, PaceToFutureDeadlineWaitsUntilApproximatelyTargetTime)
	{
		SimulationPacer pacer;

		pacer.start();

		constexpr double targetS =
			0.02;

		const double overrunS =
			pacer.paceTo(targetS);

		const double elapsedS =
			pacer.getElapsedWallTimeS();

		EXPECT_GE(
			elapsedS,
			targetS);

		EXPECT_GE(
			overrunS,
			0.0);
	}

	TEST(SimulationPacerTests, PaceToPastDeadlineReportsPositiveOverrun)
	{
		SimulationPacer pacer;

		pacer.start();

		std::this_thread::sleep_for(
			std::chrono::milliseconds(20));

		const double overrunS =
			pacer.paceTo(0.005);

		EXPECT_GT(
			overrunS,
			0.0);
	}

	TEST(SimulationPacerTests, OverrunApproximatelyMatchesLateExecution)
	{
		SimulationPacer pacer;

		pacer.start();

		std::this_thread::sleep_for(
			std::chrono::milliseconds(20));

		const double overrunS =
			pacer.paceTo(0.005);

		EXPECT_GE(
			overrunS,
			0.010);

		EXPECT_LT(
			overrunS,
			0.100);
	}

	TEST(SimulationPacerTests, SequentialAbsoluteDeadlinesDoNotAccumulateSleepDuration)
	{
		SimulationPacer pacer;

		pacer.start();

		pacer.paceTo(0.01);
		pacer.paceTo(0.02);
		pacer.paceTo(0.03);

		const double elapsedS =
			pacer.getElapsedWallTimeS();

		EXPECT_GE(
			elapsedS,
			0.03);

		EXPECT_LT(
			elapsedS,
			0.15);
	}

	TEST(SimulationPacerTests, RestartResetsElapsedWallTime)
	{
		SimulationPacer pacer;

		pacer.start();

		std::this_thread::sleep_for(
			std::chrono::milliseconds(20));

		EXPECT_GE(
			pacer.getElapsedWallTimeS(),
			0.01);

		pacer.start();

		const double elapsedAfterRestartS =
			pacer.getElapsedWallTimeS();

		EXPECT_GE(
			elapsedAfterRestartS,
			0.0);

		EXPECT_LT(
			elapsedAfterRestartS,
			0.05);
	}

	TEST(
		SimulationPacerTests,
		SequentialOneKilohertzTargetsRemainMonotonic)
	{
		aeroforge::simulation::SimulationPacer pacer;

		pacer.start();

		for (std::size_t step = 1;
			step <= 20;
			++step)
		{
			const double targetS =
				static_cast<double>(step) *
				0.001;

			const double overrunS =
				pacer.paceTo(targetS);

			EXPECT_GE(
				overrunS,
				0.0);
		}

		const double elapsedS =
			pacer.getElapsedWallTimeS();

		EXPECT_GE(
			elapsedS,
			0.019);

		EXPECT_LT(
			elapsedS,
			0.100);
	}
}