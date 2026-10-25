#include <gtest/gtest.h>

#include <simulation/SimulationClock.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace aeroforge::simulation::tests
{
	TEST(
		SimulationClockTests,
		DefaultConstructionStartsAtStepZero)
	{
		SimulationClock clock;

		EXPECT_EQ(
			clock.getCurrentStep(),
			0U);
	}

	TEST(
		SimulationClockTests,
		DefaultConstructionUsesDefaultTimeStep)
	{
		SimulationClock clock;

		EXPECT_DOUBLE_EQ(
			clock.getTimeStep(),
			0.01);
	}

	TEST(
		SimulationClockTests,
		DefaultConstructionStartsAtTimeZero)
	{
		SimulationClock clock;

		EXPECT_DOUBLE_EQ(
			clock.getCurrentTime(),
			0.0);
	}

	TEST(
		SimulationClockTests,
		CustomStartStepIsStored)
	{
		SimulationClock clock(
			25,
			0.01);

		EXPECT_EQ(
			clock.getCurrentStep(),
			25U);
	}

	TEST(
		SimulationClockTests,
		CustomTimeStepIsStored)
	{
		SimulationClock clock(
			0,
			0.125);

		EXPECT_DOUBLE_EQ(
			clock.getTimeStep(),
			0.125);
	}

	TEST(
		SimulationClockTests,
		CurrentTimeIsDerivedFromStepAndTimeStep)
	{
		SimulationClock clock(
			10,
			0.25);

		EXPECT_DOUBLE_EQ(
			clock.getCurrentTime(),
			2.5);
	}

	TEST(
		SimulationClockTests,
		AdvanceIncrementsStepByOne)
	{
		SimulationClock clock;

		clock.advance();

		EXPECT_EQ(
			clock.getCurrentStep(),
			1U);
	}

	TEST(
		SimulationClockTests,
		AdvanceUpdatesCurrentTime)
	{
		SimulationClock clock(
			0,
			0.1);

		clock.advance();

		EXPECT_DOUBLE_EQ(
			clock.getCurrentTime(),
			0.1);
	}

	TEST(
		SimulationClockTests,
		MultipleAdvancesUpdateStepCorrectly)
	{
		SimulationClock clock;

		for (int i = 0; i < 100; ++i)
		{
			clock.advance();
		}

		EXPECT_EQ(
			clock.getCurrentStep(),
			100U);
	}

	TEST(
		SimulationClockTests,
		MultipleAdvancesProduceExpectedTime)
	{
		SimulationClock clock(
			0,
			0.01);

		for (int i = 0; i < 1000; ++i)
		{
			clock.advance();
		}

		EXPECT_DOUBLE_EQ(
			clock.getCurrentTime(),
			10.0);
	}

	TEST(
		SimulationClockTests,
		ResetReturnsClockToStepZero)
	{
		SimulationClock clock;

		for (int i = 0; i < 100; ++i)
		{
			clock.advance();
		}

		clock.reset();

		EXPECT_EQ(
			clock.getCurrentStep(),
			0U);

		EXPECT_DOUBLE_EQ(
			clock.getCurrentTime(),
			0.0);
	}

	TEST(
		SimulationClockTests,
		ResetCanSetArbitraryStep)
	{
		SimulationClock clock(
			0,
			0.1);

		clock.reset(50);

		EXPECT_EQ(
			clock.getCurrentStep(),
			50U);

		EXPECT_DOUBLE_EQ(
			clock.getCurrentTime(),
			5.0);
	}

	TEST(
		SimulationClockTests,
		ResetDoesNotChangeTimeStep)
	{
		SimulationClock clock(
			0,
			0.025);

		clock.advance();
		clock.advance();

		clock.reset();

		EXPECT_DOUBLE_EQ(
			clock.getTimeStep(),
			0.025);
	}

	TEST(
		SimulationClockTests,
		AdvanceDoesNotChangeTimeStep)
	{
		SimulationClock clock(
			0,
			0.025);

		for (int i = 0; i < 100; ++i)
		{
			clock.advance();
		}

		EXPECT_DOUBLE_EQ(
			clock.getTimeStep(),
			0.025);
	}

	TEST(
		SimulationClockTests,
		RejectsZeroTimeStep)
	{
		EXPECT_THROW(
			SimulationClock(
				0,
				0.0),
			std::invalid_argument);
	}

	TEST(
		SimulationClockTests,
		RejectsNegativeTimeStep)
	{
		EXPECT_THROW(
			SimulationClock(
				0,
				-0.01),
			std::invalid_argument);
	}

	TEST(
		SimulationClockTests,
		RejectsNaNTimeStep)
	{
		EXPECT_THROW(
			SimulationClock(
				0,
				std::numeric_limits<double>::quiet_NaN()),
			std::invalid_argument);
	}

	TEST(
		SimulationClockTests,
		RejectsPositiveInfinityTimeStep)
	{
		EXPECT_THROW(
			SimulationClock(
				0,
				std::numeric_limits<double>::infinity()),
			std::invalid_argument);
	}

	TEST(
		SimulationClockTests,
		RejectsNegativeInfinityTimeStep)
	{
		EXPECT_THROW(
			SimulationClock(
				0,
				-std::numeric_limits<double>::infinity()),
			std::invalid_argument);
	}

	TEST(
		SimulationClockTests,
		TimeCalculationDoesNotAccumulateIncrementalDrift)
	{
		constexpr double DT =
			0.001;

		constexpr std::uint64_t STEPS =
			1000000;

		SimulationClock clock(
			0,
			DT);

		for (std::uint64_t i = 0;
			i < STEPS;
			++i)
		{
			clock.advance();
		}

		const double expectedTime =
			static_cast<double>(STEPS) * DT;

		EXPECT_DOUBLE_EQ(
			clock.getCurrentTime(),
			expectedTime);
	}

	TEST(
		SimulationClockTests,
		ResetThenAdvanceContinuesFromNewStep)
	{
		SimulationClock clock(
			0,
			0.5);

		clock.reset(10);
		clock.advance();

		EXPECT_EQ(
			clock.getCurrentStep(),
			11U);

		EXPECT_DOUBLE_EQ(
			clock.getCurrentTime(),
			5.5);
	}

	TEST(
		SimulationClockTests,
		TwoClocksWithSameConfigurationRemainDeterministic)
	{
		SimulationClock clockA(
			0,
			0.01);

		SimulationClock clockB(
			0,
			0.01);

		for (int i = 0; i < 10000; ++i)
		{
			clockA.advance();
			clockB.advance();
		}

		EXPECT_EQ(
			clockA.getCurrentStep(),
			clockB.getCurrentStep());

		EXPECT_DOUBLE_EQ(
			clockA.getCurrentTime(),
			clockB.getCurrentTime());
	}
}