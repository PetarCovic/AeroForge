#include <gtest/gtest.h>

#include <sensors/SensorSchedule.h>

#include <stdexcept>

namespace aeroforge::sensors::tests
{
	TEST(
		SensorScheduleTests,
		ComputesSamplePeriod)
	{
		SensorSchedule schedule(
			100.0,
			0.0);

		EXPECT_NEAR(
			schedule.getSamplePeriodS(),
			0.01,
			1e-12);
	}

	TEST(
		SensorScheduleTests,
		SamplesAtConfiguredFrequency)
	{
		SensorSchedule schedule(
			100.0,
			0.0);

		EXPECT_TRUE(
			schedule.shouldSample(0.0));

		EXPECT_FALSE(
			schedule.shouldSample(0.005));

		EXPECT_TRUE(
			schedule.shouldSample(0.01));

		EXPECT_FALSE(
			schedule.shouldSample(0.015));

		EXPECT_TRUE(
			schedule.shouldSample(0.02));
	}

	TEST(
		SensorScheduleTests,
		DoesNotSampleTwiceAtSameScheduledInstant)
	{
		SensorSchedule schedule(
			10.0,
			0.0);

		EXPECT_TRUE(
			schedule.shouldSample(0.0));

		EXPECT_FALSE(
			schedule.shouldSample(0.0));
	}

	TEST(
		SensorScheduleTests,
		HandlesSkippedSimulationTimes)
	{
		SensorSchedule schedule(
			10.0,
			0.0);

		EXPECT_TRUE(
			schedule.shouldSample(0.0));

		EXPECT_TRUE(
			schedule.shouldSample(0.35));

		EXPECT_FALSE(
			schedule.shouldSample(0.36));

		EXPECT_TRUE(
			schedule.shouldSample(0.4));
	}

	TEST(
		SensorScheduleTests,
		RejectsInvalidRate)
	{
		EXPECT_THROW(
			SensorSchedule(
				0.0,
				0.0),
			std::invalid_argument);

		EXPECT_THROW(
			SensorSchedule(
				-10.0,
				0.0),
			std::invalid_argument);
	}
}