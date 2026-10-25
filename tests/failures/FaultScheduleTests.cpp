#include <gtest/gtest.h>

#include <failures/FaultSchedule.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::failures
{
	TEST(FaultScheduleTests, StoresValidStartAndEndTimes)
	{
		FaultSchedule schedule(2.0, 5.0);

		EXPECT_DOUBLE_EQ(schedule.getStartTimeS(), 2.0);
		EXPECT_DOUBLE_EQ(schedule.getEndTimeS(), 5.0);
	}

	TEST(FaultScheduleTests, IsInactiveBeforeStartTime)
	{
		FaultSchedule schedule(2.0, 5.0);

		EXPECT_FALSE(schedule.isActive(0.0));
		EXPECT_FALSE(schedule.isActive(1.999));
	}

	TEST(FaultScheduleTests, IsActiveExactlyAtStartTime)
	{
		FaultSchedule schedule(2.0, 5.0);

		EXPECT_TRUE(schedule.isActive(2.0));
	}

	TEST(FaultScheduleTests, IsActiveInsideFaultWindow)
	{
		FaultSchedule schedule(2.0, 5.0);

		EXPECT_TRUE(schedule.isActive(2.5));
		EXPECT_TRUE(schedule.isActive(4.999));
	}

	TEST(FaultScheduleTests, IsInactiveExactlyAtEndTime)
	{
		FaultSchedule schedule(2.0, 5.0);

		EXPECT_FALSE(schedule.isActive(5.0));
	}

	TEST(FaultScheduleTests, IsInactiveAfterEndTime)
	{
		FaultSchedule schedule(2.0, 5.0);

		EXPECT_FALSE(schedule.isActive(5.001));
		EXPECT_FALSE(schedule.isActive(10.0));
	}

	TEST(FaultScheduleTests, AllowsZeroStartTime)
	{
		FaultSchedule schedule(0.0, 1.0);

		EXPECT_TRUE(schedule.isActive(0.0));
		EXPECT_TRUE(schedule.isActive(0.5));
		EXPECT_FALSE(schedule.isActive(1.0));
	}

	TEST(FaultScheduleTests, RejectsNegativeStartTime)
	{
		EXPECT_THROW(
			FaultSchedule(-1.0, 5.0),
			std::invalid_argument
		);
	}

	TEST(FaultScheduleTests, RejectsEndEqualToStart)
	{
		EXPECT_THROW(
			FaultSchedule(2.0, 2.0),
			std::invalid_argument
		);
	}

	TEST(FaultScheduleTests, RejectsEndBeforeStart)
	{
		EXPECT_THROW(
			FaultSchedule(5.0, 2.0),
			std::invalid_argument
		);
	}

	TEST(FaultScheduleTests, RejectsNonFiniteStart)
	{
		EXPECT_THROW(
			FaultSchedule(
				std::numeric_limits<double>::quiet_NaN(),
				5.0
			),
			std::invalid_argument
		);

		EXPECT_THROW(
			FaultSchedule(
				std::numeric_limits<double>::infinity(),
				5.0
			),
			std::invalid_argument
		);
	}

	TEST(FaultScheduleTests, RejectsNonFiniteEnd)
	{
		EXPECT_THROW(
			FaultSchedule(
				2.0,
				std::numeric_limits<double>::quiet_NaN()
			),
			std::invalid_argument
		);

		EXPECT_THROW(
			FaultSchedule(
				2.0,
				std::numeric_limits<double>::infinity()
			),
			std::invalid_argument
		);
	}

	TEST(FaultScheduleTests, RejectsNegativeQueryTime)
	{
		FaultSchedule schedule(2.0, 5.0);

		EXPECT_THROW(
			schedule.isActive(-0.1),
			std::invalid_argument
		);
	}

	TEST(FaultScheduleTests, RejectsNonFiniteQueryTime)
	{
		FaultSchedule schedule(2.0, 5.0);

		EXPECT_THROW(
			schedule.isActive(
				std::numeric_limits<double>::quiet_NaN()
			),
			std::invalid_argument
		);

		EXPECT_THROW(
			schedule.isActive(
				std::numeric_limits<double>::infinity()
			),
			std::invalid_argument
		);
	}
}