#include <gtest/gtest.h>

#include <telemetry/TelemetrySchedule.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::telemetry
{
	TEST(TelemetryScheduleTests, StoresPeriod)
	{
		TelemetrySchedule schedule(0.01);

		EXPECT_DOUBLE_EQ(
			schedule.getPeriodS(),
			0.01
		);
	}


	TEST(TelemetryScheduleTests, RejectsZeroPeriod)
	{
		EXPECT_THROW(
			TelemetrySchedule(0.0),
			std::invalid_argument
		);
	}


	TEST(TelemetryScheduleTests, RejectsNegativePeriod)
	{
		EXPECT_THROW(
			TelemetrySchedule(-0.01),
			std::invalid_argument
		);
	}


	TEST(TelemetryScheduleTests, RejectsNonFinitePeriod)
	{
		double nanValue =
			std::numeric_limits<double>::quiet_NaN();

		EXPECT_THROW(
			(void)TelemetrySchedule(nanValue),
			std::invalid_argument
		);
	}


	TEST(TelemetryScheduleTests, RecordsAtTimeZero)
	{
		TelemetrySchedule schedule(0.1);

		EXPECT_TRUE(
			schedule.shouldRecord(0.0)
		);
	}


	TEST(TelemetryScheduleTests, DoesNotRecordBeforeNextPeriod)
	{
		TelemetrySchedule schedule(0.1);

		EXPECT_TRUE(schedule.shouldRecord(0.0));

		EXPECT_FALSE(
			schedule.shouldRecord(0.05)
		);
	}


	TEST(TelemetryScheduleTests, RecordsAtScheduledPeriod)
	{
		TelemetrySchedule schedule(0.1);

		EXPECT_TRUE(schedule.shouldRecord(0.0));
		EXPECT_FALSE(schedule.shouldRecord(0.05));
		EXPECT_TRUE(schedule.shouldRecord(0.1));
		EXPECT_FALSE(schedule.shouldRecord(0.15));
		EXPECT_TRUE(schedule.shouldRecord(0.2));
	}


	TEST(TelemetryScheduleTests, HandlesTimeJump)
	{
		TelemetrySchedule schedule(0.1);

		EXPECT_TRUE(schedule.shouldRecord(0.0));

		EXPECT_TRUE(
			schedule.shouldRecord(0.35)
		);

		EXPECT_FALSE(
			schedule.shouldRecord(0.36)
		);

		EXPECT_TRUE(
			schedule.shouldRecord(0.4)
		);
	}


	TEST(TelemetryScheduleTests, RejectsNegativeTime)
	{
		TelemetrySchedule schedule(0.1);

		EXPECT_THROW(
			schedule.shouldRecord(-0.1),
			std::invalid_argument
		);
	}


	TEST(TelemetryScheduleTests, RejectsNonFiniteTime)
	{
		TelemetrySchedule schedule(0.1);

		EXPECT_THROW(
			schedule.shouldRecord(
				std::numeric_limits<double>::infinity()
			),
			std::invalid_argument
		);
	}


	TEST(TelemetryScheduleTests, RejectsTimeGoingBackward)
	{
		TelemetrySchedule schedule(0.1);

		schedule.shouldRecord(0.0);
		schedule.shouldRecord(0.2);

		EXPECT_THROW(
			schedule.shouldRecord(0.1),
			std::invalid_argument
		);
	}


	TEST(TelemetryScheduleTests, ResetRestartsSchedule)
	{
		TelemetrySchedule schedule(0.1);

		EXPECT_TRUE(schedule.shouldRecord(0.0));
		EXPECT_TRUE(schedule.shouldRecord(0.1));
		EXPECT_TRUE(schedule.shouldRecord(0.2));

		schedule.reset();

		EXPECT_TRUE(
			schedule.shouldRecord(0.0)
		);
	}
}