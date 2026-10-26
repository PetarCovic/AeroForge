#include <visualization/TrajectoryHistory.h>

#include <math/Vector3.h>

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::visualization
{
	TEST(
		TrajectoryHistoryTests,
		ConstructsEmptyWithRequestedCapacity)
	{
		const TrajectoryHistory history(
			100);

		EXPECT_TRUE(
			history.empty());

		EXPECT_EQ(
			history.size(),
			0u);

		EXPECT_EQ(
			history.capacity(),
			100u);
	}

	TEST(
		TrajectoryHistoryTests,
		RejectsZeroCapacity)
	{
		EXPECT_THROW(
			TrajectoryHistory(
				0),
			std::invalid_argument);
	}

	TEST(
		TrajectoryHistoryTests,
		AddStoresPoint)
	{
		TrajectoryHistory history(
			10);

		history.add(
			1.0,
			math::Vector3(
				10.0,
				20.0,
				30.0));

		ASSERT_EQ(
			history.size(),
			1u);

		const auto& points =
			history.getPoints();

		EXPECT_DOUBLE_EQ(
			points[0].simulationTimeS,
			1.0);

		EXPECT_DOUBLE_EQ(
			points[0].positionN.x,
			10.0);

		EXPECT_DOUBLE_EQ(
			points[0].positionN.y,
			20.0);

		EXPECT_DOUBLE_EQ(
			points[0].positionN.z,
			30.0);
	}

	TEST(
		TrajectoryHistoryTests,
		PreservesInsertionOrder)
	{
		TrajectoryHistory history(
			10);

		history.add(
			1.0,
			math::Vector3(
				1.0,
				0.0,
				0.0));

		history.add(
			2.0,
			math::Vector3(
				2.0,
				0.0,
				0.0));

		history.add(
			3.0,
			math::Vector3(
				3.0,
				0.0,
				0.0));

		const auto& points =
			history.getPoints();

		ASSERT_EQ(
			points.size(),
			3u);

		EXPECT_DOUBLE_EQ(
			points[0].simulationTimeS,
			1.0);

		EXPECT_DOUBLE_EQ(
			points[1].simulationTimeS,
			2.0);

		EXPECT_DOUBLE_EQ(
			points[2].simulationTimeS,
			3.0);
	}

	TEST(
		TrajectoryHistoryTests,
		DropsOldestPointWhenCapacityExceeded)
	{
		TrajectoryHistory history(
			3);

		history.add(
			1.0,
			math::Vector3(
				1.0,
				0.0,
				0.0));

		history.add(
			2.0,
			math::Vector3(
				2.0,
				0.0,
				0.0));

		history.add(
			3.0,
			math::Vector3(
				3.0,
				0.0,
				0.0));

		history.add(
			4.0,
			math::Vector3(
				4.0,
				0.0,
				0.0));

		const auto& points =
			history.getPoints();

		ASSERT_EQ(
			points.size(),
			3u);

		EXPECT_DOUBLE_EQ(
			points[0].simulationTimeS,
			2.0);

		EXPECT_DOUBLE_EQ(
			points[1].simulationTimeS,
			3.0);

		EXPECT_DOUBLE_EQ(
			points[2].simulationTimeS,
			4.0);
	}

	TEST(
		TrajectoryHistoryTests,
		NeverExceedsConfiguredCapacity)
	{
		TrajectoryHistory history(
			5);

		for (int i = 0;
			i < 100;
			++i)
		{
			history.add(
				static_cast<double>(i),
				math::Vector3(
					static_cast<double>(i),
					0.0,
					0.0));
		}

		EXPECT_EQ(
			history.size(),
			5u);
	}

	TEST(
		TrajectoryHistoryTests,
		ClearRemovesAllPoints)
	{
		TrajectoryHistory history(
			10);

		history.add(
			1.0,
			math::Vector3(
				1.0,
				2.0,
				3.0));

		history.add(
			2.0,
			math::Vector3(
				4.0,
				5.0,
				6.0));

		history.clear();

		EXPECT_TRUE(
			history.empty());

		EXPECT_EQ(
			history.size(),
			0u);

		EXPECT_EQ(
			history.capacity(),
			10u);
	}

	TEST(
		TrajectoryHistoryTests,
		RejectsNonFiniteSimulationTime)
	{
		TrajectoryHistory history(
			10);

		EXPECT_THROW(
			history.add(
				std::numeric_limits<double>::quiet_NaN(),
				math::Vector3(
					0.0,
					0.0,
					0.0)),
			std::invalid_argument);

		EXPECT_THROW(
			history.add(
				std::numeric_limits<double>::infinity(),
				math::Vector3(
					0.0,
					0.0,
					0.0)),
			std::invalid_argument);
	}

	TEST(
		TrajectoryHistoryTests,
		RejectsNonFinitePosition)
	{
		TrajectoryHistory history(
			10);

		EXPECT_THROW(
			history.add(
				1.0,
				math::Vector3(
					std::numeric_limits<double>::infinity(),
					0.0,
					0.0)),
			std::invalid_argument);

		EXPECT_THROW(
			history.add(
				1.0,
				math::Vector3(
					0.0,
					std::numeric_limits<double>::quiet_NaN(),
					0.0)),
			std::invalid_argument);
	}

	TEST(
		TrajectoryHistoryTests,
		ClearAllowsHistoryToBeReused)
	{
		TrajectoryHistory history(
			2);

		history.add(
			1.0,
			math::Vector3(
				1.0,
				0.0,
				0.0));

		history.clear();

		history.add(
			10.0,
			math::Vector3(
				10.0,
				0.0,
				0.0));

		ASSERT_EQ(
			history.size(),
			1u);

		EXPECT_DOUBLE_EQ(
			history.getPoints()[0].simulationTimeS,
			10.0);
	}
}