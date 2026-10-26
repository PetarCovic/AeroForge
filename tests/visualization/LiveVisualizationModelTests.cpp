#include <visualization/LiveVisualizationModel.h>
#include <visualization/LiveTelemetryPublisher.h>
#include <visualization/VisualizationSnapshot.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>

namespace aeroforge::visualization
{
	namespace
	{
		VisualizationSnapshot makeSnapshot(
			double simulationTimeS,
			double positionValue)
		{
			const math::Vector3 position(
				positionValue,
				positionValue + 1.0,
				positionValue + 2.0);

			const math::Vector3 zero(
				0.0,
				0.0,
				0.0);

			return VisualizationSnapshot{
				simulationTimeS,

				position,
				zero,
				math::Quaternion::identity(),
				zero,

				position,
				zero,
				math::Quaternion::identity(),
				zero,

				zero,
				zero,
				zero
			};
		}

		void expectPositionEquals(
			const math::Vector3& actual,
			const math::Vector3& expected)
		{
			EXPECT_DOUBLE_EQ(
				actual.x,
				expected.x);

			EXPECT_DOUBLE_EQ(
				actual.y,
				expected.y);

			EXPECT_DOUBLE_EQ(
				actual.z,
				expected.z);
		}
	}

	TEST(
		LiveVisualizationModelTests,
		ConstructsWithoutSnapshot)
	{
		const auto publisher =
			std::make_shared<LiveTelemetryPublisher>();

		const LiveVisualizationModel model(
			publisher,
			100);

		EXPECT_FALSE(
			model.hasSnapshot());

		EXPECT_TRUE(
			model.getTrajectoryHistory().empty());

		EXPECT_EQ(
			model.getTrajectoryHistory().size(),
			0u);
	}

	TEST(
		LiveVisualizationModelTests,
		RejectsNullPublisher)
	{
		const std::shared_ptr<const LiveTelemetryPublisher>
			publisher = nullptr;

		EXPECT_THROW(
			LiveVisualizationModel(
				publisher,
				100),
			std::invalid_argument);
	}

	TEST(
		LiveVisualizationModelTests,
		RejectsZeroTrajectoryCapacity)
	{
		const auto publisher =
			std::make_shared<LiveTelemetryPublisher>();

		EXPECT_THROW(
			LiveVisualizationModel(
				publisher,
				0),
			std::invalid_argument);
	}

	TEST(
		LiveVisualizationModelTests,
		UpdateWithoutPublishedSnapshotDoesNothing)
	{
		const auto publisher =
			std::make_shared<LiveTelemetryPublisher>();

		LiveVisualizationModel model(
			publisher,
			100);

		model.update();

		EXPECT_FALSE(
			model.hasSnapshot());

		EXPECT_TRUE(
			model.getTrajectoryHistory().empty());
	}

	TEST(
		LiveVisualizationModelTests,
		HasSnapshotFalseBeforeFirstUpdate)
	{
		const auto publisher =
			std::make_shared<LiveTelemetryPublisher>();

		LiveVisualizationModel model(
			publisher,
			100);

		publisher->publish(
			makeSnapshot(
				1.0,
				10.0));

		EXPECT_FALSE(
			model.hasSnapshot());
	}

	TEST(
		LiveVisualizationModelTests,
		UpdateConsumesPublishedSnapshot)
	{
		const auto publisher =
			std::make_shared<LiveTelemetryPublisher>();

		LiveVisualizationModel model(
			publisher,
			100);

		const VisualizationSnapshot snapshot =
			makeSnapshot(
				1.0,
				10.0);

		publisher->publish(
			snapshot);

		model.update();

		ASSERT_TRUE(
			model.hasSnapshot());

		EXPECT_DOUBLE_EQ(
			model.getCurrentSnapshot().simulationTimeS,
			1.0);

		expectPositionEquals(
			model.getCurrentSnapshot().truthPositionN,
			snapshot.truthPositionN);
	}

	TEST(
		LiveVisualizationModelTests,
		HasSnapshotTrueAfterFirstSnapshot)
	{
		const auto publisher =
			std::make_shared<LiveTelemetryPublisher>();

		LiveVisualizationModel model(
			publisher,
			100);

		publisher->publish(
			makeSnapshot(
				1.0,
				10.0));

		model.update();

		EXPECT_TRUE(
			model.hasSnapshot());
	}

	TEST(
		LiveVisualizationModelTests,
		UpdateAddsTruthPositionToTrajectory)
	{
		const auto publisher =
			std::make_shared<LiveTelemetryPublisher>();

		LiveVisualizationModel model(
			publisher,
			100);

		const VisualizationSnapshot snapshot =
			makeSnapshot(
				2.0,
				20.0);

		publisher->publish(
			snapshot);

		model.update();

		const TrajectoryHistory& history =
			model.getTrajectoryHistory();

		ASSERT_EQ(
			history.size(),
			1u);

		const TrajectoryPoint& point =
			history.getPoints()[0];

		EXPECT_DOUBLE_EQ(
			point.simulationTimeS,
			2.0);

		expectPositionEquals(
			point.positionN,
			snapshot.truthPositionN);
	}

	TEST(
		LiveVisualizationModelTests,
		RepeatedUpdateDoesNotDuplicateSnapshot)
	{
		const auto publisher =
			std::make_shared<LiveTelemetryPublisher>();

		LiveVisualizationModel model(
			publisher,
			100);

		publisher->publish(
			makeSnapshot(
				1.0,
				10.0));

		model.update();

		const double firstTime =
			model.getCurrentSnapshot().simulationTimeS;

		model.update();

		EXPECT_DOUBLE_EQ(
			model.getCurrentSnapshot().simulationTimeS,
			firstTime);
	}

	TEST(
		LiveVisualizationModelTests,
		RepeatedUpdateDoesNotDuplicateTrajectoryPoint)
	{
		const auto publisher =
			std::make_shared<LiveTelemetryPublisher>();

		LiveVisualizationModel model(
			publisher,
			100);

		publisher->publish(
			makeSnapshot(
				1.0,
				10.0));

		model.update();
		model.update();
		model.update();

		EXPECT_EQ(
			model.getTrajectoryHistory().size(),
			1u);
	}

	TEST(
		LiveVisualizationModelTests,
		NewerSnapshotReplacesCurrentSnapshot)
	{
		const auto publisher =
			std::make_shared<LiveTelemetryPublisher>();

		LiveVisualizationModel model(
			publisher,
			100);

		publisher->publish(
			makeSnapshot(
				1.0,
				10.0));

		model.update();

		publisher->publish(
			makeSnapshot(
				2.0,
				20.0));

		model.update();

		ASSERT_TRUE(
			model.hasSnapshot());

		EXPECT_DOUBLE_EQ(
			model.getCurrentSnapshot().simulationTimeS,
			2.0);

		EXPECT_DOUBLE_EQ(
			model.getCurrentSnapshot().truthPositionN.x,
			20.0);
	}

	TEST(
		LiveVisualizationModelTests,
		NewerSnapshotAddsTrajectoryPoint)
	{
		const auto publisher =
			std::make_shared<LiveTelemetryPublisher>();

		LiveVisualizationModel model(
			publisher,
			100);

		publisher->publish(
			makeSnapshot(
				1.0,
				10.0));

		model.update();

		publisher->publish(
			makeSnapshot(
				2.0,
				20.0));

		model.update();

		const TrajectoryHistory& history =
			model.getTrajectoryHistory();

		ASSERT_EQ(
			history.size(),
			2u);

		EXPECT_DOUBLE_EQ(
			history.getPoints()[0].simulationTimeS,
			1.0);

		EXPECT_DOUBLE_EQ(
			history.getPoints()[1].simulationTimeS,
			2.0);
	}

	TEST(
		LiveVisualizationModelTests,
		BackwardTimeStartsNewTrajectory)
	{
		const auto publisher =
			std::make_shared<LiveTelemetryPublisher>();

		LiveVisualizationModel model(
			publisher,
			100);

		publisher->publish(
			makeSnapshot(
				10.0,
				100.0));

		model.update();

		publisher->publish(
			makeSnapshot(
				11.0,
				110.0));

		model.update();

		publisher->publish(
			makeSnapshot(
				0.0,
				5.0));

		model.update();

		ASSERT_TRUE(
			model.hasSnapshot());

		EXPECT_DOUBLE_EQ(
			model.getCurrentSnapshot().simulationTimeS,
			0.0);

		EXPECT_DOUBLE_EQ(
			model.getCurrentSnapshot().truthPositionN.x,
			5.0);
	}

	TEST(
		LiveVisualizationModelTests,
		BackwardTimeClearsPreviousTrajectory)
	{
		const auto publisher =
			std::make_shared<LiveTelemetryPublisher>();

		LiveVisualizationModel model(
			publisher,
			100);

		publisher->publish(
			makeSnapshot(
				10.0,
				100.0));

		model.update();

		publisher->publish(
			makeSnapshot(
				11.0,
				110.0));

		model.update();

		ASSERT_EQ(
			model.getTrajectoryHistory().size(),
			2u);

		publisher->publish(
			makeSnapshot(
				0.0,
				5.0));

		model.update();

		const TrajectoryHistory& history =
			model.getTrajectoryHistory();

		ASSERT_EQ(
			history.size(),
			1u);

		EXPECT_DOUBLE_EQ(
			history.getPoints()[0].simulationTimeS,
			0.0);

		EXPECT_DOUBLE_EQ(
			history.getPoints()[0].positionN.x,
			5.0);
	}

	TEST(
		LiveVisualizationModelTests,
		GetCurrentSnapshotThrowsBeforeSnapshotExists)
	{
		const auto publisher =
			std::make_shared<LiveTelemetryPublisher>();

		const LiveVisualizationModel model(
			publisher,
			100);

		EXPECT_THROW(
			model.getCurrentSnapshot(),
			std::logic_error);
	}

	TEST(
		LiveVisualizationModelTests,
		GetCurrentSnapshotReturnsLatestSnapshot)
	{
		const auto publisher =
			std::make_shared<LiveTelemetryPublisher>();

		LiveVisualizationModel model(
			publisher,
			100);

		publisher->publish(
			makeSnapshot(
				1.0,
				10.0));

		model.update();

		publisher->publish(
			makeSnapshot(
				2.0,
				20.0));

		model.update();

		const VisualizationSnapshot& result =
			model.getCurrentSnapshot();

		EXPECT_DOUBLE_EQ(
			result.simulationTimeS,
			2.0);

		EXPECT_DOUBLE_EQ(
			result.truthPositionN.x,
			20.0);
	}

	TEST(
		LiveVisualizationModelTests,
		GetTrajectoryHistoryInitiallyEmpty)
	{
		const auto publisher =
			std::make_shared<LiveTelemetryPublisher>();

		const LiveVisualizationModel model(
			publisher,
			100);

		EXPECT_TRUE(
			model.getTrajectoryHistory().empty());

		EXPECT_EQ(
			model.getTrajectoryHistory().size(),
			0u);
	}

	TEST(
		LiveVisualizationModelTests,
		TrajectoryHistoryRespectsConfiguredCapacity)
	{
		const auto publisher =
			std::make_shared<LiveTelemetryPublisher>();

		LiveVisualizationModel model(
			publisher,
			3);

		for (int i = 0;
			i < 5;
			++i)
		{
			publisher->publish(
				makeSnapshot(
					static_cast<double>(i),
					static_cast<double>(i) * 10.0));

			model.update();
		}

		const TrajectoryHistory& history =
			model.getTrajectoryHistory();

		ASSERT_EQ(
			history.size(),
			3u);

		EXPECT_EQ(
			history.capacity(),
			3u);

		EXPECT_DOUBLE_EQ(
			history.getPoints()[0].simulationTimeS,
			2.0);

		EXPECT_DOUBLE_EQ(
			history.getPoints()[1].simulationTimeS,
			3.0);

		EXPECT_DOUBLE_EQ(
			history.getPoints()[2].simulationTimeS,
			4.0);
	}

	TEST(
		LiveVisualizationModelTests,
		TrajectoryHistoryStoresNewestPositions)
	{
		const auto publisher =
			std::make_shared<LiveTelemetryPublisher>();

		LiveVisualizationModel model(
			publisher,
			2);

		publisher->publish(
			makeSnapshot(
				1.0,
				10.0));

		model.update();

		publisher->publish(
			makeSnapshot(
				2.0,
				20.0));

		model.update();

		publisher->publish(
			makeSnapshot(
				3.0,
				30.0));

		model.update();

		const auto& points =
			model.getTrajectoryHistory().getPoints();

		ASSERT_EQ(
			points.size(),
			2u);

		EXPECT_DOUBLE_EQ(
			points[0].positionN.x,
			20.0);

		EXPECT_DOUBLE_EQ(
			points[1].positionN.x,
			30.0);
	}
}