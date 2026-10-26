#include <visualization/LiveTelemetryPublisher.h>
#include <visualization/VisualizationSnapshot.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::visualization
{
	namespace
	{
		VisualizationSnapshot makeLiveSnapshot(
			double timeS,
			double value)
		{
			const math::Vector3 vector(
				value,
				value,
				value);

			return VisualizationSnapshot{
				timeS,

				vector,
				vector,
				math::Quaternion::identity(),
				vector,

				vector,
				vector,
				math::Quaternion::identity(),
				vector,

				vector,
				vector,
				vector
			};
		}
	}

	TEST(
		LiveTelemetryPublisherTests,
		StartsWithoutSnapshot)
	{
		const LiveTelemetryPublisher publisher;

		EXPECT_FALSE(
			publisher.hasSnapshot());

		EXPECT_FALSE(
			publisher.getLatestSnapshot().has_value());
	}

	TEST(
		LiveTelemetryPublisherTests,
		PublishMakesSnapshotAvailable)
	{
		LiveTelemetryPublisher publisher;

		publisher.publish(
			makeLiveSnapshot(
				1.0,
				5.0));

		EXPECT_TRUE(
			publisher.hasSnapshot());

		const auto latest =
			publisher.getLatestSnapshot();

		ASSERT_TRUE(
			latest.has_value());

		EXPECT_DOUBLE_EQ(
			latest->simulationTimeS,
			1.0);

		EXPECT_DOUBLE_EQ(
			latest->truthPositionN.x,
			5.0);
	}

	TEST(
		LiveTelemetryPublisherTests,
		LatestPublishReplacesOlderSnapshot)
	{
		LiveTelemetryPublisher publisher;

		publisher.publish(
			makeLiveSnapshot(
				1.0,
				1.0));

		publisher.publish(
			makeLiveSnapshot(
				2.0,
				2.0));

		const auto latest =
			publisher.getLatestSnapshot();

		ASSERT_TRUE(
			latest.has_value());

		EXPECT_DOUBLE_EQ(
			latest->simulationTimeS,
			2.0);

		EXPECT_DOUBLE_EQ(
			latest->truthPositionN.x,
			2.0);
	}

	TEST(
		LiveTelemetryPublisherTests,
		ReadingLatestSnapshotDoesNotConsumeIt)
	{
		LiveTelemetryPublisher publisher;

		publisher.publish(
			makeLiveSnapshot(
				1.0,
				5.0));

		const auto firstRead =
			publisher.getLatestSnapshot();

		const auto secondRead =
			publisher.getLatestSnapshot();

		ASSERT_TRUE(
			firstRead.has_value());

		ASSERT_TRUE(
			secondRead.has_value());

		EXPECT_DOUBLE_EQ(
			firstRead->simulationTimeS,
			secondRead->simulationTimeS);

		EXPECT_TRUE(
			publisher.hasSnapshot());
	}

	TEST(
		LiveTelemetryPublisherTests,
		RejectsNonFiniteSimulationTime)
	{
		LiveTelemetryPublisher publisher;

		auto invalid =
			makeLiveSnapshot(
				0.0,
				0.0);

		invalid.simulationTimeS =
			std::numeric_limits<double>::quiet_NaN();

		EXPECT_THROW(
			publisher.publish(
				invalid),
			std::invalid_argument);

		invalid.simulationTimeS =
			std::numeric_limits<double>::infinity();

		EXPECT_THROW(
			publisher.publish(
				invalid),
			std::invalid_argument);
	}

	TEST(
		LiveTelemetryPublisherTests,
		RejectedPublishDoesNotReplaceExistingSnapshot)
	{
		LiveTelemetryPublisher publisher;

		publisher.publish(
			makeLiveSnapshot(
				1.0,
				1.0));

		auto invalid =
			makeLiveSnapshot(
				2.0,
				2.0);

		invalid.simulationTimeS =
			std::numeric_limits<double>::quiet_NaN();

		EXPECT_THROW(
			publisher.publish(
				invalid),
			std::invalid_argument);

		const auto latest =
			publisher.getLatestSnapshot();

		ASSERT_TRUE(
			latest.has_value());

		EXPECT_DOUBLE_EQ(
			latest->simulationTimeS,
			1.0);
	}
}