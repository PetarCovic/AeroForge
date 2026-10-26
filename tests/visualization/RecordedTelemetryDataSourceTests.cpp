#include <visualization/RecordedTelemetryDataSource.h>
#include <visualization/VisualizationSnapshot.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <vector>

namespace aeroforge::visualization
{
	namespace
	{
		VisualizationSnapshot makeRecordedSnapshot(
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
		RecordedTelemetryDataSourceTests,
		ConstructsFromOrderedSnapshots)
	{
		const RecordedTelemetryDataSource source(
			std::vector<VisualizationSnapshot>{
			makeRecordedSnapshot(
				0.0,
				0.0),

				makeRecordedSnapshot(
					1.0,
					1.0),

				makeRecordedSnapshot(
					2.0,
					2.0)
		});

		EXPECT_EQ(
			source.size(),
			3u);

		EXPECT_FALSE(
			source.empty());
	}

	TEST(
		RecordedTelemetryDataSourceTests,
		RejectsEmptySnapshotCollection)
	{
		EXPECT_THROW(
			RecordedTelemetryDataSource(
				std::vector<VisualizationSnapshot>{}),
			std::invalid_argument);
	}

	TEST(
		RecordedTelemetryDataSourceTests,
		ReportsStartAndEndTimes)
	{
		const RecordedTelemetryDataSource source(
			std::vector<VisualizationSnapshot>{
			makeRecordedSnapshot(
				5.0,
				0.0),

				makeRecordedSnapshot(
					10.0,
					1.0),

				makeRecordedSnapshot(
					20.0,
					2.0)
		});

		EXPECT_DOUBLE_EQ(
			source.getStartTimeS(),
			5.0);

		EXPECT_DOUBLE_EQ(
			source.getEndTimeS(),
			20.0);
	}

	TEST(
		RecordedTelemetryDataSourceTests,
		RejectsNonFiniteSnapshotTimes)
	{
		auto invalid =
			makeRecordedSnapshot(
				0.0,
				0.0);

		invalid.simulationTimeS =
			std::numeric_limits<double>::quiet_NaN();

		EXPECT_THROW(
			RecordedTelemetryDataSource(
				std::vector<VisualizationSnapshot>{
			invalid
		}),
			std::invalid_argument);
	}

	TEST(
		RecordedTelemetryDataSourceTests,
		RejectsDecreasingSnapshotTimes)
	{
		EXPECT_THROW(
			RecordedTelemetryDataSource(
				std::vector<VisualizationSnapshot>{
			makeRecordedSnapshot(
				2.0,
				2.0),

				makeRecordedSnapshot(
					1.0,
					1.0)
		}),
			std::invalid_argument);
	}

	TEST(
		RecordedTelemetryDataSourceTests,
		RejectsDuplicateSnapshotTimes)
	{
		EXPECT_THROW(
			RecordedTelemetryDataSource(
				std::vector<VisualizationSnapshot>{
			makeRecordedSnapshot(
				1.0,
				1.0),

				makeRecordedSnapshot(
					1.0,
					2.0)
		}),
			std::invalid_argument);
	}

	TEST(
		RecordedTelemetryDataSourceTests,
		SampleAtExactTimestampReturnsStoredSnapshot)
	{
		const RecordedTelemetryDataSource source(
			std::vector<VisualizationSnapshot>{
			makeRecordedSnapshot(
				0.0,
				0.0),

				makeRecordedSnapshot(
					5.0,
					5.0),

				makeRecordedSnapshot(
					10.0,
					10.0)
		});

		const VisualizationSnapshot result =
			source.sampleAt(
				5.0);

		EXPECT_DOUBLE_EQ(
			result.simulationTimeS,
			5.0);

		EXPECT_DOUBLE_EQ(
			result.truthPositionN.x,
			5.0);
	}

	TEST(
		RecordedTelemetryDataSourceTests,
		SampleBeforeStartReturnsFirstSnapshot)
	{
		const RecordedTelemetryDataSource source(
			std::vector<VisualizationSnapshot>{
			makeRecordedSnapshot(
				5.0,
				5.0),

				makeRecordedSnapshot(
					10.0,
					10.0)
		});

		const VisualizationSnapshot result =
			source.sampleAt(
				-100.0);

		EXPECT_DOUBLE_EQ(
			result.simulationTimeS,
			5.0);

		EXPECT_DOUBLE_EQ(
			result.truthPositionN.x,
			5.0);
	}

	TEST(
		RecordedTelemetryDataSourceTests,
		SampleAfterEndReturnsLastSnapshot)
	{
		const RecordedTelemetryDataSource source(
			std::vector<VisualizationSnapshot>{
			makeRecordedSnapshot(
				5.0,
				5.0),

				makeRecordedSnapshot(
					10.0,
					10.0)
		});

		const VisualizationSnapshot result =
			source.sampleAt(
				100.0);

		EXPECT_DOUBLE_EQ(
			result.simulationTimeS,
			10.0);

		EXPECT_DOUBLE_EQ(
			result.truthPositionN.x,
			10.0);
	}

	TEST(
		RecordedTelemetryDataSourceTests,
		SampleBetweenSnapshotsInterpolates)
	{
		const RecordedTelemetryDataSource source(
			std::vector<VisualizationSnapshot>{
			makeRecordedSnapshot(
				0.0,
				0.0),

				makeRecordedSnapshot(
					10.0,
					10.0)
		});

		const VisualizationSnapshot result =
			source.sampleAt(
				5.0);

		EXPECT_DOUBLE_EQ(
			result.simulationTimeS,
			5.0);

		EXPECT_NEAR(
			result.truthPositionN.x,
			5.0,
			1e-9);
	}

	TEST(
		RecordedTelemetryDataSourceTests,
		RejectsNonFiniteSampleTime)
	{
		const RecordedTelemetryDataSource source(
			std::vector<VisualizationSnapshot>{
			makeRecordedSnapshot(
				0.0,
				0.0),

				makeRecordedSnapshot(
					10.0,
					10.0)
		});

		EXPECT_THROW(
			source.sampleAt(
				std::numeric_limits<double>::quiet_NaN()),
			std::invalid_argument);

		EXPECT_THROW(
			source.sampleAt(
				std::numeric_limits<double>::infinity()),
			std::invalid_argument);
	}
}