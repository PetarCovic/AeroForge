#include <gtest/gtest.h>

#include <telemetry/TelemetryRecorder.h>
#include <telemetry/TelemetrySample.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <stdexcept>

namespace aeroforge::telemetry
{
	namespace
	{
		TelemetrySample makeSample(double timeS)
		{
			return TelemetrySample{
				timeS,

				math::Vector3(1.0, 2.0, 3.0),
				math::Vector3(4.0, 5.0, 6.0),
				math::Quaternion(1.0, 0.0, 0.0, 0.0),
				math::Vector3(0.1, 0.2, 0.3),

				math::Vector3(1.1, 2.1, 3.1),
				math::Vector3(4.1, 5.1, 6.1),
				math::Quaternion(1.0, 0.0, 0.0, 0.0),
				math::Vector3(0.11, 0.21, 0.31),

				math::Vector3(0.01, 0.02, 0.03),
				math::Vector3(1.0, 2.0, 3.0),
				math::Vector3(0.9, 1.9, 2.9),

				0.75,
				0.05,
				-0.03,

				false
			};
		}
	}


	TEST(TelemetryRecorderTests, StartsEmpty)
	{
		TelemetryRecorder recorder;

		EXPECT_TRUE(recorder.empty());
		EXPECT_EQ(recorder.size(), 0u);
		EXPECT_TRUE(recorder.getSamples().empty());
	}


	TEST(TelemetryRecorderTests, RecordsSample)
	{
		TelemetryRecorder recorder;

		TelemetrySample sample =
			makeSample(0.0);

		recorder.record(sample);

		ASSERT_EQ(recorder.size(), 1u);
		EXPECT_FALSE(recorder.empty());

		const TelemetrySample& stored =
			recorder.getSamples().front();

		EXPECT_DOUBLE_EQ(stored.timeS, 0.0);
		EXPECT_DOUBLE_EQ(stored.positionN.x, 1.0);
		EXPECT_DOUBLE_EQ(stored.velocityN.y, 5.0);
		EXPECT_DOUBLE_EQ(stored.throttle, 0.75);
	}


	TEST(TelemetryRecorderTests, PreservesChronologicalOrder)
	{
		TelemetryRecorder recorder;

		recorder.record(makeSample(0.0));
		recorder.record(makeSample(0.1));
		recorder.record(makeSample(0.2));

		const auto& samples =
			recorder.getSamples();

		ASSERT_EQ(samples.size(), 3u);

		EXPECT_DOUBLE_EQ(samples[0].timeS, 0.0);
		EXPECT_DOUBLE_EQ(samples[1].timeS, 0.1);
		EXPECT_DOUBLE_EQ(samples[2].timeS, 0.2);
	}


	TEST(TelemetryRecorderTests, RejectsNegativeTimestamp)
	{
		TelemetryRecorder recorder;

		EXPECT_THROW(
			recorder.record(
				makeSample(-0.1)
			),
			std::invalid_argument
		);

		EXPECT_TRUE(recorder.empty());
	}


	TEST(TelemetryRecorderTests, RejectsDuplicateTimestamp)
	{
		TelemetryRecorder recorder;

		recorder.record(
			makeSample(1.0)
		);

		EXPECT_THROW(
			recorder.record(
				makeSample(1.0)
			),
			std::invalid_argument
		);

		EXPECT_EQ(recorder.size(), 1u);
	}


	TEST(TelemetryRecorderTests, RejectsTimestampGoingBackward)
	{
		TelemetryRecorder recorder;

		recorder.record(
			makeSample(2.0)
		);

		EXPECT_THROW(
			recorder.record(
				makeSample(1.5)
			),
			std::invalid_argument
		);

		EXPECT_EQ(recorder.size(), 1u);
	}


	TEST(TelemetryRecorderTests, ClearRemovesAllSamples)
	{
		TelemetryRecorder recorder;

		recorder.record(makeSample(0.0));
		recorder.record(makeSample(0.1));

		recorder.clear();

		EXPECT_TRUE(recorder.empty());
		EXPECT_EQ(recorder.size(), 0u);
	}


	TEST(TelemetryRecorderTests, CanRestartAtZeroAfterClear)
	{
		TelemetryRecorder recorder;

		recorder.record(makeSample(5.0));

		recorder.clear();

		EXPECT_NO_THROW(
			recorder.record(
				makeSample(0.0)
			)
		);

		ASSERT_EQ(recorder.size(), 1u);

		EXPECT_DOUBLE_EQ(
			recorder.getSamples()[0].timeS,
			0.0
		);
	}
}