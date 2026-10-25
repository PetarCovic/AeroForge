#include <gtest/gtest.h>

#include <telemetry/TelemetryRecorder.h>
#include <telemetry/TelemetrySample.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::telemetry
{
	namespace
	{
		TelemetrySample validSample()
		{
			return TelemetrySample{
				0.0,

				math::Vector3(),
				math::Vector3(),
				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0
				),
				math::Vector3(),

				math::Vector3(),
				math::Vector3(),
				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0
				),
				math::Vector3(),

				math::Vector3(),
				math::Vector3(),
				math::Vector3(),

				0.0,
				0.0,
				0.0,

				false
			};
		}
	}


	TEST(
		TelemetryIntegrityTests,
		RejectsNonFiniteTime
	)
	{
		TelemetrySample sample =
			validSample();

		sample.timeS =
			std::numeric_limits<double>::quiet_NaN();

		TelemetryRecorder recorder;

		EXPECT_THROW(
			recorder.record(sample),
			std::invalid_argument
		);
	}


	TEST(
		TelemetryIntegrityTests,
		RejectsNonFiniteTruthPosition
	)
	{
		TelemetrySample sample =
			validSample();

		sample.positionN.x =
			std::numeric_limits<double>::infinity();

		TelemetryRecorder recorder;

		EXPECT_THROW(
			recorder.record(sample),
			std::invalid_argument
		);
	}


	TEST(
		TelemetryIntegrityTests,
		RejectsNonFiniteTruthVelocity
	)
	{
		TelemetrySample sample =
			validSample();

		sample.velocityN.z =
			std::numeric_limits<double>::quiet_NaN();

		TelemetryRecorder recorder;

		EXPECT_THROW(
			recorder.record(sample),
			std::invalid_argument
		);
	}


	TEST(
		TelemetryIntegrityTests,
		RejectsNonFiniteTruthAttitude
	)
	{
		TelemetrySample sample =
			validSample();

		sample.attitudeNB.w =
			std::numeric_limits<double>::quiet_NaN();

		TelemetryRecorder recorder;

		EXPECT_THROW(
			recorder.record(sample),
			std::invalid_argument
		);
	}


	TEST(
		TelemetryIntegrityTests,
		RejectsNonFiniteTruthAngularVelocity
	)
	{
		TelemetrySample sample =
			validSample();

		sample.angularVelocityB.y =
			std::numeric_limits<double>::infinity();

		TelemetryRecorder recorder;

		EXPECT_THROW(
			recorder.record(sample),
			std::invalid_argument
		);
	}


	TEST(
		TelemetryIntegrityTests,
		RejectsNonFiniteEstimatedPosition
	)
	{
		TelemetrySample sample =
			validSample();

		sample.estimatedPositionN.x =
			std::numeric_limits<double>::infinity();

		TelemetryRecorder recorder;

		EXPECT_THROW(
			recorder.record(sample),
			std::invalid_argument
		);
	}


	TEST(
		TelemetryIntegrityTests,
		RejectsNonFiniteEstimatedVelocity
	)
	{
		TelemetrySample sample =
			validSample();

		sample.estimatedVelocityN.y =
			std::numeric_limits<double>::infinity();

		TelemetryRecorder recorder;

		EXPECT_THROW(
			recorder.record(sample),
			std::invalid_argument
		);
	}


	TEST(
		TelemetryIntegrityTests,
		RejectsNonFiniteEstimatedAttitude
	)
	{
		TelemetrySample sample =
			validSample();

		sample.estimatedAttitudeNB.z =
			std::numeric_limits<double>::quiet_NaN();

		TelemetryRecorder recorder;

		EXPECT_THROW(
			recorder.record(sample),
			std::invalid_argument
		);
	}


	TEST(
		TelemetryIntegrityTests,
		RejectsNonFiniteEstimatedAngularVelocity
	)
	{
		TelemetrySample sample =
			validSample();

		sample.estimatedAngularVelocityB.z =
			std::numeric_limits<double>::infinity();

		TelemetryRecorder recorder;

		EXPECT_THROW(
			recorder.record(sample),
			std::invalid_argument
		);
	}


	TEST(
		TelemetryIntegrityTests,
		RejectsNonFiniteAttitudeError
	)
	{
		TelemetrySample sample =
			validSample();

		sample.attitudeErrorB.x =
			std::numeric_limits<double>::quiet_NaN();

		TelemetryRecorder recorder;

		EXPECT_THROW(
			recorder.record(sample),
			std::invalid_argument
		);
	}


	TEST(
		TelemetryIntegrityTests,
		RejectsNonFiniteCommandedMoment
	)
	{
		TelemetrySample sample =
			validSample();

		sample.commandedMomentB.z =
			std::numeric_limits<double>::infinity();

		TelemetryRecorder recorder;

		EXPECT_THROW(
			recorder.record(sample),
			std::invalid_argument
		);
	}


	TEST(
		TelemetryIntegrityTests,
		RejectsNonFiniteActualMoment
	)
	{
		TelemetrySample sample =
			validSample();

		sample.actualMomentB.x =
			std::numeric_limits<double>::quiet_NaN();

		TelemetryRecorder recorder;

		EXPECT_THROW(
			recorder.record(sample),
			std::invalid_argument
		);
	}


	TEST(
		TelemetryIntegrityTests,
		RejectsNonFiniteThrottle
	)
	{
		TelemetrySample sample =
			validSample();

		sample.throttle =
			std::numeric_limits<double>::quiet_NaN();

		TelemetryRecorder recorder;

		EXPECT_THROW(
			recorder.record(sample),
			std::invalid_argument
		);
	}


	TEST(
		TelemetryIntegrityTests,
		RejectsNonFiniteGimbalPitch
	)
	{
		TelemetrySample sample =
			validSample();

		sample.gimbalPitchRad =
			std::numeric_limits<double>::infinity();

		TelemetryRecorder recorder;

		EXPECT_THROW(
			recorder.record(sample),
			std::invalid_argument
		);
	}


	TEST(
		TelemetryIntegrityTests,
		RejectsNonFiniteGimbalYaw
	)
	{
		TelemetrySample sample =
			validSample();

		sample.gimbalYawRad =
			std::numeric_limits<double>::quiet_NaN();

		TelemetryRecorder recorder;

		EXPECT_THROW(
			recorder.record(sample),
			std::invalid_argument
		);
	}


	TEST(
		TelemetryIntegrityTests,
		DoesNotRequireQuaternionToBeNormalized
	)
	{
		TelemetrySample sample =
			validSample();

		sample.attitudeNB =
			math::Quaternion(
				2.0,
				0.0,
				0.0,
				0.0
			);

		TelemetryRecorder recorder;

		EXPECT_NO_THROW(
			recorder.record(sample)
		);

		ASSERT_EQ(
			recorder.size(),
			1u
		);

		EXPECT_DOUBLE_EQ(
			recorder.getSamples()[0].attitudeNB.w,
			2.0
		);
	}


	TEST(
		TelemetryIntegrityTests,
		RejectedSampleDoesNotModifyRecorder
	)
	{
		TelemetryRecorder recorder;

		TelemetrySample good =
			validSample();

		recorder.record(good);

		TelemetrySample bad =
			validSample();

		bad.timeS = 1.0;

		bad.actualMomentB.x =
			std::numeric_limits<double>::quiet_NaN();

		EXPECT_THROW(
			recorder.record(bad),
			std::invalid_argument
		);

		EXPECT_EQ(
			recorder.size(),
			1u
		);
	}
}