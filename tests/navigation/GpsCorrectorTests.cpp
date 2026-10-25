#include <gtest/gtest.h>

#include <navigation/GpsCorrector.h>
#include <navigation/NavigationState.h>

#include <sensors/GpsModel.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <stdexcept>

namespace aeroforge::navigation::tests
{
	TEST(
		GpsCorrectorTests,
		UnitGainSnapsPositionAndVelocityToGps)
	{
		const GpsCorrector corrector(
			1.0,
			1.0);

		const NavigationState state(
			math::Vector3(),
			math::Vector3(),
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3());

		const sensors::GpsMeasurement gps{
			1.0,
			math::Vector3(10.0, 20.0, 30.0),
			math::Vector3(1.0, 2.0, 3.0)
		};

		const NavigationState corrected =
			corrector.correct(
				state,
				gps);

		EXPECT_DOUBLE_EQ(
			corrected.getPositionN().x,
			10.0);

		EXPECT_DOUBLE_EQ(
			corrected.getVelocityN().z,
			3.0);
	}

	TEST(
		GpsCorrectorTests,
		PartialGainMovesEstimateTowardGps)
	{
		const GpsCorrector corrector(
			0.25,
			0.5);

		const NavigationState state(
			math::Vector3(),
			math::Vector3(),
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3());

		const sensors::GpsMeasurement gps{
			1.0,
			math::Vector3(8.0, 0.0, 0.0),
			math::Vector3(4.0, 0.0, 0.0)
		};

		const NavigationState corrected =
			corrector.correct(
				state,
				gps);

		EXPECT_NEAR(
			corrected.getPositionN().x,
			2.0,
			1e-12);

		EXPECT_NEAR(
			corrected.getVelocityN().x,
			2.0,
			1e-12);
	}

	TEST(
		GpsCorrectorTests,
		DoesNotModifyAttitudeOrAngularVelocity)
	{
		const GpsCorrector corrector(
			1.0,
			1.0);

		const NavigationState state(
			math::Vector3(),
			math::Vector3(),
			math::Quaternion(
				0.9,
				0.0,
				0.0,
				0.1),
			math::Vector3(
				0.1,
				0.2,
				0.3));

		const sensors::GpsMeasurement gps{
			1.0,
			math::Vector3(5.0, 6.0, 7.0),
			math::Vector3(1.0, 2.0, 3.0)
		};

		const NavigationState corrected =
			corrector.correct(
				state,
				gps);

		EXPECT_NEAR(
			corrected.getAttitudeNB().w,
			state.getAttitudeNB().w,
			1e-12);

		EXPECT_NEAR(
			corrected.getAngularVelocityB().z,
			0.3,
			1e-12);
	}

	TEST(
		GpsCorrectorTests,
		RejectsGainOutsideZeroOne)
	{
		EXPECT_THROW(
			GpsCorrector(
				-0.1,
				0.5),
			std::invalid_argument);

		EXPECT_THROW(
			GpsCorrector(
				0.5,
				1.1),
			std::invalid_argument);
	}
}