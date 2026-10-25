#include <gtest/gtest.h>

#include <navigation/GpsCorrector.h>
#include <navigation/InertialNavigator.h>
#include <navigation/NavigationState.h>

#include <sensors/GpsModel.h>
#include <sensors/ImuModel.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

namespace aeroforge::integration::tests
{
	TEST(
		NavigationIntegrationTests,
		PerfectImuTracksConstantAcceleration)
	{
		navigation::NavigationState state(
			math::Vector3(),
			math::Vector3(),
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3());

		const navigation::InertialNavigator navigator;

		constexpr double dt =
			0.01;

		for (int i = 0; i < 100; ++i)
		{
			const sensors::ImuMeasurement imu{
				i * dt,
				math::Vector3(
					2.0,
					0.0,
					-9.80665),
				math::Vector3()
			};

			state =
				navigator.propagate(
					state,
					imu,
					math::Vector3(
						0.0,
						0.0,
						9.80665),
					dt);
		}

		EXPECT_NEAR(
			state.getPositionN().x,
			1.0,
			1e-9);

		EXPECT_NEAR(
			state.getVelocityN().x,
			2.0,
			1e-9);
	}

	TEST(
		NavigationIntegrationTests,
		GpsCorrectionReducesPositionError)
	{
		const navigation::NavigationState drifted(
			math::Vector3(
				20.0,
				0.0,
				0.0),
			math::Vector3(
				3.0,
				0.0,
				0.0),
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3());

		const sensors::GpsMeasurement gps{
			10.0,
			math::Vector3(
				10.0,
				0.0,
				0.0),
			math::Vector3(
				2.0,
				0.0,
				0.0)
		};

		const navigation::GpsCorrector corrector(
			0.5,
			0.5);

		const navigation::NavigationState corrected =
			corrector.correct(
				drifted,
				gps);

		const double beforeError =
			std::abs(
				drifted.getPositionN().x
				- gps.positionN.x);

		const double afterError =
			std::abs(
				corrected.getPositionN().x
				- gps.positionN.x);

		EXPECT_LT(
			afterError,
			beforeError);
	}
}