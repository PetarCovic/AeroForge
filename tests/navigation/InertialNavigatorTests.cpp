#include <gtest/gtest.h>

#include <navigation/InertialNavigator.h>
#include <navigation/NavigationState.h>

#include <sensors/ImuModel.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <cmath>
#include <stdexcept>

namespace aeroforge::navigation::tests
{
	TEST(
		InertialNavigatorTests,
		StationaryImuMaintainsPositionAndVelocity)
	{
		const NavigationState initial(
			math::Vector3(),
			math::Vector3(),
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3());

		const sensors::ImuMeasurement imu{
			0.0,
			math::Vector3(
				0.0,
				0.0,
				-9.80665),
			math::Vector3()
		};

		const InertialNavigator navigator;

		const NavigationState next =
			navigator.propagate(
				initial,
				imu,
				math::Vector3(
					0.0,
					0.0,
					9.80665),
				1.0);

		EXPECT_NEAR(
			next.getPositionN().magnitude(),
			0.0,
			1e-12);

		EXPECT_NEAR(
			next.getVelocityN().magnitude(),
			0.0,
			1e-12);
	}

	TEST(
		InertialNavigatorTests,
		PropagatesConstantAcceleration)
	{
		const NavigationState initial(
			math::Vector3(),
			math::Vector3(),
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3());

		const sensors::ImuMeasurement imu{
			0.0,
			math::Vector3(
				2.0,
				0.0,
				-9.80665),
			math::Vector3()
		};

		const InertialNavigator navigator;

		const NavigationState next =
			navigator.propagate(
				initial,
				imu,
				math::Vector3(
					0.0,
					0.0,
					9.80665),
				1.0);

		EXPECT_NEAR(
			next.getPositionN().x,
			1.0,
			1e-12);

		EXPECT_NEAR(
			next.getVelocityN().x,
			2.0,
			1e-12);
	}

	TEST(
		InertialNavigatorTests,
		PropagatesAngularVelocity)
	{
		const NavigationState initial(
			math::Vector3(),
			math::Vector3(),
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3());

		const sensors::ImuMeasurement imu{
			0.0,
			math::Vector3(
				0.0,
				0.0,
				-9.80665),
			math::Vector3(
				0.0,
				0.0,
				1.0)
		};

		const InertialNavigator navigator;

		const NavigationState next =
			navigator.propagate(
				initial,
				imu,
				math::Vector3(
					0.0,
					0.0,
					9.80665),
				0.01);

		EXPECT_GT(
			next.getAttitudeNB().z,
			0.0);

		EXPECT_NEAR(
			next.getAngularVelocityB().z,
			1.0,
			1e-12);
	}

	TEST(
		InertialNavigatorTests,
		RejectsNegativeDeltaTime)
	{
		const NavigationState initial(
			math::Vector3(),
			math::Vector3(),
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3());

		const sensors::ImuMeasurement imu{
			0.0,
			math::Vector3(),
			math::Vector3()
		};

		const InertialNavigator navigator;

		EXPECT_THROW(
			navigator.propagate(
				initial,
				imu,
				math::Vector3(),
				-0.1),
			std::invalid_argument);
	}
}