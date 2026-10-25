#include <gtest/gtest.h>

#include <navigation/NavigationState.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <cmath>
#include <stdexcept>

namespace aeroforge::navigation::tests
{
	TEST(
		NavigationStateTests,
		StoresNavigationEstimate)
	{
		const NavigationState state(
			math::Vector3(1.0, 2.0, 3.0),
			math::Vector3(4.0, 5.0, 6.0),
			math::Quaternion(1.0, 0.0, 0.0, 0.0),
			math::Vector3(0.1, 0.2, 0.3));

		EXPECT_DOUBLE_EQ(
			state.getPositionN().x,
			1.0);

		EXPECT_DOUBLE_EQ(
			state.getVelocityN().y,
			5.0);

		EXPECT_DOUBLE_EQ(
			state.getAngularVelocityB().z,
			0.3);
	}

	TEST(
		NavigationStateTests,
		NormalizesAttitude)
	{
		const NavigationState state(
			math::Vector3(),
			math::Vector3(),
			math::Quaternion(2.0, 0.0, 0.0, 0.0),
			math::Vector3());

		EXPECT_NEAR(
			state.getAttitudeNB().w,
			1.0,
			1e-12);
	}

	TEST(
		NavigationStateTests,
		RejectsZeroQuaternion)
	{
		EXPECT_THROW(
			(NavigationState(
				math::Vector3(),
				math::Vector3(),
				math::Quaternion(
					0.0,
					0.0,
					0.0,
					0.0),
				math::Vector3())),
			std::invalid_argument);
	}
}