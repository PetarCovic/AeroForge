#include <gtest/gtest.h>

#include <dynamics/RotationalKinematics.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace aeroforge::dynamics::tests
{
	TEST(
		RotationalKinematicsTests,
		ZeroAngularVelocityProducesZeroDerivative)
	{
		const math::Quaternion attitude(
			1.0,
			0.0,
			0.0,
			0.0);

		const math::Quaternion derivative =
			computeQuaternionDerivativeNB(
				attitude,
				math::Vector3(
					0.0,
					0.0,
					0.0));

		EXPECT_DOUBLE_EQ(derivative.w, 0.0);
		EXPECT_DOUBLE_EQ(derivative.x, 0.0);
		EXPECT_DOUBLE_EQ(derivative.y, 0.0);
		EXPECT_DOUBLE_EQ(derivative.z, 0.0);
	}

	TEST(
		RotationalKinematicsTests,
		IdentityQuaternionProducesExpectedDerivative)
	{
		const math::Quaternion attitude(
			1.0,
			0.0,
			0.0,
			0.0);

		const math::Quaternion derivative =
			computeQuaternionDerivativeNB(
				attitude,
				math::Vector3(
					2.0,
					4.0,
					6.0));

		EXPECT_DOUBLE_EQ(
			derivative.w,
			0.0);

		EXPECT_DOUBLE_EQ(
			derivative.x,
			1.0);

		EXPECT_DOUBLE_EQ(
			derivative.y,
			2.0);

		EXPECT_DOUBLE_EQ(
			derivative.z,
			3.0);
	}

	TEST(
		RotationalKinematicsTests,
		UsesBodyFrameAngularVelocityConvention)
	{
		const double rootHalf =
			std::sqrt(0.5);

		const math::Quaternion attitude(
			rootHalf,
			0.0,
			0.0,
			rootHalf);

		const math::Quaternion derivative =
			computeQuaternionDerivativeNB(
				attitude,
				math::Vector3(
					0.0,
					0.0,
					2.0));

		EXPECT_NEAR(
			derivative.w,
			-rootHalf,
			1e-12);

		EXPECT_NEAR(
			derivative.x,
			0.0,
			1e-12);

		EXPECT_NEAR(
			derivative.y,
			0.0,
			1e-12);

		EXPECT_NEAR(
			derivative.z,
			rootHalf,
			1e-12);
	}

	TEST(
		RotationalKinematicsTests,
		RejectsNonFiniteAngularVelocity)
	{
		const math::Quaternion attitude(
			1.0,
			0.0,
			0.0,
			0.0);

		EXPECT_THROW(
			computeQuaternionDerivativeNB(
				attitude,
				math::Vector3(
					std::numeric_limits<double>::
					quiet_NaN(),
					0.0,
					0.0)),
			std::invalid_argument);
	}

	TEST(
		RotationalKinematicsTests,
		RejectsZeroQuaternion)
	{
		const math::Quaternion attitude(
			0.0,
			0.0,
			0.0,
			0.0);

		EXPECT_THROW(
			computeQuaternionDerivativeNB(
				attitude,
				math::Vector3(
					1.0,
					0.0,
					0.0)),
			std::invalid_argument);
	}
}