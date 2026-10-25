#include <gtest/gtest.h>

#include <gnc/AttitudeError.h>

#include <math/Quaternion.h>

#include <cmath>

namespace aeroforge::gnc::tests
{
	TEST(
		AttitudeErrorTests,
		EqualAttitudesProduceZeroError)
	{
		const math::Vector3 error =
			computeAttitudeErrorVectorB(
				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0),
				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0));

		EXPECT_NEAR(
			error.magnitude(),
			0.0,
			1e-12);
	}

	TEST(
		AttitudeErrorTests,
		PositiveDesiredYawProducesPositiveZError)
	{
		const double angle =
			20.0
			* 3.14159265358979323846
			/ 180.0;

		const math::Quaternion desired(
			std::cos(angle / 2.0),
			0.0,
			0.0,
			std::sin(angle / 2.0));

		const math::Vector3 error =
			computeAttitudeErrorVectorB(
				desired,
				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0));

		EXPECT_GT(
			error.z,
			0.0);

		EXPECT_NEAR(
			error.z,
			2.0 * std::sin(angle / 2.0),
			1e-12);
	}

	TEST(
		AttitudeErrorTests,
		PositiveCurrentYawWithIdentityTargetProducesNegativeZError)
	{
		const double angle =
			20.0
			* 3.14159265358979323846
			/ 180.0;

		const math::Quaternion current(
			std::cos(angle / 2.0),
			0.0,
			0.0,
			std::sin(angle / 2.0));

		const math::Vector3 error =
			computeAttitudeErrorVectorB(
				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0),
				current);

		EXPECT_LT(
			error.z,
			0.0);
	}

	TEST(
		AttitudeErrorTests,
		QuaternionSignDoesNotChangePhysicalError)
	{
		const double angle =
			0.2;

		const math::Quaternion desired(
			std::cos(angle / 2.0),
			0.0,
			std::sin(angle / 2.0),
			0.0);

		const math::Quaternion negativeDesired(
			-desired.w,
			-desired.x,
			-desired.y,
			-desired.z);

		const math::Quaternion current(
			1.0,
			0.0,
			0.0,
			0.0);

		const math::Vector3 first =
			computeAttitudeErrorVectorB(
				desired,
				current);

		const math::Vector3 second =
			computeAttitudeErrorVectorB(
				negativeDesired,
				current);

		EXPECT_NEAR(
			first.x,
			second.x,
			1e-12);

		EXPECT_NEAR(
			first.y,
			second.y,
			1e-12);

		EXPECT_NEAR(
			first.z,
			second.z,
			1e-12);
	}
}