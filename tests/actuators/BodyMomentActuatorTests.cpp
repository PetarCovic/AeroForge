#include <gtest/gtest.h>

#include <actuators/BodyMomentActuator.h>

#include <math/Vector3.h>

#include <stdexcept>

namespace aeroforge::actuators::tests
{
	TEST(
		BodyMomentActuatorTests,
		StartsAtZeroMoment)
	{
		const BodyMomentActuator actuator(
			math::Vector3(
				10.0,
				20.0,
				30.0),
			math::Vector3(
				100.0,
				100.0,
				100.0));

		EXPECT_TRUE(
			actuator
			.getActualMomentB()
			.isZero());
	}

	TEST(
		BodyMomentActuatorTests,
		RateLimitsEachAxis)
	{
		BodyMomentActuator actuator(
			math::Vector3(
				10.0,
				20.0,
				30.0),
			math::Vector3(
				5.0,
				10.0,
				15.0));

		const math::Vector3 actual =
			actuator.update(
				math::Vector3(
					100.0,
					-100.0,
					100.0),
				1.0);

		EXPECT_NEAR(actual.x, 5.0, 1e-12);
		EXPECT_NEAR(actual.y, -10.0, 1e-12);
		EXPECT_NEAR(actual.z, 15.0, 1e-12);
	}

	TEST(
		BodyMomentActuatorTests,
		SaturatesAtMaximumMoment)
	{
		BodyMomentActuator actuator(
			math::Vector3(
				10.0,
				20.0,
				30.0),
			math::Vector3(
				1000.0,
				1000.0,
				1000.0));

		const math::Vector3 actual =
			actuator.update(
				math::Vector3(
					100.0,
					-100.0,
					100.0),
				1.0);

		EXPECT_DOUBLE_EQ(actual.x, 10.0);
		EXPECT_DOUBLE_EQ(actual.y, -20.0);
		EXPECT_DOUBLE_EQ(actual.z, 30.0);
	}

	TEST(
		BodyMomentActuatorTests,
		ZeroDeltaTimeMaintainsPreviousMoment)
	{
		BodyMomentActuator actuator(
			math::Vector3(
				10.0,
				10.0,
				10.0),
			math::Vector3(
				10.0,
				10.0,
				10.0));

		actuator.update(
			math::Vector3(
				5.0,
				0.0,
				0.0),
			1.0);

		const math::Vector3 actual =
			actuator.update(
				math::Vector3(
					-5.0,
					0.0,
					0.0),
				0.0);

		EXPECT_DOUBLE_EQ(actual.x, 5.0);
	}

	TEST(
		BodyMomentActuatorTests,
		RejectsNonPositiveLimits)
	{
		EXPECT_THROW(
			(BodyMomentActuator(
				math::Vector3(
					0.0,
					1.0,
					1.0),
				math::Vector3(
					1.0,
					1.0,
					1.0))),
			std::invalid_argument);
	}
}