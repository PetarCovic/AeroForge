#include <gtest/gtest.h>

#include <actuators/ThrottleActuator.h>

#include <stdexcept>

namespace aeroforge::actuators::tests
{
	TEST(
		ThrottleActuatorTests,
		StoresInitialThrottle)
	{
		const ThrottleActuator actuator(
			0.25,
			1.0);

		EXPECT_DOUBLE_EQ(
			actuator.getThrottle(),
			0.25);
	}

	TEST(
		ThrottleActuatorTests,
		RateLimitsIncreasingThrottle)
	{
		ThrottleActuator actuator(
			0.0,
			0.5);

		const double actual =
			actuator.update(
				1.0,
				1.0);

		EXPECT_NEAR(
			actual,
			0.5,
			1e-12);
	}

	TEST(
		ThrottleActuatorTests,
		ReachesCommandAcrossMultipleUpdates)
	{
		ThrottleActuator actuator(
			0.0,
			0.5);

		actuator.update(
			1.0,
			1.0);

		const double actual =
			actuator.update(
				1.0,
				1.0);

		EXPECT_NEAR(
			actual,
			1.0,
			1e-12);
	}

	TEST(
		ThrottleActuatorTests,
		ClampsCommandToPhysicalRange)
	{
		ThrottleActuator actuator(
			0.5,
			10.0);

		EXPECT_DOUBLE_EQ(
			actuator.update(
				2.0,
				1.0),
			1.0);

		EXPECT_DOUBLE_EQ(
			actuator.update(
				-1.0,
				1.0),
			0.0);
	}

	TEST(
		ThrottleActuatorTests,
		ZeroDeltaTimeDoesNotMoveActuator)
	{
		ThrottleActuator actuator(
			0.25,
			1.0);

		EXPECT_DOUBLE_EQ(
			actuator.update(
				1.0,
				0.0),
			0.25);
	}

	TEST(
		ThrottleActuatorTests,
		RejectsInvalidConfiguration)
	{
		EXPECT_THROW(
			ThrottleActuator(
				-0.1,
				1.0),
			std::invalid_argument);

		EXPECT_THROW(
			ThrottleActuator(
				0.5,
				0.0),
			std::invalid_argument);
	}
}