#include <gtest/gtest.h>

#include <actuators/GimbalActuator.h>

#include <cmath>
#include <stdexcept>

namespace aeroforge::actuators::tests
{
	TEST(
		GimbalActuatorTests,
		StartsCentered)
	{
		const GimbalActuator actuator(
			0.2,
			1.0);

		EXPECT_DOUBLE_EQ(
			actuator.getAngles().pitchRad,
			0.0);

		EXPECT_DOUBLE_EQ(
			actuator.getAngles().yawRad,
			0.0);
	}

	TEST(
		GimbalActuatorTests,
		RateLimitsBothAxes)
	{
		GimbalActuator actuator(
			0.5,
			0.1);

		const GimbalAngles angles =
			actuator.update(
				0.5,
				-0.5,
				1.0);

		EXPECT_NEAR(
			angles.pitchRad,
			0.1,
			1e-12);

		EXPECT_NEAR(
			angles.yawRad,
			-0.1,
			1e-12);
	}

	TEST(
		GimbalActuatorTests,
		ClampsToMaximumAngle)
	{
		GimbalActuator actuator(
			0.2,
			10.0);

		const GimbalAngles angles =
			actuator.update(
				1.0,
				-1.0,
				1.0);

		EXPECT_NEAR(
			angles.pitchRad,
			0.2,
			1e-12);

		EXPECT_NEAR(
			angles.yawRad,
			-0.2,
			1e-12);
	}

	TEST(
		GimbalActuatorTests,
		CenteredGimbalPointsAlongBodyX)
	{
		const GimbalActuator actuator(
			0.2,
			1.0);

		const math::Vector3 direction =
			actuator.getThrustDirectionB();

		EXPECT_NEAR(direction.x, 1.0, 1e-12);
		EXPECT_NEAR(direction.y, 0.0, 1e-12);
		EXPECT_NEAR(direction.z, 0.0, 1e-12);
	}

	TEST(
		GimbalActuatorTests,
		PositivePitchDeflectsTowardPositiveBodyZ)
	{
		GimbalActuator actuator(
			0.5,
			10.0);

		actuator.update(
			0.1,
			0.0,
			1.0);

		const math::Vector3 direction =
			actuator.getThrustDirectionB();

		EXPECT_NEAR(
			direction.x,
			std::cos(0.1),
			1e-12);

		EXPECT_NEAR(
			direction.z,
			std::sin(0.1),
			1e-12);
	}

	TEST(
		GimbalActuatorTests,
		OutputDirectionIsNormalized)
	{
		GimbalActuator actuator(
			0.5,
			10.0);

		actuator.update(
			0.2,
			-0.15,
			1.0);

		EXPECT_NEAR(
			actuator
			.getThrustDirectionB()
			.magnitude(),
			1.0,
			1e-12);
	}

	TEST(
		GimbalActuatorTests,
		RejectsInvalidConfiguration)
	{
		EXPECT_THROW(
			GimbalActuator(
				0.0,
				1.0),
			std::invalid_argument);

		EXPECT_THROW(
			GimbalActuator(
				0.2,
				0.0),
			std::invalid_argument);
	}
}