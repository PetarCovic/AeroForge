#include <gtest/gtest.h>

#include <actuators/GimbalActuator.h>
#include <actuators/GimbaledThrust.h>

#include <rocket/RocketEngine.h>
#include <rocket/ThrustCurve.h>

#include <math/Vector3.h>

#include <cmath>
#include <vector>

namespace aeroforge::actuators::tests
{
	TEST(
		GimbaledThrustTests,
		CenteredGimbalProducesBodyXThrust)
	{
		const rocket::RocketEngine engine(
			rocket::ThrustCurve(
				std::vector<rocket::ThrustPoint>{
					{0.0, 100.0},
					{ 10.0, 100.0 }
		}),
			200.0,
			math::Vector3(),
			math::Vector3(
				1.0,
				0.0,
				0.0));

		const GimbalActuator gimbal(
			0.5,
			1.0);

		const math::Vector3 force =
			computeGimbaledThrustForceB(
				engine,
				1.0,
				1.0,
				gimbal);

		EXPECT_NEAR(force.x, 100.0, 1e-12);
		EXPECT_NEAR(force.y, 0.0, 1e-12);
		EXPECT_NEAR(force.z, 0.0, 1e-12);
	}

	TEST(
		GimbaledThrustTests,
		GimbalChangesThrustDirectionButNotMagnitude)
	{
		const rocket::RocketEngine engine(
			rocket::ThrustCurve(
				std::vector<rocket::ThrustPoint>{
					{0.0, 100.0},
					{ 10.0, 100.0 }
		}),
			200.0,
			math::Vector3(),
			math::Vector3(
				1.0,
				0.0,
				0.0));

		GimbalActuator gimbal(
			0.5,
			10.0);

		gimbal.update(
			0.1,
			0.2,
			1.0);

		const math::Vector3 force =
			computeGimbaledThrustForceB(
				engine,
				1.0,
				1.0,
				gimbal);

		EXPECT_NEAR(
			force.magnitude(),
			100.0,
			1e-10);

		EXPECT_GT(
			force.y,
			0.0);

		EXPECT_GT(
			force.z,
			0.0);
	}

	TEST(
		GimbaledThrustTests,
		ThrottleStillControlsMagnitude)
	{
		const rocket::RocketEngine engine(
			rocket::ThrustCurve(
				std::vector<rocket::ThrustPoint>{
					{0.0, 100.0},
					{ 10.0, 100.0 }
		}),
			200.0,
			math::Vector3(),
			math::Vector3(
				1.0,
				0.0,
				0.0));

		const GimbalActuator gimbal(
			0.5,
			10.0);

		const math::Vector3 force =
			computeGimbaledThrustForceB(
				engine,
				1.0,
				0.25,
				gimbal);

		EXPECT_NEAR(
			force.magnitude(),
			25.0,
			1e-12);
	}
}