#include <gtest/gtest.h>

#include <gnc/AttitudePdController.h>
#include <gnc/AttitudeSetpoint.h>

#include <navigation/NavigationState.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <cmath>
#include <stdexcept>

namespace aeroforge::gnc::tests
{
	TEST(
		AttitudePdControllerTests,
		ZeroErrorProducesZeroMoment)
	{
		const AttitudePdController controller(
			math::Vector3(
				10.0,
				10.0,
				10.0),
			math::Vector3(
				2.0,
				2.0,
				2.0),
			math::Vector3(
				100.0,
				100.0,
				100.0));

		const AttitudeSetpoint setpoint(
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3());

		const navigation::NavigationState state(
			math::Vector3(),
			math::Vector3(),
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3());

		const math::Vector3 moment =
			controller.computeMomentCommandB(
				setpoint,
				state);

		EXPECT_NEAR(
			moment.magnitude(),
			0.0,
			1e-12);
	}

	TEST(
		AttitudePdControllerTests,
		PositiveAttitudeErrorProducesCorrectiveMoment)
	{
		const AttitudePdController controller(
			math::Vector3(
				10.0,
				10.0,
				10.0),
			math::Vector3(),
			math::Vector3(
				100.0,
				100.0,
				100.0));

		const double angle =
			0.2;

		const AttitudeSetpoint setpoint(
			math::Quaternion(
				std::cos(angle / 2.0),
				0.0,
				0.0,
				std::sin(angle / 2.0)),
			math::Vector3());

		const navigation::NavigationState state(
			math::Vector3(),
			math::Vector3(),
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3());

		const math::Vector3 moment =
			controller.computeMomentCommandB(
				setpoint,
				state);

		EXPECT_GT(
			moment.z,
			0.0);
	}

	TEST(
		AttitudePdControllerTests,
		AngularVelocityFeedbackProvidesDamping)
	{
		const AttitudePdController controller(
			math::Vector3(),
			math::Vector3(
				2.0,
				2.0,
				2.0),
			math::Vector3(
				100.0,
				100.0,
				100.0));

		const AttitudeSetpoint setpoint(
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3());

		const navigation::NavigationState state(
			math::Vector3(),
			math::Vector3(),
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3(
				0.0,
				0.0,
				2.0));

		const math::Vector3 moment =
			controller.computeMomentCommandB(
				setpoint,
				state);

		EXPECT_LT(
			moment.z,
			0.0);
	}

	TEST(
		AttitudePdControllerTests,
		SaturatesMomentCommand)
	{
		const AttitudePdController controller(
			math::Vector3(
				1000.0,
				1000.0,
				1000.0),
			math::Vector3(),
			math::Vector3(
				1.0,
				2.0,
				3.0));

		const double angle =
			1.0;

		const AttitudeSetpoint setpoint(
			math::Quaternion(
				std::cos(angle / 2.0),
				0.0,
				0.0,
				std::sin(angle / 2.0)),
			math::Vector3());

		const navigation::NavigationState state(
			math::Vector3(),
			math::Vector3(),
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3());

		const math::Vector3 moment =
			controller.computeMomentCommandB(
				setpoint,
				state);

		EXPECT_NEAR(
			moment.z,
			3.0,
			1e-12);
	}

	TEST(
		AttitudePdControllerTests,
		RejectsNegativeGain)
	{
		EXPECT_THROW(
			(AttitudePdController(
				math::Vector3(
					-1.0,
					1.0,
					1.0),
				math::Vector3(),
				math::Vector3(
					10.0,
					10.0,
					10.0))),
			std::invalid_argument);
	}
}