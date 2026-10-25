#include <gtest/gtest.h>

#include <gnc/AttitudePdController.h>
#include <gnc/AttitudeSetpoint.h>

#include <navigation/InertialNavigator.h>
#include <navigation/NavigationState.h>

#include <sensors/ImuModel.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <cmath>

namespace aeroforge::integration::tests
{
	TEST(
		GncNavigationIntegrationTests,
		ControllerRespondsToEstimatedAttitudeError)
	{
		const double yaw =
			10.0
			* 3.14159265358979323846
			/ 180.0;

		const navigation::NavigationState estimate(
			math::Vector3(),
			math::Vector3(),
			math::Quaternion(
				std::cos(yaw / 2.0),
				0.0,
				0.0,
				std::sin(yaw / 2.0)),
			math::Vector3());

		const gnc::AttitudeSetpoint target(
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3());

		const gnc::AttitudePdController controller(
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

		const math::Vector3 command =
			controller.computeMomentCommandB(
				target,
				estimate);

		EXPECT_LT(
			command.z,
			0.0);
	}
}