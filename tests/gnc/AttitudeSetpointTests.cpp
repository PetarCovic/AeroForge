#include <gtest/gtest.h>

#include <gnc/AttitudeSetpoint.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <stdexcept>

namespace aeroforge::gnc::tests
{
	TEST(
		AttitudeSetpointTests,
		StoresDesiredState)
	{
		const AttitudeSetpoint setpoint(
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3(
				0.1,
				0.2,
				0.3));

		EXPECT_DOUBLE_EQ(
			setpoint.getAttitudeNB().w,
			1.0);

		EXPECT_DOUBLE_EQ(
			setpoint
			.getAngularVelocityB()
			.z,
			0.3);
	}

	TEST(
		AttitudeSetpointTests,
		NormalizesDesiredQuaternion)
	{
		const AttitudeSetpoint setpoint(
			math::Quaternion(
				2.0,
				0.0,
				0.0,
				0.0),
			math::Vector3());

		EXPECT_NEAR(
			setpoint.getAttitudeNB().w,
			1.0,
			1e-12);
	}

	TEST(
		AttitudeSetpointTests,
		RejectsZeroQuaternion)
	{
		EXPECT_THROW(
			(AttitudeSetpoint(
				math::Quaternion(
					0.0,
					0.0,
					0.0,
					0.0),
				math::Vector3())),
			std::invalid_argument);
	}
}