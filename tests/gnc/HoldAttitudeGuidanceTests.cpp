#include <gtest/gtest.h>

#include <gnc/AttitudeSetpoint.h>
#include <gnc/HoldAttitudeGuidance.h>

#include <math/Quaternion.h>
#include <math/Vector3.h>

#include <stdexcept>

namespace aeroforge::gnc::tests
{
	TEST(
		HoldAttitudeGuidanceTests,
		ReturnsSameSetpointOverTime)
	{
		const AttitudeSetpoint target(
			math::Quaternion(
				1.0,
				0.0,
				0.0,
				0.0),
			math::Vector3());

		const HoldAttitudeGuidance guidance(
			target);

		const AttitudeSetpoint first =
			guidance.getSetpoint(
				0.0);

		const AttitudeSetpoint second =
			guidance.getSetpoint(
				10.0);

		EXPECT_DOUBLE_EQ(
			first.getAttitudeNB().w,
			second.getAttitudeNB().w);
	}

	TEST(
		HoldAttitudeGuidanceTests,
		RejectsInvalidTime)
	{
		const HoldAttitudeGuidance guidance(
			AttitudeSetpoint(
				math::Quaternion(
					1.0,
					0.0,
					0.0,
					0.0),
				math::Vector3()));

		EXPECT_THROW(
			guidance.getSetpoint(
				-1.0),
			std::invalid_argument);
	}
}