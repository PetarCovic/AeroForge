#include <gtest/gtest.h>

#include <rocket/ThrustMoment.h>

#include <math/Vector3.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::rocket::tests
{
	TEST(
		ThrustMomentTests,
		ForceThroughCenterOfMassProducesNoMoment)
	{
		const math::Vector3 moment =
			computeThrustMomentB(
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(100.0, 0.0, 0.0));

		EXPECT_DOUBLE_EQ(moment.x, 0.0);
		EXPECT_DOUBLE_EQ(moment.y, 0.0);
		EXPECT_DOUBLE_EQ(moment.z, 0.0);
	}

	TEST(
		ThrustMomentTests,
		OffsetEngineProducesExpectedMoment)
	{
		const math::Vector3 moment =
			computeThrustMomentB(
				math::Vector3(0.0, 1.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(100.0, 0.0, 0.0));

		EXPECT_NEAR(moment.x, 0.0, 1e-12);
		EXPECT_NEAR(moment.y, 0.0, 1e-12);
		EXPECT_NEAR(moment.z, -100.0, 1e-12);
	}

	TEST(
		ThrustMomentTests,
		ChangingCenterOfMassChangesMomentArm)
	{
		const math::Vector3 first =
			computeThrustMomentB(
				math::Vector3(0.0, 1.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(100.0, 0.0, 0.0));

		const math::Vector3 second =
			computeThrustMomentB(
				math::Vector3(0.0, 1.0, 0.0),
				math::Vector3(0.0, 0.5, 0.0),
				math::Vector3(100.0, 0.0, 0.0));

		EXPECT_NEAR(
			second.z,
			0.5 * first.z,
			1e-12);
	}

	TEST(
		ThrustMomentTests,
		RejectsNonFiniteInput)
	{
		EXPECT_THROW(
			computeThrustMomentB(
				math::Vector3(
					std::numeric_limits<double>::
					infinity(),
					0.0,
					0.0),
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(1.0, 0.0, 0.0)),
			std::invalid_argument);
	}
}