#include <gtest/gtest.h>

#include <aerodynamics/AerodynamicAngles.h>

#include <math/Vector3.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <stdexcept>

namespace aeroforge::aerodynamics::tests
{
	TEST(
		AerodynamicAnglesTests,
		ForwardFlowHasZeroAngles)
	{
		const math::Vector3 velocity(
			100.0,
			0.0,
			0.0);

		EXPECT_NEAR(
			computeAngleOfAttackRad(velocity),
			0.0,
			1e-12);

		EXPECT_NEAR(
			computeSideslipAngleRad(velocity),
			0.0,
			1e-12);
	}

	TEST(
		AerodynamicAnglesTests,
		PositiveDownVelocityProducesPositiveAngleOfAttack)
	{
		const math::Vector3 velocity(
			100.0,
			0.0,
			10.0);

		EXPECT_NEAR(
			computeAngleOfAttackRad(velocity),
			std::atan2(10.0, 100.0),
			1e-12);
	}

	TEST(
		AerodynamicAnglesTests,
		NegativeDownVelocityProducesNegativeAngleOfAttack)
	{
		const math::Vector3 velocity(
			100.0,
			0.0,
			-10.0);

		EXPECT_LT(
			computeAngleOfAttackRad(velocity),
			0.0);
	}

	TEST(
		AerodynamicAnglesTests,
		PositiveRightVelocityProducesPositiveSideslip)
	{
		const math::Vector3 velocity(
			100.0,
			10.0,
			0.0);

		EXPECT_NEAR(
			computeSideslipAngleRad(velocity),
			std::atan2(10.0, 100.0),
			1e-12);
	}

	TEST(
		AerodynamicAnglesTests,
		NegativeRightVelocityProducesNegativeSideslip)
	{
		const math::Vector3 velocity(
			100.0,
			-10.0,
			0.0);

		EXPECT_LT(
			computeSideslipAngleRad(velocity),
			0.0);
	}

	TEST(
		AerodynamicAnglesTests,
		ZeroAirspeedReturnsZeroAngles)
	{
		const math::Vector3 velocity(
			0.0,
			0.0,
			0.0);

		EXPECT_DOUBLE_EQ(
			computeAngleOfAttackRad(velocity),
			0.0);

		EXPECT_DOUBLE_EQ(
			computeSideslipAngleRad(velocity),
			0.0);
	}

	TEST(
		AerodynamicAnglesTests,
		RejectsNonFiniteVelocity)
	{
		const math::Vector3 velocity(
			std::numeric_limits<double>::
			quiet_NaN(),
			0.0,
			0.0);

		EXPECT_THROW(
			computeAngleOfAttackRad(
				velocity),
			std::invalid_argument);

		EXPECT_THROW(
			computeSideslipAngleRad(
				velocity),
			std::invalid_argument);
	}
}