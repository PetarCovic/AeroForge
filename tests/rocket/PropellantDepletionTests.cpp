#include <gtest/gtest.h>

#include <rocket/PropellantDepletion.h>
#include <rocket/PropellantTank.h>
#include <rocket/RocketEngine.h>
#include <rocket/ThrustCurve.h>

#include <math/Vector3.h>

#include <stdexcept>
#include <vector>

namespace aeroforge::rocket::tests
{
	namespace
	{
		RocketEngine makeOneKgPerSecondEngine()
		{
			// T = Isp * g0 gives exactly 1 kg/s.
			constexpr double isp =
				100.0;

			constexpr double thrust =
				isp * 9.80665;

			return RocketEngine(
				ThrustCurve(
					std::vector<ThrustPoint>{
						{0.0, thrust},
						{ 100.0, thrust }
			}),
				isp,
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(1.0, 0.0, 0.0));
		}
	}

	TEST(
		PropellantDepletionTests,
		ConsumesMassFlowTimesDeltaTime)
	{
		PropellantTank tank(
			10.0,
			5.0);

		const RocketEngine engine =
			makeOneKgPerSecondEngine();

		const double consumed =
			consumePropellantForStep(
				tank,
				engine,
				0.0,
				0.25,
				1.0);

		EXPECT_NEAR(
			consumed,
			0.25,
			1e-12);

		EXPECT_NEAR(
			tank.getPropellantMassKg(),
			4.75,
			1e-12);
	}

	TEST(
		PropellantDepletionTests,
		ThrottleReducesConsumption)
	{
		PropellantTank tank(
			10.0,
			5.0);

		const RocketEngine engine =
			makeOneKgPerSecondEngine();

		const double consumed =
			consumePropellantForStep(
				tank,
				engine,
				0.0,
				1.0,
				0.5);

		EXPECT_NEAR(
			consumed,
			0.5,
			1e-12);
	}

	TEST(
		PropellantDepletionTests,
		CannotConsumeMoreThanTankContains)
	{
		PropellantTank tank(
			10.0,
			0.2);

		const RocketEngine engine =
			makeOneKgPerSecondEngine();

		const double consumed =
			consumePropellantForStep(
				tank,
				engine,
				0.0,
				1.0,
				1.0);

		EXPECT_NEAR(
			consumed,
			0.2,
			1e-12);

		EXPECT_TRUE(
			tank.isEmpty());
	}

	TEST(
		PropellantDepletionTests,
		ZeroDeltaTimeConsumesNothing)
	{
		PropellantTank tank(
			10.0,
			5.0);

		const RocketEngine engine =
			makeOneKgPerSecondEngine();

		EXPECT_DOUBLE_EQ(
			consumePropellantForStep(
				tank,
				engine,
				0.0,
				0.0,
				1.0),
			0.0);
	}

	TEST(
		PropellantDepletionTests,
		RejectsNegativeDeltaTime)
	{
		PropellantTank tank(
			10.0,
			5.0);

		const RocketEngine engine =
			makeOneKgPerSecondEngine();

		EXPECT_THROW(
			consumePropellantForStep(
				tank,
				engine,
				0.0,
				-0.1,
				1.0),
			std::invalid_argument);
	}
}