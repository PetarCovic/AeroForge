#include <gtest/gtest.h>

#include <rocket/RocketEngine.h>
#include <rocket/ThrustCurve.h>

#include <math/Vector3.h>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace aeroforge::rocket::tests
{
	namespace
	{
		ThrustCurve makeCurve()
		{
			return ThrustCurve(
				std::vector<ThrustPoint>{
					{0.0, 100.0},
					{ 10.0, 100.0 }
			});
		}
	}

	TEST(
		RocketEngineTests,
		NormalizesThrustDirection)
	{
		const RocketEngine engine(
			makeCurve(),
			200.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(2.0, 0.0, 0.0));

		EXPECT_NEAR(
			engine.getThrustDirectionB().x,
			1.0,
			1e-12);

		EXPECT_NEAR(
			engine.getThrustDirectionB().magnitude(),
			1.0,
			1e-12);
	}

	TEST(
		RocketEngineTests,
		ComputesBodyFrameThrustForce)
	{
		const RocketEngine engine(
			makeCurve(),
			200.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(1.0, 0.0, 0.0));

		const math::Vector3 force =
			engine.computeThrustForceB(
				5.0,
				1.0);

		EXPECT_NEAR(force.x, 100.0, 1e-12);
		EXPECT_NEAR(force.y, 0.0, 1e-12);
		EXPECT_NEAR(force.z, 0.0, 1e-12);
	}

	TEST(
		RocketEngineTests,
		ThrottleScalesThrust)
	{
		const RocketEngine engine(
			makeCurve(),
			200.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(1.0, 0.0, 0.0));

		EXPECT_NEAR(
			engine.getThrustN(
				5.0,
				0.25),
			25.0,
			1e-12);
	}

	TEST(
		RocketEngineTests,
		ZeroThrottleProducesZeroForceAndMassFlow)
	{
		const RocketEngine engine(
			makeCurve(),
			200.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(1.0, 0.0, 0.0));

		EXPECT_DOUBLE_EQ(
			engine.getThrustN(
				5.0,
				0.0),
			0.0);

		EXPECT_DOUBLE_EQ(
			engine.computeMassFlowRateKgPerS(
				5.0,
				0.0),
			0.0);
	}

	TEST(
		RocketEngineTests,
		ComputesMassFlowFromSpecificImpulse)
	{
		const RocketEngine engine(
			makeCurve(),
			200.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(1.0, 0.0, 0.0));

		const double expected =
			100.0
			/ (200.0 * 9.80665);

		EXPECT_NEAR(
			engine.computeMassFlowRateKgPerS(
				5.0,
				1.0),
			expected,
			1e-12);
	}

	TEST(
		RocketEngineTests,
		RejectsNonPositiveSpecificImpulse)
	{
		EXPECT_THROW(
			(RocketEngine(
				makeCurve(),
				0.0,
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(1.0, 0.0, 0.0))),
			std::invalid_argument);
	}

	TEST(
		RocketEngineTests,
		RejectsZeroThrustDirection)
	{
		EXPECT_THROW(
			(RocketEngine(
				makeCurve(),
				200.0,
				math::Vector3(0.0, 0.0, 0.0),
				math::Vector3(0.0, 0.0, 0.0))),
			std::invalid_argument);
	}

	TEST(
		RocketEngineTests,
		RejectsInvalidThrottle)
	{
		const RocketEngine engine(
			makeCurve(),
			200.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Vector3(1.0, 0.0, 0.0));

		EXPECT_THROW(
			engine.getThrustN(
				1.0,
				-0.1),
			std::invalid_argument);

		EXPECT_THROW(
			engine.getThrustN(
				1.0,
				1.1),
			std::invalid_argument);
	}
}