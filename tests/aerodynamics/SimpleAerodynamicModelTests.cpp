#include <gtest/gtest.h>

#include <aerodynamics/AerodynamicCoefficients.h>
#include <aerodynamics/AerodynamicReference.h>
#include <aerodynamics/SimpleAerodynamicModel.h>

#include <environment/AtmosphereState.h>

#include <math/Vector3.h>

#include <cmath>

namespace aeroforge::aerodynamics::tests
{
	namespace
	{
		environment::AtmosphereState seaLevel()
		{
			return environment::AtmosphereState(
				288.15,
				101325.0,
				1.225,
				340.294);
		}
	}

	TEST(
		SimpleAerodynamicModelTests,
		ZeroAirspeedProducesZeroForce)
	{
		const AerodynamicReference reference(
			0.1,
			1.0);

		const AerodynamicCoefficients coefficients(
			0.5,
			1.0,
			0.2);

		const SimpleAerodynamicModel model(
			reference,
			coefficients);

		const math::Vector3 force =
			model.computeForceBody(
				seaLevel(),
				math::Vector3(
					0.0,
					0.0,
					0.0));

		EXPECT_DOUBLE_EQ(force.x, 0.0);
		EXPECT_DOUBLE_EQ(force.y, 0.0);
		EXPECT_DOUBLE_EQ(force.z, 0.0);
	}

	TEST(
		SimpleAerodynamicModelTests,
		ZeroCoefficientsProduceZeroForce)
	{
		const SimpleAerodynamicModel model(
			AerodynamicReference(
				0.1,
				1.0),
			AerodynamicCoefficients(
				0.0,
				0.0,
				0.0));

		const math::Vector3 force =
			model.computeForceBody(
				seaLevel(),
				math::Vector3(
					100.0,
					0.0,
					0.0));

		EXPECT_DOUBLE_EQ(force.x, 0.0);
		EXPECT_DOUBLE_EQ(force.y, 0.0);
		EXPECT_DOUBLE_EQ(force.z, 0.0);
	}

	TEST(
		SimpleAerodynamicModelTests,
		AlignedFlowProducesExpectedForces)
	{
		const SimpleAerodynamicModel model(
			AerodynamicReference(
				0.1,
				1.0),
			AerodynamicCoefficients(
				0.5,
				1.0,
				0.2));

		const math::Vector3 force =
			model.computeForceBody(
				seaLevel(),
				math::Vector3(
					100.0,
					0.0,
					0.0));

		EXPECT_NEAR(
			force.x,
			-306.25,
			1e-10);

		EXPECT_NEAR(
			force.y,
			122.5,
			1e-10);

		EXPECT_NEAR(
			force.z,
			-612.5,
			1e-10);
	}

	TEST(
		SimpleAerodynamicModelTests,
		DragOpposesForwardMotion)
	{
		const SimpleAerodynamicModel model(
			AerodynamicReference(
				0.1,
				1.0),
			AerodynamicCoefficients(
				0.5,
				0.0,
				0.0));

		const math::Vector3 force =
			model.computeForceBody(
				seaLevel(),
				math::Vector3(
					100.0,
					0.0,
					0.0));

		EXPECT_LT(force.x, 0.0);
		EXPECT_NEAR(force.y, 0.0, 1e-12);
		EXPECT_NEAR(force.z, 0.0, 1e-12);
	}

	TEST(
		SimpleAerodynamicModelTests,
		PositiveLiftActsUpwardAtZeroAlpha)
	{
		const SimpleAerodynamicModel model(
			AerodynamicReference(
				0.1,
				1.0),
			AerodynamicCoefficients(
				0.0,
				1.0,
				0.0));

		const math::Vector3 force =
			model.computeForceBody(
				seaLevel(),
				math::Vector3(
					100.0,
					0.0,
					0.0));

		// Body +Z is down, so lift is -Z.
		EXPECT_LT(force.z, 0.0);
	}

	TEST(
		SimpleAerodynamicModelTests,
		PositiveSideForceActsToBodyRightAtZeroBeta)
	{
		const SimpleAerodynamicModel model(
			AerodynamicReference(
				0.1,
				1.0),
			AerodynamicCoefficients(
				0.0,
				0.0,
				1.0));

		const math::Vector3 force =
			model.computeForceBody(
				seaLevel(),
				math::Vector3(
					100.0,
					0.0,
					0.0));

		EXPECT_GT(force.y, 0.0);
	}

	TEST(
		SimpleAerodynamicModelTests,
		DragRemainsOppositeRelativeVelocityAtPositiveAlpha)
	{
		const double speed =
			100.0;

		const double alpha =
			0.3;

		const math::Vector3 velocity(
			speed * std::cos(alpha),
			0.0,
			speed * std::sin(alpha));

		const SimpleAerodynamicModel model(
			AerodynamicReference(
				0.1,
				1.0),
			AerodynamicCoefficients(
				0.5,
				0.0,
				0.0));

		const math::Vector3 force =
			model.computeForceBody(
				seaLevel(),
				velocity);

		EXPECT_LT(force.x, 0.0);
		EXPECT_LT(force.z, 0.0);

		const double dot =
			force.x * velocity.x
			+ force.y * velocity.y
			+ force.z * velocity.z;

		EXPECT_LT(dot, 0.0);
	}

	TEST(
		SimpleAerodynamicModelTests,
		ForceScalesWithReferenceArea)
	{
		const environment::AtmosphereState atmosphere =
			seaLevel();

		const AerodynamicCoefficients coefficients(
			0.5,
			0.0,
			0.0);

		const SimpleAerodynamicModel small(
			AerodynamicReference(
				0.1,
				1.0),
			coefficients);

		const SimpleAerodynamicModel large(
			AerodynamicReference(
				0.2,
				1.0),
			coefficients);

		const math::Vector3 velocity(
			100.0,
			0.0,
			0.0);

		const math::Vector3 smallForce =
			small.computeForceBody(
				atmosphere,
				velocity);

		const math::Vector3 largeForce =
			large.computeForceBody(
				atmosphere,
				velocity);

		EXPECT_NEAR(
			largeForce.x,
			2.0 * smallForce.x,
			1e-10);
	}
}