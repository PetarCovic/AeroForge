#include <gtest/gtest.h>

#include <aerodynamics/AerodynamicCoefficients.h>
#include <aerodynamics/AerodynamicReference.h>
#include <aerodynamics/SimpleAerodynamicModel.h>

#include <environment/ConstantWindModel.h>
#include <environment/RelativeAirVelocity.h>
#include <environment/StandardAtmosphere.h>

#include <math/Vector3.h>

namespace aeroforge::integration::tests
{
	TEST(
		AerodynamicIntegrationTests,
		EnvironmentFeedsAerodynamicModel)
	{
		const environment::StandardAtmosphere atmosphere;

		const environment::ConstantWindModel wind;

		const aerodynamics::SimpleAerodynamicModel model(
			aerodynamics::AerodynamicReference(
				0.1,
				1.0),
			aerodynamics::AerodynamicCoefficients(
				0.5,
				0.0,
				0.0));

		const math::Vector3 vehicleVelocityN(
			100.0,
			0.0,
			0.0);

		const math::Vector3 relativeVelocity =
			environment::
			computeRelativeAirVelocityN(
				vehicleVelocityN,
				wind.getWindVelocityN(
					0.0,
					0.0));

		const math::Vector3 force =
			model.computeForceBody(
				atmosphere.getAtmosphere(
					0.0),
				relativeVelocity);

		EXPECT_NEAR(
			force.x,
			-306.25,
			0.02);

		EXPECT_NEAR(force.y, 0.0, 1e-10);
		EXPECT_NEAR(force.z, 0.0, 1e-10);
	}

	TEST(
		AerodynamicIntegrationTests,
		TailwindReducesDrag)
	{
		const environment::StandardAtmosphere atmosphere;

		const environment::ConstantWindModel noWind;

		const environment::ConstantWindModel tailwind(
			math::Vector3(
				20.0,
				0.0,
				0.0));

		const aerodynamics::SimpleAerodynamicModel model(
			aerodynamics::AerodynamicReference(
				0.1,
				1.0),
			aerodynamics::AerodynamicCoefficients(
				0.5,
				0.0,
				0.0));

		const math::Vector3 vehicleVelocity(
			100.0,
			0.0,
			0.0);

		const math::Vector3 noWindRelative =
			environment::
			computeRelativeAirVelocityN(
				vehicleVelocity,
				noWind.getWindVelocityN(
					0.0,
					0.0));

		const math::Vector3 tailwindRelative =
			environment::
			computeRelativeAirVelocityN(
				vehicleVelocity,
				tailwind.getWindVelocityN(
					0.0,
					0.0));

		const environment::AtmosphereState air =
			atmosphere.getAtmosphere(
				0.0);

		const math::Vector3 noWindForce =
			model.computeForceBody(
				air,
				noWindRelative);

		const math::Vector3 tailwindForce =
			model.computeForceBody(
				air,
				tailwindRelative);

		EXPECT_LT(
			std::abs(tailwindForce.x),
			std::abs(noWindForce.x));
	}

	TEST(
		AerodynamicIntegrationTests,
		HigherAltitudeProducesLessDragAtSameAirspeed)
	{
		const environment::StandardAtmosphere atmosphere;

		const aerodynamics::SimpleAerodynamicModel model(
			aerodynamics::AerodynamicReference(
				0.1,
				1.0),
			aerodynamics::AerodynamicCoefficients(
				0.5,
				0.0,
				0.0));

		const math::Vector3 velocity(
			100.0,
			0.0,
			0.0);

		const math::Vector3 seaLevelForce =
			model.computeForceBody(
				atmosphere.getAtmosphere(
					0.0),
				velocity);

		const math::Vector3 highAltitudeForce =
			model.computeForceBody(
				atmosphere.getAtmosphere(
					10000.0),
				velocity);

		EXPECT_LT(
			std::abs(highAltitudeForce.x),
			std::abs(seaLevelForce.x));
	}

	TEST(
		AerodynamicIntegrationTests,
		CrosswindChangesAerodynamicForceDirection)
	{
		const environment::StandardAtmosphere atmosphere;

		const environment::ConstantWindModel wind(
			math::Vector3(
				0.0,
				20.0,
				0.0));

		const aerodynamics::SimpleAerodynamicModel model(
			aerodynamics::AerodynamicReference(
				0.1,
				1.0),
			aerodynamics::AerodynamicCoefficients(
				0.5,
				0.0,
				0.0));

		const math::Vector3 relativeVelocity =
			environment::
			computeRelativeAirVelocityN(
				math::Vector3(
					100.0,
					0.0,
					0.0),
				wind.getWindVelocityN(
					0.0,
					0.0));

		const math::Vector3 force =
			model.computeForceBody(
				atmosphere.getAtmosphere(
					0.0),
				relativeVelocity);

		// Relative Y velocity is negative,
		// so drag contributes +Y.
		EXPECT_GT(force.y, 0.0);
	}

	TEST(
		AerodynamicIntegrationTests,
		RepeatedCalculationIsDeterministic)
	{
		const environment::StandardAtmosphere atmosphere;

		const aerodynamics::SimpleAerodynamicModel model(
			aerodynamics::AerodynamicReference(
				0.15,
				1.5),
			aerodynamics::AerodynamicCoefficients(
				0.4,
				0.7,
				0.1));

		const environment::AtmosphereState air =
			atmosphere.getAtmosphere(
				5000.0);

		const math::Vector3 velocity(
			120.0,
			5.0,
			10.0);

		const math::Vector3 first =
			model.computeForceBody(
				air,
				velocity);

		const math::Vector3 second =
			model.computeForceBody(
				air,
				velocity);

		EXPECT_DOUBLE_EQ(first.x, second.x);
		EXPECT_DOUBLE_EQ(first.y, second.y);
		EXPECT_DOUBLE_EQ(first.z, second.z);
	}
}