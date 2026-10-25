#include <gtest/gtest.h>

#include <environment/AtmosphereModel.h>
#include <environment/ConstantWindModel.h>
#include <environment/RelativeAirVelocity.h>
#include <environment/StandardAtmosphere.h>
#include <environment/WindModel.h>

#include <math/Vector3.h>

namespace aeroforge::integration::tests
{
	TEST(
		EnvironmentIntegrationTests,
		AtmosphereAndWindCanBeQueriedAtSameFlightCondition)
	{
		const environment::StandardAtmosphere atmosphere;
		const environment::ConstantWindModel wind(
			math::Vector3(
				10.0,
				5.0,
				-2.0));

		const double altitudeMeters =
			5000.0;

		const double timeSeconds =
			20.0;

		const environment::AtmosphereState air =
			atmosphere.getAtmosphere(
				altitudeMeters);

		const math::Vector3 windVelocity =
			wind.getWindVelocityN(
				altitudeMeters,
				timeSeconds);

		EXPECT_GT(
			air.getDensityKgPerM3(),
			0.0);

		EXPECT_GT(
			air.getSpeedOfSoundMps(),
			0.0);

		EXPECT_DOUBLE_EQ(
			windVelocity.x,
			10.0);

		EXPECT_DOUBLE_EQ(
			windVelocity.y,
			5.0);

		EXPECT_DOUBLE_EQ(
			windVelocity.z,
			-2.0);
	}

	TEST(
		EnvironmentIntegrationTests,
		AtmosphereAndWindWorkThroughInterfaces)
	{
		const environment::StandardAtmosphere
			standardAtmosphere;

		const environment::ConstantWindModel
			constantWind(
				math::Vector3(
					15.0,
					0.0,
					0.0));

		const environment::AtmosphereModel&
			atmosphere =
			standardAtmosphere;

		const environment::WindModel&
			wind =
			constantWind;

		const environment::AtmosphereState air =
			atmosphere.getAtmosphere(
				1000.0);

		const math::Vector3 windVelocity =
			wind.getWindVelocityN(
				1000.0,
				10.0);

		EXPECT_GT(
			air.getPressurePa(),
			0.0);

		EXPECT_DOUBLE_EQ(
			windVelocity.x,
			15.0);
	}

	TEST(
		EnvironmentIntegrationTests,
		TailwindReducesVehicleRelativeAirVelocity)
	{
		const environment::ConstantWindModel wind(
			math::Vector3(
				20.0,
				0.0,
				0.0));

		const math::Vector3 vehicleVelocity(
			100.0,
			0.0,
			0.0);

		const math::Vector3 windVelocity =
			wind.getWindVelocityN(
				1000.0,
				5.0);

		const math::Vector3 relativeVelocity =
			environment::
			computeRelativeAirVelocityN(
				vehicleVelocity,
				windVelocity);

		EXPECT_DOUBLE_EQ(
			relativeVelocity.x,
			80.0);

		EXPECT_DOUBLE_EQ(
			relativeVelocity.y,
			0.0);

		EXPECT_DOUBLE_EQ(
			relativeVelocity.z,
			0.0);
	}

	TEST(
		EnvironmentIntegrationTests,
		CrosswindCreatesLateralRelativeAirVelocity)
	{
		const environment::ConstantWindModel wind(
			math::Vector3(
				0.0,
				25.0,
				0.0));

		const math::Vector3 vehicleVelocity(
			100.0,
			0.0,
			0.0);

		const math::Vector3 relativeVelocity =
			environment::
			computeRelativeAirVelocityN(
				vehicleVelocity,
				wind.getWindVelocityN(
					3000.0,
					15.0));

		EXPECT_DOUBLE_EQ(
			relativeVelocity.x,
			100.0);

		EXPECT_DOUBLE_EQ(
			relativeVelocity.y,
			-25.0);
	}

	TEST(
		EnvironmentIntegrationTests,
		AtmosphericDensityFallsAsVehicleClimbs)
	{
		const environment::StandardAtmosphere atmosphere;

		const environment::AtmosphereState low =
			atmosphere.getAtmosphere(
				1000.0);

		const environment::AtmosphereState high =
			atmosphere.getAtmosphere(
				10000.0);

		EXPECT_GT(
			low.getDensityKgPerM3(),
			high.getDensityKgPerM3());

		EXPECT_GT(
			low.getPressurePa(),
			high.getPressurePa());
	}

	TEST(
		EnvironmentIntegrationTests,
		FullEnvironmentConditionIsDeterministic)
	{
		const environment::StandardAtmosphere atmosphere;

		const environment::ConstantWindModel wind(
			math::Vector3(
				10.0,
				-7.0,
				3.0));

		const environment::AtmosphereState airA =
			atmosphere.getAtmosphere(
				7500.0);

		const environment::AtmosphereState airB =
			atmosphere.getAtmosphere(
				7500.0);

		const math::Vector3 windA =
			wind.getWindVelocityN(
				7500.0,
				25.0);

		const math::Vector3 windB =
			wind.getWindVelocityN(
				7500.0,
				25.0);

		EXPECT_DOUBLE_EQ(
			airA.getTemperatureK(),
			airB.getTemperatureK());

		EXPECT_DOUBLE_EQ(
			airA.getPressurePa(),
			airB.getPressurePa());

		EXPECT_DOUBLE_EQ(
			airA.getDensityKgPerM3(),
			airB.getDensityKgPerM3());

		EXPECT_DOUBLE_EQ(
			airA.getSpeedOfSoundMps(),
			airB.getSpeedOfSoundMps());

		EXPECT_DOUBLE_EQ(
			windA.x,
			windB.x);

		EXPECT_DOUBLE_EQ(
			windA.y,
			windB.y);

		EXPECT_DOUBLE_EQ(
			windA.z,
			windB.z);
	}
}