#include <gtest/gtest.h>

#include <environment/StandardAtmosphere.h>

namespace aeroforge::integration::tests
{
	TEST(
		AtmosphereValidationTests,
		MatchesReferenceAtFiveKilometers)
	{
		const environment::StandardAtmosphere model;

		const environment::AtmosphereState state =
			model.getAtmosphere(5000.0);

		EXPECT_NEAR(
			state.getTemperatureK(),
			255.65,
			1e-8);

		EXPECT_NEAR(
			state.getPressurePa(),
			54019.89,
			0.5);

		EXPECT_NEAR(
			state.getDensityKgPerM3(),
			0.736116,
			1e-5);

		EXPECT_NEAR(
			state.getSpeedOfSoundMps(),
			320.529,
			0.01);
	}

	TEST(
		AtmosphereValidationTests,
		MatchesReferenceAtElevenKilometers)
	{
		const environment::StandardAtmosphere model;

		const environment::AtmosphereState state =
			model.getAtmosphere(11000.0);

		EXPECT_NEAR(
			state.getTemperatureK(),
			216.65,
			1e-8);

		EXPECT_NEAR(
			state.getPressurePa(),
			22632.04,
			0.5);

		EXPECT_NEAR(
			state.getDensityKgPerM3(),
			0.363918,
			1e-5);

		EXPECT_NEAR(
			state.getSpeedOfSoundMps(),
			295.069,
			0.01);
	}

	TEST(
		AtmosphereValidationTests,
		MatchesReferenceAtFifteenKilometers)
	{
		const environment::StandardAtmosphere model;

		const environment::AtmosphereState state =
			model.getAtmosphere(15000.0);

		EXPECT_NEAR(
			state.getTemperatureK(),
			216.65,
			1e-8);

		EXPECT_NEAR(
			state.getPressurePa(),
			12044.55,
			0.5);

		EXPECT_NEAR(
			state.getDensityKgPerM3(),
			0.193673,
			1e-5);

		EXPECT_NEAR(
			state.getSpeedOfSoundMps(),
			295.069,
			0.01);
	}

	TEST(
		AtmosphereValidationTests,
		MatchesReferenceAtTwentyKilometers)
	{
		const environment::StandardAtmosphere model;

		const environment::AtmosphereState state =
			model.getAtmosphere(20000.0);

		EXPECT_NEAR(
			state.getTemperatureK(),
			216.65,
			1e-8);

		EXPECT_NEAR(
			state.getPressurePa(),
			5474.88,
			0.5);

		EXPECT_NEAR(
			state.getDensityKgPerM3(),
			0.088035,
			1e-5);

		EXPECT_NEAR(
			state.getSpeedOfSoundMps(),
			295.069,
			0.01);
	}

	TEST(
		AtmosphereValidationTests,
		IsContinuousAcrossElevenKilometerLayerBoundary)
	{
		const environment::StandardAtmosphere model;

		const environment::AtmosphereState below =
			model.getAtmosphere(10999.999);

		const environment::AtmosphereState above =
			model.getAtmosphere(11000.001);

		EXPECT_NEAR(
			below.getTemperatureK(),
			above.getTemperatureK(),
			1e-4);

		EXPECT_NEAR(
			below.getPressurePa(),
			above.getPressurePa(),
			0.1);

		EXPECT_NEAR(
			below.getDensityKgPerM3(),
			above.getDensityKgPerM3(),
			1e-5);

		EXPECT_NEAR(
			below.getSpeedOfSoundMps(),
			above.getSpeedOfSoundMps(),
			1e-3);
	}
}