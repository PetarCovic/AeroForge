#include <gtest/gtest.h>

#include <environment/AtmosphereModel.h>
#include <environment/StandardAtmosphere.h>

#include <limits>
#include <stdexcept>
#include <type_traits>

namespace aeroforge::environment::tests
{
	TEST(
		StandardAtmosphereTests,
		ImplementsAtmosphereModel)
	{
		EXPECT_TRUE(
			(std::is_base_of_v<
				AtmosphereModel,
				StandardAtmosphere>));
	}

	TEST(
		StandardAtmosphereTests,
		ReturnsSeaLevelAtmosphere)
	{
		const StandardAtmosphere atmosphere;

		const AtmosphereState state =
			atmosphere.getAtmosphere(0.0);

		EXPECT_NEAR(
			state.getTemperatureK(),
			288.15,
			1e-10);

		EXPECT_NEAR(
			state.getPressurePa(),
			101325.0,
			1e-6);

		EXPECT_NEAR(
			state.getDensityKgPerM3(),
			1.225,
			1e-5);

		EXPECT_NEAR(
			state.getSpeedOfSoundMps(),
			340.294,
			0.01);
	}

	TEST(
		StandardAtmosphereTests,
		AtmosphericPropertiesDecreaseThroughTroposphere)
	{
		const StandardAtmosphere atmosphere;

		const AtmosphereState seaLevel =
			atmosphere.getAtmosphere(0.0);

		const AtmosphereState fiveKm =
			atmosphere.getAtmosphere(5000.0);

		const AtmosphereState tenKm =
			atmosphere.getAtmosphere(10000.0);

		EXPECT_GT(
			seaLevel.getTemperatureK(),
			fiveKm.getTemperatureK());

		EXPECT_GT(
			fiveKm.getTemperatureK(),
			tenKm.getTemperatureK());

		EXPECT_GT(
			seaLevel.getPressurePa(),
			fiveKm.getPressurePa());

		EXPECT_GT(
			fiveKm.getPressurePa(),
			tenKm.getPressurePa());

		EXPECT_GT(
			seaLevel.getDensityKgPerM3(),
			fiveKm.getDensityKgPerM3());

		EXPECT_GT(
			fiveKm.getDensityKgPerM3(),
			tenKm.getDensityKgPerM3());
	}

	TEST(
		StandardAtmosphereTests,
		IsothermalLayerMaintainsTemperatureAboveElevenKm)
	{
		const StandardAtmosphere atmosphere;

		const AtmosphereState elevenKm =
			atmosphere.getAtmosphere(11000.0);

		const AtmosphereState fifteenKm =
			atmosphere.getAtmosphere(15000.0);

		const AtmosphereState twentyKm =
			atmosphere.getAtmosphere(20000.0);

		EXPECT_NEAR(
			elevenKm.getTemperatureK(),
			216.65,
			1e-10);

		EXPECT_NEAR(
			fifteenKm.getTemperatureK(),
			216.65,
			1e-10);

		EXPECT_NEAR(
			twentyKm.getTemperatureK(),
			216.65,
			1e-10);
	}

	TEST(
		StandardAtmosphereTests,
		RejectsNegativeAltitude)
	{
		const StandardAtmosphere atmosphere;

		EXPECT_THROW(
			atmosphere.getAtmosphere(-1.0),
			std::invalid_argument);
	}

	TEST(
		StandardAtmosphereTests,
		RejectsAltitudeAboveSupportedRange)
	{
		const StandardAtmosphere atmosphere;

		EXPECT_THROW(
			atmosphere.getAtmosphere(20000.0001),
			std::invalid_argument);
	}

	TEST(
		StandardAtmosphereTests,
		AcceptsMaximumSupportedAltitude)
	{
		const StandardAtmosphere atmosphere;

		EXPECT_NO_THROW(
			atmosphere.getAtmosphere(20000.0));
	}

	TEST(
		StandardAtmosphereTests,
		RejectsNaNAltitude)
	{
		const StandardAtmosphere atmosphere;

		EXPECT_THROW(
			atmosphere.getAtmosphere(
				std::numeric_limits<double>::
				quiet_NaN()),
			std::invalid_argument);
	}

	TEST(
		StandardAtmosphereTests,
		RejectsInfiniteAltitude)
	{
		const StandardAtmosphere atmosphere;

		EXPECT_THROW(
			atmosphere.getAtmosphere(
				std::numeric_limits<double>::
				infinity()),
			std::invalid_argument);
	}
}