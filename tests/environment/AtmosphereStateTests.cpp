#include <gtest/gtest.h>

#include <environment/AtmosphereState.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::environment::tests
{
	TEST(
		AtmosphereStateTests,
		StoresAtmosphericProperties)
	{
		const AtmosphereState state(
			288.15,
			101325.0,
			1.225,
			340.294);

		EXPECT_DOUBLE_EQ(
			state.getTemperatureK(),
			288.15);

		EXPECT_DOUBLE_EQ(
			state.getPressurePa(),
			101325.0);

		EXPECT_DOUBLE_EQ(
			state.getDensityKgPerM3(),
			1.225);

		EXPECT_DOUBLE_EQ(
			state.getSpeedOfSoundMps(),
			340.294);
	}

	TEST(
		AtmosphereStateTests,
		AcceptsPositiveFractionalValues)
	{
		const AtmosphereState state(
			200.5,
			5000.25,
			0.25,
			275.75);

		EXPECT_DOUBLE_EQ(
			state.getTemperatureK(),
			200.5);

		EXPECT_DOUBLE_EQ(
			state.getPressurePa(),
			5000.25);

		EXPECT_DOUBLE_EQ(
			state.getDensityKgPerM3(),
			0.25);

		EXPECT_DOUBLE_EQ(
			state.getSpeedOfSoundMps(),
			275.75);
	}

	TEST(
		AtmosphereStateTests,
		RejectsZeroTemperature)
	{
		EXPECT_THROW(
			AtmosphereState(
				0.0,
				101325.0,
				1.225,
				340.0),
			std::invalid_argument);
	}

	TEST(
		AtmosphereStateTests,
		RejectsZeroPressure)
	{
		EXPECT_THROW(
			AtmosphereState(
				288.15,
				0.0,
				1.225,
				340.0),
			std::invalid_argument);
	}

	TEST(
		AtmosphereStateTests,
		RejectsZeroDensity)
	{
		EXPECT_THROW(
			AtmosphereState(
				288.15,
				101325.0,
				0.0,
				340.0),
			std::invalid_argument);
	}

	TEST(
		AtmosphereStateTests,
		RejectsZeroSpeedOfSound)
	{
		EXPECT_THROW(
			AtmosphereState(
				288.15,
				101325.0,
				1.225,
				0.0),
			std::invalid_argument);
	}

	TEST(
		AtmosphereStateTests,
		RejectsNegativeProperties)
	{
		EXPECT_THROW(
			AtmosphereState(
				-1.0,
				101325.0,
				1.225,
				340.0),
			std::invalid_argument);

		EXPECT_THROW(
			AtmosphereState(
				288.15,
				-1.0,
				1.225,
				340.0),
			std::invalid_argument);

		EXPECT_THROW(
			AtmosphereState(
				288.15,
				101325.0,
				-1.0,
				340.0),
			std::invalid_argument);

		EXPECT_THROW(
			AtmosphereState(
				288.15,
				101325.0,
				1.225,
				-1.0),
			std::invalid_argument);
	}

	TEST(
		AtmosphereStateTests,
		RejectsNaNProperties)
	{
		const double nan =
			std::numeric_limits<double>::quiet_NaN();

		EXPECT_THROW(
			AtmosphereState(
				nan,
				101325.0,
				1.225,
				340.0),
			std::invalid_argument);

		EXPECT_THROW(
			AtmosphereState(
				288.15,
				nan,
				1.225,
				340.0),
			std::invalid_argument);

		EXPECT_THROW(
			AtmosphereState(
				288.15,
				101325.0,
				nan,
				340.0),
			std::invalid_argument);

		EXPECT_THROW(
			AtmosphereState(
				288.15,
				101325.0,
				1.225,
				nan),
			std::invalid_argument);
	}

	TEST(
		AtmosphereStateTests,
		RejectsInfiniteProperties)
	{
		const double infinity =
			std::numeric_limits<double>::infinity();

		EXPECT_THROW(
			AtmosphereState(
				infinity,
				101325.0,
				1.225,
				340.0),
			std::invalid_argument);

		EXPECT_THROW(
			AtmosphereState(
				288.15,
				infinity,
				1.225,
				340.0),
			std::invalid_argument);

		EXPECT_THROW(
			AtmosphereState(
				288.15,
				101325.0,
				infinity,
				340.0),
			std::invalid_argument);

		EXPECT_THROW(
			AtmosphereState(
				288.15,
				101325.0,
				1.225,
				infinity),
			std::invalid_argument);
	}
}