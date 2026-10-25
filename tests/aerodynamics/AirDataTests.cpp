#include <gtest/gtest.h>

#include <aerodynamics/AirData.h>

#include <environment/AtmosphereState.h>

#include <math/Vector3.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::aerodynamics::tests
{
	TEST(
		AirDataTests,
		StoresValidAirData)
	{
		const AirData data(
			100.0,
			6125.0,
			0.2939);

		EXPECT_DOUBLE_EQ(
			data.getAirspeedMps(),
			100.0);

		EXPECT_DOUBLE_EQ(
			data.getDynamicPressurePa(),
			6125.0);

		EXPECT_DOUBLE_EQ(
			data.getMachNumber(),
			0.2939);
	}

	TEST(
		AirDataTests,
		AllowsZeroAirData)
	{
		const AirData data(
			0.0,
			0.0,
			0.0);

		EXPECT_DOUBLE_EQ(
			data.getAirspeedMps(),
			0.0);

		EXPECT_DOUBLE_EQ(
			data.getDynamicPressurePa(),
			0.0);

		EXPECT_DOUBLE_EQ(
			data.getMachNumber(),
			0.0);
	}

	TEST(
		AirDataTests,
		ComputesSeaLevelAirDataAtOneHundredMps)
	{
		const environment::AtmosphereState atmosphere(
			288.15,
			101325.0,
			1.225,
			340.294);

		const math::Vector3 velocity(
			100.0,
			0.0,
			0.0);

		const AirData data =
			computeAirData(
				atmosphere,
				velocity);

		EXPECT_NEAR(
			data.getAirspeedMps(),
			100.0,
			1e-12);

		EXPECT_NEAR(
			data.getDynamicPressurePa(),
			6125.0,
			1e-10);

		EXPECT_NEAR(
			data.getMachNumber(),
			100.0 / 340.294,
			1e-12);
	}

	TEST(
		AirDataTests,
		AirspeedUsesAllThreeVelocityComponents)
	{
		const environment::AtmosphereState atmosphere(
			288.15,
			101325.0,
			1.225,
			340.0);

		const math::Vector3 velocity(
			3.0,
			4.0,
			12.0);

		const AirData data =
			computeAirData(
				atmosphere,
				velocity);

		EXPECT_NEAR(
			data.getAirspeedMps(),
			13.0,
			1e-12);
	}

	TEST(
		AirDataTests,
		DynamicPressureScalesWithVelocitySquared)
	{
		const environment::AtmosphereState atmosphere(
			288.15,
			101325.0,
			1.225,
			340.0);

		const AirData slow =
			computeAirData(
				atmosphere,
				math::Vector3(
					50.0,
					0.0,
					0.0));

		const AirData fast =
			computeAirData(
				atmosphere,
				math::Vector3(
					100.0,
					0.0,
					0.0));

		EXPECT_NEAR(
			fast.getDynamicPressurePa(),
			4.0 * slow.getDynamicPressurePa(),
			1e-10);
	}

	TEST(
		AirDataTests,
		RejectsNegativeStoredValues)
	{
		EXPECT_THROW(
			(AirData{
				-1.0,
				0.0,
				0.0
				}),
			std::invalid_argument);

		EXPECT_THROW(
			(AirData{
				1.0,
				-1.0,
				0.0
				}),
			std::invalid_argument);

		EXPECT_THROW(
			(AirData{
				1.0,
				1.0,
				-1.0
				}),
			std::invalid_argument);
	}

	TEST(
		AirDataTests,
		RejectsNonFiniteStoredValues)
	{
		const double nan =
			std::numeric_limits<double>::quiet_NaN();

		const double infinity =
			std::numeric_limits<double>::infinity();

		EXPECT_THROW(
			(AirData{
				nan,
				1.0,
				1.0
				}),
			std::invalid_argument);

		EXPECT_THROW(
			(AirData{
				1.0,
				infinity,
				1.0
				}),
			std::invalid_argument);

		EXPECT_THROW(
			(AirData{
				1.0,
				1.0,
				nan
				}),
			std::invalid_argument);
	}

	TEST(
		AirDataTests,
		RejectsNonFiniteVelocityInput)
	{
		const environment::AtmosphereState atmosphere(
			288.15,
			101325.0,
			1.225,
			340.0);

		const math::Vector3 velocity(
			std::numeric_limits<double>::infinity(),
			0.0,
			0.0);

		EXPECT_THROW(
			computeAirData(
				atmosphere,
				velocity),
			std::invalid_argument);
	}
}