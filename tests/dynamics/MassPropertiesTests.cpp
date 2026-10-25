#include <gtest/gtest.h>

#include <dynamics/MassProperties.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::dynamics::tests
{
	TEST(
		MassPropertiesTests,
		StoresValidIntegerMass)
	{
		const MassProperties mass(
			100.0);

		EXPECT_DOUBLE_EQ(
			mass.getMassKg(),
			100.0);
	}

	TEST(
		MassPropertiesTests,
		StoresValidFractionalMass)
	{
		const MassProperties mass(
			12.75);

		EXPECT_DOUBLE_EQ(
			mass.getMassKg(),
			12.75);
	}

	TEST(
		MassPropertiesTests,
		AcceptsVerySmallPositiveMass)
	{
		const MassProperties mass(
			1e-9);

		EXPECT_DOUBLE_EQ(
			mass.getMassKg(),
			1e-9);
	}

	TEST(
		MassPropertiesTests,
		SetMassUpdatesStoredMass)
	{
		MassProperties mass(
			10.0);

		mass.setMassKg(
			25.5);

		EXPECT_DOUBLE_EQ(
			mass.getMassKg(),
			25.5);
	}

	TEST(
		MassPropertiesTests,
		SetMassAcceptsFractionalMass)
	{
		MassProperties mass(
			10.0);

		mass.setMassKg(
			3.14159);

		EXPECT_DOUBLE_EQ(
			mass.getMassKg(),
			3.14159);
	}

	TEST(
		MassPropertiesTests,
		ConstructorRejectsZeroMass)
	{
		EXPECT_THROW(
			MassProperties(
				0.0),
			std::invalid_argument);
	}

	TEST(
		MassPropertiesTests,
		ConstructorRejectsNegativeMass)
	{
		EXPECT_THROW(
			MassProperties(
				-1.0),
			std::invalid_argument);
	}

	TEST(
		MassPropertiesTests,
		ConstructorRejectsNaNMass)
	{
		EXPECT_THROW(
			MassProperties{
				std::numeric_limits<double>::quiet_NaN()
			},
			std::invalid_argument);
	}

	TEST(
		MassPropertiesTests,
		ConstructorRejectsPositiveInfinity)
	{
		EXPECT_THROW(
			MassProperties{
				std::numeric_limits<double>::infinity()
			},
			std::invalid_argument);
	}

	TEST(
		MassPropertiesTests,
		ConstructorRejectsNegativeInfinity)
	{
		EXPECT_THROW(
			MassProperties(
				-std::numeric_limits<double>::
				infinity()),
			std::invalid_argument);
	}

	TEST(
		MassPropertiesTests,
		SetMassRejectsZero)
	{
		MassProperties mass(
			10.0);

		EXPECT_THROW(
			mass.setMassKg(
				0.0),
			std::invalid_argument);
	}

	TEST(
		MassPropertiesTests,
		SetMassRejectsNegativeValue)
	{
		MassProperties mass(
			10.0);

		EXPECT_THROW(
			mass.setMassKg(
				-5.0),
			std::invalid_argument);
	}

	TEST(
		MassPropertiesTests,
		SetMassRejectsNaN)
	{
		MassProperties mass(
			10.0);

		EXPECT_THROW(
			mass.setMassKg(
				std::numeric_limits<double>::
				quiet_NaN()),
			std::invalid_argument);
	}

	TEST(
		MassPropertiesTests,
		SetMassRejectsPositiveInfinity)
	{
		MassProperties mass(
			10.0);

		EXPECT_THROW(
			mass.setMassKg(
				std::numeric_limits<double>::
				infinity()),
			std::invalid_argument);
	}

	TEST(
		MassPropertiesTests,
		SetMassRejectsNegativeInfinity)
	{
		MassProperties mass(
			10.0);

		EXPECT_THROW(
			mass.setMassKg(
				-std::numeric_limits<double>::
				infinity()),
			std::invalid_argument);
	}

	TEST(
		MassPropertiesTests,
		FailedSetterDoesNotModifyExistingMass)
	{
		MassProperties mass(
			42.5);

		EXPECT_THROW(
			mass.setMassKg(
				-100.0),
			std::invalid_argument);

		EXPECT_DOUBLE_EQ(
			mass.getMassKg(),
			42.5);
	}

	TEST(
		MassPropertiesTests,
		RepeatedValidUpdatesWork)
	{
		MassProperties mass(
			100.0);

		mass.setMassKg(
			75.0);

		mass.setMassKg(
			50.0);

		mass.setMassKg(
			25.0);

		EXPECT_DOUBLE_EQ(
			mass.getMassKg(),
			25.0);
	}
}