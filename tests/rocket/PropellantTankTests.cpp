#include <gtest/gtest.h>

#include <rocket/PropellantTank.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::rocket::tests
{
	TEST(
		PropellantTankTests,
		StoresCapacityAndInitialMass)
	{
		const PropellantTank tank(
			10.0,
			7.5);

		EXPECT_DOUBLE_EQ(
			tank.getCapacityKg(),
			10.0);

		EXPECT_DOUBLE_EQ(
			tank.getPropellantMassKg(),
			7.5);
	}

	TEST(
		PropellantTankTests,
		ComputesPropellantFraction)
	{
		const PropellantTank tank(
			10.0,
			7.5);

		EXPECT_DOUBLE_EQ(
			tank.getPropellantFraction(),
			0.75);
	}

	TEST(
		PropellantTankTests,
		AllowsInitiallyEmptyTank)
	{
		const PropellantTank tank(
			10.0,
			0.0);

		EXPECT_TRUE(
			tank.isEmpty());

		EXPECT_DOUBLE_EQ(
			tank.getPropellantFraction(),
			0.0);
	}

	TEST(
		PropellantTankTests,
		FullTankHasFractionOne)
	{
		const PropellantTank tank(
			10.0,
			10.0);

		EXPECT_DOUBLE_EQ(
			tank.getPropellantFraction(),
			1.0);
	}

	TEST(
		PropellantTankTests,
		ConsumesRequestedMass)
	{
		PropellantTank tank(
			10.0,
			8.0);

		const double consumed =
			tank.consume(3.0);

		EXPECT_DOUBLE_EQ(
			consumed,
			3.0);

		EXPECT_DOUBLE_EQ(
			tank.getPropellantMassKg(),
			5.0);
	}

	TEST(
		PropellantTankTests,
		ConsumptionCannotMakeMassNegative)
	{
		PropellantTank tank(
			10.0,
			2.0);

		const double consumed =
			tank.consume(5.0);

		EXPECT_DOUBLE_EQ(
			consumed,
			2.0);

		EXPECT_DOUBLE_EQ(
			tank.getPropellantMassKg(),
			0.0);

		EXPECT_TRUE(
			tank.isEmpty());
	}

	TEST(
		PropellantTankTests,
		ConsumingZeroChangesNothing)
	{
		PropellantTank tank(
			10.0,
			5.0);

		EXPECT_DOUBLE_EQ(
			tank.consume(0.0),
			0.0);

		EXPECT_DOUBLE_EQ(
			tank.getPropellantMassKg(),
			5.0);
	}

	TEST(
		PropellantTankTests,
		RejectsNonPositiveCapacity)
	{
		EXPECT_THROW(
			PropellantTank(
				0.0,
				0.0),
			std::invalid_argument);

		EXPECT_THROW(
			PropellantTank(
				-1.0,
				0.0),
			std::invalid_argument);
	}

	TEST(
		PropellantTankTests,
		RejectsInvalidInitialMass)
	{
		EXPECT_THROW(
			PropellantTank(
				10.0,
				-1.0),
			std::invalid_argument);

		EXPECT_THROW(
			PropellantTank(
				10.0,
				11.0),
			std::invalid_argument);
	}

	TEST(
		PropellantTankTests,
		RejectsNonFiniteConstructionValues)
	{
		const double nan =
			std::numeric_limits<double>::quiet_NaN();

		const double infinity =
			std::numeric_limits<double>::infinity();

		EXPECT_THROW(
			PropellantTank(
				nan,
				0.0),
			std::invalid_argument);

		EXPECT_THROW(
			PropellantTank(
				10.0,
				infinity),
			std::invalid_argument);
	}

	TEST(
		PropellantTankTests,
		RejectsInvalidConsumption)
	{
		PropellantTank tank(
			10.0,
			5.0);

		EXPECT_THROW(
			tank.consume(-1.0),
			std::invalid_argument);

		EXPECT_THROW(
			tank.consume(
				std::numeric_limits<double>::
				infinity()),
			std::invalid_argument);
	}

	TEST(
		PropellantTankTests,
		FailedConsumptionDoesNotModifyTank)
	{
		PropellantTank tank(
			10.0,
			5.0);

		EXPECT_THROW(
			tank.consume(-1.0),
			std::invalid_argument);

		EXPECT_DOUBLE_EQ(
			tank.getPropellantMassKg(),
			5.0);
	}
}