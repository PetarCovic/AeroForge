#include <gtest/gtest.h>

#include <rocket/RocketMassProperties.h>

#include <math/Matrix.h>
#include <math/Vector3.h>

#include <limits>
#include <stdexcept>

namespace aeroforge::rocket::tests
{
	TEST(
		RocketMassPropertiesTests,
		ComputesTotalMass)
	{
		const RocketMassProperties properties(
			8.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Matrix(
				3,
				3,
				{
					2.0, 0.0, 0.0,
					0.0, 3.0, 0.0,
					0.0, 0.0, 4.0
				}),
			math::Vector3(1.0, 0.0, 0.0));

		EXPECT_DOUBLE_EQ(
			properties.getTotalMassKg(2.0),
			10.0);

		EXPECT_DOUBLE_EQ(
			properties.getTotalMassKg(0.0),
			8.0);
	}

	TEST(
		RocketMassPropertiesTests,
		CenterOfMassMovesWithPropellant)
	{
		const RocketMassProperties properties(
			8.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Matrix(
				3,
				3,
				{
					2.0, 0.0, 0.0,
					0.0, 3.0, 0.0,
					0.0, 0.0, 4.0
				}),
			math::Vector3(1.0, 0.0, 0.0));

		const math::Vector3 fullCenter =
			properties.getCenterOfMassB(2.0);

		EXPECT_NEAR(
			fullCenter.x,
			0.2,
			1e-12);

		EXPECT_NEAR(
			fullCenter.y,
			0.0,
			1e-12);

		EXPECT_NEAR(
			fullCenter.z,
			0.0,
			1e-12);

		const math::Vector3 emptyCenter =
			properties.getCenterOfMassB(0.0);

		EXPECT_NEAR(
			emptyCenter.x,
			0.0,
			1e-12);
	}

	TEST(
		RocketMassPropertiesTests,
		EmptyTankReturnsDryInertia)
	{
		const math::Matrix dryInertia(
			3,
			3,
			{
				2.0, 0.0, 0.0,
				0.0, 3.0, 0.0,
				0.0, 0.0, 4.0
			});

		const RocketMassProperties properties(
			8.0,
			math::Vector3(0.0, 0.0, 0.0),
			dryInertia,
			math::Vector3(1.0, 0.0, 0.0));

		const math::Matrix result =
			properties.getInertiaTensorBody(0.0);

		EXPECT_TRUE(
			result.isApprox(
				dryInertia,
				1e-12));
	}

	TEST(
		RocketMassPropertiesTests,
		PropellantChangesInertiaUsingParallelAxisTheorem)
	{
		const RocketMassProperties properties(
			8.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Matrix(
				3,
				3,
				{
					2.0, 0.0, 0.0,
					0.0, 3.0, 0.0,
					0.0, 0.0, 4.0
				}),
			math::Vector3(1.0, 0.0, 0.0));

		const math::Matrix inertia =
			properties.getInertiaTensorBody(2.0);

		EXPECT_NEAR(
			inertia(0, 0),
			2.0,
			1e-12);

		EXPECT_NEAR(
			inertia(1, 1),
			4.6,
			1e-12);

		EXPECT_NEAR(
			inertia(2, 2),
			5.6,
			1e-12);
	}

	TEST(
		RocketMassPropertiesTests,
		ResultingInertiaRemainsSymmetricAndPositiveDefinite)
	{
		const RocketMassProperties properties(
			8.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Matrix(
				3,
				3,
				{
					2.0, 0.1, 0.0,
					0.1, 3.0, 0.0,
					0.0, 0.0, 4.0
				}),
			math::Vector3(1.0, 0.2, 0.0));

		const math::Matrix inertia =
			properties.getInertiaTensorBody(2.0);

		EXPECT_TRUE(
			inertia.isSymmetric());

		EXPECT_TRUE(
			inertia.isPositiveDefinite());
	}

	TEST(
		RocketMassPropertiesTests,
		RejectsNonPositiveDryMass)
	{
		const math::Matrix inertia =
			math::Matrix::identity(3);

		EXPECT_THROW(
			(RocketMassProperties(
				0.0,
				math::Vector3(0.0, 0.0, 0.0),
				inertia,
				math::Vector3(0.0, 0.0, 0.0))),
			std::invalid_argument);
	}

	TEST(
		RocketMassPropertiesTests,
		RejectsInvalidDryInertia)
	{
		const math::Matrix inertia(
			3,
			3,
			{
				1.0, 0.0, 0.0,
				0.0, -1.0, 0.0,
				0.0, 0.0, 1.0
			});

		EXPECT_THROW(
			(RocketMassProperties(
				10.0,
				math::Vector3(0.0, 0.0, 0.0),
				inertia,
				math::Vector3(0.0, 0.0, 0.0))),
			std::invalid_argument);
	}

	TEST(
		RocketMassPropertiesTests,
		RejectsNegativePropellantMassQuery)
	{
		const RocketMassProperties properties(
			8.0,
			math::Vector3(0.0, 0.0, 0.0),
			math::Matrix::identity(3),
			math::Vector3(1.0, 0.0, 0.0));

		EXPECT_THROW(
			properties.getTotalMassKg(-1.0),
			std::invalid_argument);

		EXPECT_THROW(
			properties.getCenterOfMassB(-1.0),
			std::invalid_argument);

		EXPECT_THROW(
			properties.getInertiaTensorBody(-1.0),
			std::invalid_argument);
	}
}